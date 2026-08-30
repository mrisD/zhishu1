#include "zhishu_robot_rest_api/rest_server.hpp"

#include <httplib.h>

#include <algorithm>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <functional>
#include <iomanip>
#include <initializer_list>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace zhishu_robot_rest_api
{
namespace
{

std::string timestamp_now()
{
  const auto now = std::chrono::system_clock::now();
  const auto value = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
  gmtime_r(&value, &utc);
  std::ostringstream stream;
  stream << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return stream.str();
}

std::string bearer_token(const httplib::Request & request)
{
  constexpr char kPrefix[] = "Bearer ";
  const auto authorization = request.get_header_value("Authorization");
  if (authorization.rfind(kPrefix, 0U) != 0U) {
    return "";
  }
  return authorization.substr(sizeof(kPrefix) - 1U);
}

std::size_t bounded_size(
  const std::string & raw,
  std::size_t default_value,
  std::size_t maximum)
{
  if (raw.empty()) {
    return default_value;
  }
  unsigned long long parsed = 0U;
  const auto conversion = std::from_chars(raw.data(), raw.data() + raw.size(), parsed, 10);
  if (conversion.ec != std::errc{} || conversion.ptr != raw.data() + raw.size() ||
    parsed == 0U || parsed > maximum ||
    parsed > std::numeric_limits<std::size_t>::max())
  {
    throw std::invalid_argument("numeric query parameter is outside its allowed range");
  }
  return static_cast<std::size_t>(parsed);
}

Json parse_json_body(const httplib::Request & request)
{
  const auto content_type = request.get_header_value("Content-Type");
  if (content_type.rfind("application/json", 0U) != 0U) {
    throw std::invalid_argument("Content-Type must be application/json");
  }
  auto body = Json::parse(request.body);
  if (!body.is_object()) {
    throw std::invalid_argument("request body must be a JSON object");
  }
  return body;
}

void reject_unknown_keys(
  const Json & body,
  std::initializer_list<std::string_view> allowed_keys)
{
  for (auto iterator = body.begin(); iterator != body.end(); ++iterator) {
    if (std::find(
        allowed_keys.begin(), allowed_keys.end(), std::string_view{iterator.key()}) ==
      allowed_keys.end())
    {
      throw std::invalid_argument("unknown JSON field: " + iterator.key());
    }
  }
}

std::size_t bounded_json_size(
  const Json & body,
  const char * key,
  std::size_t default_value,
  std::size_t maximum)
{
  if (!body.contains(key)) {
    return default_value;
  }
  const auto value = body.at(key).get<long long>();
  if (value <= 0 || static_cast<unsigned long long>(value) > maximum) {
    throw std::invalid_argument(std::string{key} + " is outside its allowed range");
  }
  return static_cast<std::size_t>(value);
}

}  // namespace

class RestServer::Impl
{
public:
  Impl(
    std::shared_ptr<RobotBridge> bridge,
    RestOptions options,
    std::string openapi_document)
  : bridge_(std::move(bridge)),
    options_(std::move(options)),
    openapi_document_(std::move(openapi_document)),
    started_at_(std::chrono::steady_clock::now())
  {
    if (!bridge_) {
      throw std::invalid_argument("RestServer requires a RobotBridge");
    }
    options_.validate();
    server_.set_payload_max_length(options_.max_request_bytes);
    server_.set_read_timeout(10, 0);
    server_.set_write_timeout(10, 0);
    register_routes();
  }

  bool listen()
  {
    return server_.listen(options_.host, static_cast<int>(options_.port));
  }

  void stop()
  {
    server_.stop();
  }

private:
  using Handler = std::function<ApiResult(const httplib::Request &)>;

  void register_routes()
  {
    server_.Options(R"(/api/v1/.*)", [this](
        const httplib::Request & request, httplib::Response & response) {
        apply_headers(request, response);
        response.set_header("Access-Control-Allow-Methods", "GET, POST, PUT, OPTIONS");
        response.set_header("Access-Control-Allow-Headers", "Authorization, Content-Type");
        response.status = 204;
      });

    server_.Get("/api/v1/health", [this](
        const httplib::Request & request, httplib::Response & response) {
        const auto uptime = std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::steady_clock::now() - started_at_).count();
        send(
          request, response,
          ApiResult::success(Json{{"status", "ok"}, {"ros_ok", rclcpp::ok()}, {"uptime_s", uptime}}));
      });
    server_.Get("/api/v1/openapi.json", [this](
        const httplib::Request & request, httplib::Response & response) {
        apply_headers(request, response);
        response.status = 200;
        response.set_content(openapi_document_, "application/json; charset=utf-8");
      });

    register_read("GET", "/api/v1/server", [this](const httplib::Request &) {
        return bridge_->server_info();
      });
    register_read("GET", "/api/v1/robot/status", [this](const httplib::Request &) {
        return bridge_->robot_status();
      });
    register_read("GET", "/api/v1/ethercat/status", [this](const httplib::Request &) {
        return bridge_->ethercat_status();
      });
    register_read("GET", "/api/v1/ros/nodes", [this](const httplib::Request &) {
        return bridge_->list_ros_nodes();
      });
    register_read("GET", "/api/v1/ros/topics", [this](const httplib::Request & request) {
        const auto filter = request.has_param("filter") ? request.get_param_value("filter") : "";
        const auto limit = bounded_size(
          request.has_param("limit") ? request.get_param_value("limit") : "", 200U, 500U);
        return bridge_->list_ros_topics(filter, limit);
      });
    register_read("GET", "/api/v1/logs", [this](const httplib::Request & request) {
        const auto limit = bounded_size(
          request.has_param("limit") ? request.get_param_value("limit") : "", 50U, 200U);
        return bridge_->list_recent_logs(limit);
      });
    register_read("POST", "/api/v1/logs/read", [this](const httplib::Request & request) {
        const auto body = parse_json_body(request);
        reject_unknown_keys(body, {"root", "path", "tail_lines"});
        return bridge_->read_log(
          body.at("root").get<std::string>(),
          body.at("path").get<std::string>(),
          bounded_json_size(body, "tail_lines", 200U, 2000U));
      });
    register_read("GET", "/api/v1/motions/current", [this](const httplib::Request &) {
        return bridge_->motion_status();
      });

    register_write("POST", "/api/v1/motions/named", [this](const httplib::Request & request) {
        const auto body = parse_json_body(request);
        reject_unknown_keys(body, {"group", "target", "confirmation"});
        return bridge_->move_named(
          body.at("group").get<std::string>(),
          body.at("target").get<std::string>(),
          body.at("confirmation").get<std::string>(),
          bearer_token(request));
      });
    register_write("POST", "/api/v1/motions/joints", [this](const httplib::Request & request) {
        const auto body = parse_json_body(request);
        reject_unknown_keys(body, {"group", "positions", "confirmation"});
        return bridge_->move_joints(
          body.at("group").get<std::string>(),
          body.at("positions").get<std::vector<double>>(),
          body.at("confirmation").get<std::string>(),
          bearer_token(request));
      });
    register_write("POST", "/api/v1/grippers", [this](const httplib::Request & request) {
        const auto body = parse_json_body(request);
        reject_unknown_keys(body, {"side", "position", "confirmation"});
        return bridge_->set_gripper(
          body.at("side").get<std::string>(),
          body.at("position").get<double>(),
          body.at("confirmation").get<std::string>(),
          bearer_token(request));
      });
    register_write("POST", "/api/v1/motions/stop", [this](const httplib::Request & request) {
        return bridge_->stop_motion(bearer_token(request));
      });
    register_write("POST", "/api/v1/faults/reset", [this](const httplib::Request & request) {
        const auto body = parse_json_body(request);
        reject_unknown_keys(body, {"arm", "confirmation"});
        return bridge_->reset_fault(
          body.at("arm").get<std::string>(),
          body.at("confirmation").get<std::string>(),
          bearer_token(request));
      });
    register_write("PUT", "/api/v1/motors/enabled", [this](const httplib::Request & request) {
        const auto body = parse_json_body(request);
        reject_unknown_keys(body, {"arm", "enabled", "confirmation"});
        return bridge_->set_motors_enabled(
          body.at("arm").get<std::string>(),
          body.at("enabled").get<bool>(),
          body.at("confirmation").get<std::string>(),
          bearer_token(request));
      });

    server_.set_error_handler([this](
        const httplib::Request & request, httplib::Response & response) {
        const int status = response.status;
        if (status == 404) {
          send(request, response, ApiResult::failure(
              "route_not_found", "REST route was not found", status));
        } else if (status == 413) {
          send(request, response, ApiResult::failure(
              "payload_too_large", "request body exceeds the configured limit", status));
        } else if (status == 405) {
          send(request, response, ApiResult::failure(
              "method_not_allowed", "HTTP method is not allowed for this route", status));
        } else {
          send(request, response, ApiResult::failure(
              "http_error", "HTTP request could not be processed", status));
        }
      });
  }

  void register_read(const std::string & method, const std::string & path, Handler handler)
  {
    const auto wrapped = [this, handler = std::move(handler)](
      const httplib::Request & request, httplib::Response & response) {
        std::string reason;
        if (!bridge_->policy().authorize_read(bearer_token(request), reason)) {
          send(request, response, ApiResult::failure("unauthorized", reason, 401));
          return;
        }
        execute(request, response, handler);
      };
    if (method == "GET") {
      server_.Get(path, wrapped);
    } else {
      server_.Post(path, wrapped);
    }
  }

  void register_write(const std::string & method, const std::string & path, Handler handler)
  {
    const auto wrapped = [this, handler = std::move(handler)](
      const httplib::Request & request, httplib::Response & response) {
        execute(request, response, handler);
      };
    if (method == "PUT") {
      server_.Put(path, wrapped);
    } else {
      server_.Post(path, wrapped);
    }
  }

  void execute(
    const httplib::Request & request,
    httplib::Response & response,
    const Handler & handler)
  {
    try {
      send(request, response, handler(request));
    } catch (const Json::exception & error) {
      send(
        request, response,
        ApiResult::failure("invalid_json", error.what(), 400));
    } catch (const std::invalid_argument & error) {
      send(
        request, response,
        ApiResult::failure("invalid_request", error.what(), 400));
    } catch (const std::exception & error) {
      std::fprintf(stderr, "zhishu_robot_rest_api handler error: %s\n", error.what());
      send(
        request, response,
        ApiResult::failure("internal_error", "internal server error", 500));
    }
  }

  void send(
    const httplib::Request & request,
    httplib::Response & response,
    ApiResult result)
  {
    const auto request_number = ++request_sequence_;
    const auto request_id = "rest-" + std::to_string(request_number);
    Json envelope{
      {"ok", result.ok},
      {"code", result.code},
      {"message", result.message},
      {"data", result.data},
      {"request_id", request_id},
      {"timestamp", timestamp_now()},
    };
    apply_headers(request, response);
    response.status = result.http_status;
    response.set_header("X-Request-ID", request_id);
    if (result.http_status == 401) {
      response.set_header("WWW-Authenticate", "Bearer realm=\"zhishu-robot-rest-api\"");
    }
    response.set_content(envelope.dump(), "application/json; charset=utf-8");
  }

  void apply_headers(const httplib::Request & request, httplib::Response & response) const
  {
    response.set_header("Cache-Control", "no-store");
    response.set_header("X-Content-Type-Options", "nosniff");
    if (!options_.cors_origin.empty() &&
      request.get_header_value("Origin") == options_.cors_origin)
    {
      response.set_header("Access-Control-Allow-Origin", options_.cors_origin);
      response.set_header("Vary", "Origin");
    }
  }

  std::shared_ptr<RobotBridge> bridge_;
  RestOptions options_;
  std::string openapi_document_;
  httplib::Server server_;
  std::chrono::steady_clock::time_point started_at_;
  std::atomic_uint64_t request_sequence_{0U};
};

RestServer::RestServer(
  std::shared_ptr<RobotBridge> bridge,
  RestOptions options,
  std::string openapi_document)
: impl_(std::make_unique<Impl>(
      std::move(bridge), std::move(options), std::move(openapi_document)))
{
}

RestServer::~RestServer() = default;

bool RestServer::listen()
{
  return impl_->listen();
}

void RestServer::stop()
{
  impl_->stop();
}

}  // namespace zhishu_robot_rest_api
