#include "zhishu_robot_rest_api/log_access.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <system_error>
#include <utility>

namespace zhishu_robot_rest_api
{
namespace
{

std::string format_time(std::filesystem::file_time_type file_time)
{
  const auto system_time = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
    file_time - std::filesystem::file_time_type::clock::now() +
    std::chrono::system_clock::now());
  const auto value = std::chrono::system_clock::to_time_t(system_time);
  std::tm utc{};
  gmtime_r(&value, &utc);
  std::ostringstream stream;
  stream << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return stream.str();
}

struct LogEntry
{
  std::string root;
  std::string relative_path;
  std::uintmax_t size{0U};
  std::filesystem::file_time_type modified_at{};
};

}  // namespace

LogAccess::LogAccess(std::vector<LogRoot> roots)
: roots_(std::move(roots))
{
  for (auto & root : roots_) {
    root.path = root.path.lexically_normal();
  }
}

ApiResult LogAccess::list_recent(std::size_t limit) const
{
  limit = std::clamp<std::size_t>(limit, 1U, 200U);
  std::vector<LogEntry> entries;
  for (const auto & root : roots_) {
    std::error_code error;
    if (!std::filesystem::is_directory(root.path, error)) {
      continue;
    }
    const auto canonical_root = std::filesystem::weakly_canonical(root.path, error);
    if (error) {
      continue;
    }
    const auto options = std::filesystem::directory_options::skip_permission_denied;
    std::filesystem::recursive_directory_iterator iterator(root.path, options, error);
    const std::filesystem::recursive_directory_iterator end;
    while (!error && iterator != end) {
      const auto & entry = *iterator;
      const auto canonical_entry = std::filesystem::weakly_canonical(entry.path(), error);
      if (!error && is_within(canonical_entry, canonical_root) &&
        entry.is_regular_file(error) && !error && allowed_extension(entry.path()))
      {
        LogEntry result;
        result.root = root.name;
        result.relative_path =
          std::filesystem::relative(entry.path(), root.path, error).generic_string();
        if (!error) {
          result.size = entry.file_size(error);
        }
        if (!error) {
          result.modified_at = entry.last_write_time(error);
        }
        if (!error) {
          entries.push_back(std::move(result));
        }
      }
      error.clear();
      iterator.increment(error);
    }
  }

  std::sort(
    entries.begin(), entries.end(),
    [](const LogEntry & left, const LogEntry & right) {
      return left.modified_at > right.modified_at;
    });
  if (entries.size() > limit) {
    entries.resize(limit);
  }

  Json files = Json::array();
  for (const auto & entry : entries) {
    files.push_back(Json{
      {"root", entry.root},
      {"path", entry.relative_path},
      {"size_bytes", entry.size},
      {"modified_at", format_time(entry.modified_at)},
    });
  }
  return ApiResult::success(
    Json{{"roots", roots_summary()}, {"files", files}, {"count", files.size()}});
}

ApiResult LogAccess::read_tail(
  const std::string & root_name,
  const std::filesystem::path & relative_path,
  std::size_t tail_lines,
  std::size_t max_bytes) const
{
  const auto * root = find_root(root_name);
  if (root == nullptr) {
    return ApiResult::failure("invalid_log_root", "unknown log root", 400);
  }
  if (relative_path.empty() || relative_path.is_absolute()) {
    return ApiResult::failure("invalid_log_path", "log path must be relative", 400);
  }

  std::error_code error;
  const auto canonical_root = std::filesystem::weakly_canonical(root->path, error);
  if (error) {
    return ApiResult::failure("log_root_unavailable", error.message(), 503);
  }
  const auto candidate = std::filesystem::weakly_canonical(root->path / relative_path, error);
  if (error || !is_within(candidate, canonical_root)) {
    return ApiResult::failure("log_path_denied", "path escapes configured log root", 403);
  }
  if (!std::filesystem::is_regular_file(candidate, error) || error ||
    !allowed_extension(candidate))
  {
    return ApiResult::failure("log_not_found", "log file is unavailable", 404);
  }

  const auto file_size = std::filesystem::file_size(candidate, error);
  if (error) {
    return ApiResult::failure("log_read_failed", error.message(), 500);
  }
  tail_lines = std::clamp<std::size_t>(tail_lines, 1U, 2000U);
  max_bytes = std::clamp<std::size_t>(max_bytes, 1024U, 1024U * 1024U);

  std::ifstream stream(candidate, std::ios::binary);
  if (!stream) {
    return ApiResult::failure("log_read_failed", "cannot open log file", 500);
  }
  const auto start_offset = file_size > max_bytes ? file_size - max_bytes : 0U;
  stream.seekg(static_cast<std::streamoff>(start_offset));
  if (start_offset > 0U) {
    std::string discarded;
    std::getline(stream, discarded);
  }

  std::vector<std::string> lines;
  std::string line;
  while (std::getline(stream, line)) {
    lines.push_back(line);
  }
  const bool truncated = start_offset > 0U || lines.size() > tail_lines;
  const auto first = lines.size() > tail_lines ? lines.size() - tail_lines : 0U;
  std::ostringstream text;
  for (std::size_t index = first; index < lines.size(); ++index) {
    text << lines[index];
    if (index + 1U < lines.size()) {
      text << '\n';
    }
  }

  return ApiResult::success(Json{
    {"root", root_name},
    {"path", std::filesystem::relative(candidate, canonical_root).generic_string()},
    {"size_bytes", file_size},
    {"returned_lines", lines.size() - first},
    {"truncated", truncated},
    {"text", text.str()},
  });
}

Json LogAccess::roots_summary() const
{
  Json result = Json::array();
  for (const auto & root : roots_) {
    std::error_code error;
    result.push_back(Json{
      {"name", root.name},
      {"path", root.path.string()},
      {"available", std::filesystem::is_directory(root.path, error) && !error},
    });
  }
  return result;
}

const LogRoot * LogAccess::find_root(const std::string & name) const
{
  const auto iterator = std::find_if(
    roots_.begin(), roots_.end(),
    [&name](const LogRoot & root) {return root.name == name;});
  return iterator == roots_.end() ? nullptr : &*iterator;
}

bool LogAccess::allowed_extension(const std::filesystem::path & path)
{
  const auto extension = path.extension().string();
  return extension == ".log" || extension == ".txt";
}

bool LogAccess::is_within(
  const std::filesystem::path & child,
  const std::filesystem::path & parent)
{
  auto child_iterator = child.begin();
  for (auto parent_iterator = parent.begin(); parent_iterator != parent.end();
    ++parent_iterator, ++child_iterator)
  {
    if (child_iterator == child.end() || *child_iterator != *parent_iterator) {
      return false;
    }
  }
  return true;
}

}  // namespace zhishu_robot_rest_api
