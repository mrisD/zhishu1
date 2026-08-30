#ifndef ZHISHU_ROBOT_REST_API__REST_SERVER_HPP_
#define ZHISHU_ROBOT_REST_API__REST_SERVER_HPP_

#include "zhishu_robot_rest_api/robot_bridge.hpp"
#include "zhishu_robot_rest_api/types.hpp"

#include <memory>
#include <string>

namespace zhishu_robot_rest_api
{

class RestServer
{
public:
  RestServer(
    std::shared_ptr<RobotBridge> bridge,
    RestOptions options,
    std::string openapi_document);
  ~RestServer();

  RestServer(const RestServer &) = delete;
  RestServer & operator=(const RestServer &) = delete;

  bool listen();
  void stop();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace zhishu_robot_rest_api

#endif  // ZHISHU_ROBOT_REST_API__REST_SERVER_HPP_
