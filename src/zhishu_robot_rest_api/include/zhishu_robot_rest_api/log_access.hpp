#ifndef ZHISHU_ROBOT_REST_API__LOG_ACCESS_HPP_
#define ZHISHU_ROBOT_REST_API__LOG_ACCESS_HPP_

#include "zhishu_robot_rest_api/types.hpp"

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace zhishu_robot_rest_api
{

struct LogRoot
{
  std::string name;
  std::filesystem::path path;
};

class LogAccess
{
public:
  explicit LogAccess(std::vector<LogRoot> roots);

  ApiResult list_recent(std::size_t limit) const;
  ApiResult read_tail(
    const std::string & root_name,
    const std::filesystem::path & relative_path,
    std::size_t tail_lines,
    std::size_t max_bytes = 256U * 1024U) const;
  Json roots_summary() const;

private:
  const LogRoot * find_root(const std::string & name) const;
  static bool allowed_extension(const std::filesystem::path & path);
  static bool is_within(
    const std::filesystem::path & child,
    const std::filesystem::path & parent);

  std::vector<LogRoot> roots_;
};

}  // namespace zhishu_robot_rest_api

#endif  // ZHISHU_ROBOT_REST_API__LOG_ACCESS_HPP_
