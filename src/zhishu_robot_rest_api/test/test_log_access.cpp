#include "zhishu_robot_rest_api/log_access.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace zhishu_robot_rest_api
{
namespace
{

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    const auto suffix = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
      ("zhishu_robot_rest_api_test_" + std::to_string(suffix));
    std::filesystem::create_directories(path_ / "nested");
  }

  ~TemporaryDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  const std::filesystem::path & path() const
  {
    return path_;
  }

private:
  std::filesystem::path path_;
};

void write_text(const std::filesystem::path & path, const std::string & text)
{
  std::ofstream stream(path);
  ASSERT_TRUE(stream.good());
  stream << text;
}

}  // namespace

TEST(LogAccess, ListsOnlySupportedLogExtensions)
{
  TemporaryDirectory temporary;
  write_text(temporary.path() / "nested" / "robot.log", "one\ntwo\n");
  write_text(temporary.path() / "ignored.bin", "not a log");
  LogAccess access({LogRoot{"robot", temporary.path()}});

  const auto result = access.list_recent(10U);
  ASSERT_TRUE(result.ok);
  ASSERT_EQ(result.data.at("files").size(), 1U);
  EXPECT_EQ(result.data.at("files").at(0).at("path"), "nested/robot.log");
}

TEST(LogAccess, ReadsBoundedTail)
{
  TemporaryDirectory temporary;
  write_text(temporary.path() / "robot.txt", "one\ntwo\nthree\nfour\n");
  LogAccess access({LogRoot{"robot", temporary.path()}});

  const auto result = access.read_tail("robot", "robot.txt", 2U);
  ASSERT_TRUE(result.ok);
  EXPECT_EQ(result.data.at("returned_lines"), 2U);
  EXPECT_EQ(result.data.at("text"), "three\nfour");
  EXPECT_TRUE(result.data.at("truncated"));
}

TEST(LogAccess, RejectsPathTraversal)
{
  TemporaryDirectory temporary;
  const auto outside = temporary.path().parent_path() /
    (temporary.path().filename().string() + "_outside.log");
  write_text(outside, "outside");
  LogAccess access({LogRoot{"robot", temporary.path()}});

  const auto result = access.read_tail(
    "robot", "../" + outside.filename().string(), 10U);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.code, "log_path_denied");

  std::error_code error;
  std::filesystem::remove(outside, error);
}

}  // namespace zhishu_robot_rest_api
