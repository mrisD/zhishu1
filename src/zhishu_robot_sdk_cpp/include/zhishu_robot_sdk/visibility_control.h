#ifndef ZHISHU_ROBOT_SDK__VISIBILITY_CONTROL_H_
#define ZHISHU_ROBOT_SDK__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef ZHISHU_ROBOT_SDK_BUILDING_DLL
    #define ZHISHU_ROBOT_SDK_PUBLIC __declspec(dllexport)
  #else
    #define ZHISHU_ROBOT_SDK_PUBLIC __declspec(dllimport)
  #endif
  #define ZHISHU_ROBOT_SDK_LOCAL
#else
  #define ZHISHU_ROBOT_SDK_PUBLIC __attribute__((visibility("default")))
  #define ZHISHU_ROBOT_SDK_LOCAL  __attribute__((visibility("hidden")))
#endif

#endif  // ZHISHU_ROBOT_SDK__VISIBILITY_CONTROL_H_
