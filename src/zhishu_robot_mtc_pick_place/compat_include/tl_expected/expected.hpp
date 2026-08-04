#pragma once

// Compatibility shim for ROS 2 Jazzy installations that still ship the
// deprecated <tl_expected/expected.hpp> wrapper. MoveIt/parameter_traits
// includes that legacy path transitively. Resolve it to the maintained copy
// bundled by rcpputils without changing files under /opt/ros.
#include <rcpputils/tl_expected/expected.hpp>
