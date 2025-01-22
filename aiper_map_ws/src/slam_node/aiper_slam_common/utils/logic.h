#pragma once

namespace slam_utils {
enum TaskStatus {
  INITIAL = -1,
  FAILED = 0,
  SUCCESS = 1,
  RUNNING = 2,
  STOP = 3
};
}  // namespace slam_utils