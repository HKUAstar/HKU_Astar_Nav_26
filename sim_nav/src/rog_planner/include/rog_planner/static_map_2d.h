#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Core>
#include <cstdint>
#include <mutex>
#include <vector>

namespace rog_planner {

struct StaticMapParams {
  int occ_threshold = 65;
  double inflation_radius = 0.05;
  bool unknown_as_occupied = false;
  bool outside_as_occupied = true;
};

class StaticMap2D {
public:
  explicit StaticMap2D(const StaticMapParams& params);

  void update(const nav_msgs::OccupancyGrid& msg);
  bool ready() const;

  bool isOccupied(const Eigen::Vector2d& p) const;
  double clearance(const Eigen::Vector2d& p, double max_dist,
                   Eigen::Vector2d* grad) const;

private:
  bool worldToMapUnlocked(const Eigen::Vector2d& p, int& mx, int& my) const;
  Eigen::Vector2d mapToWorldUnlocked(int mx, int my) const;
  bool isBlockedValue(int8_t value) const;
  bool inBounds(int mx, int my) const;
  size_t index(int mx, int my) const;

  StaticMapParams params_;
  mutable std::mutex mtx_;
  bool ready_ = false;
  nav_msgs::MapMetaData info_;
  double yaw_ = 0.0;
  double cos_yaw_ = 1.0;
  double sin_yaw_ = 0.0;
  std::vector<uint8_t> occupied_;
  std::vector<uint8_t> inflated_;
};

}  // namespace rog_planner