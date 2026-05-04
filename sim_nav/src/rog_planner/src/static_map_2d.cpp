#include <rog_planner/static_map_2d.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <ros/console.h>

namespace rog_planner {

StaticMap2D::StaticMap2D(const StaticMapParams& params) : params_(params) {}

bool StaticMap2D::inBounds(int mx, int my) const {
  return mx >= 0 && my >= 0 &&
         mx < static_cast<int>(info_.width) &&
         my < static_cast<int>(info_.height);
}

size_t StaticMap2D::index(int mx, int my) const {
  return static_cast<size_t>(my) * info_.width + static_cast<size_t>(mx);
}

bool StaticMap2D::isBlockedValue(int8_t value) const {
  if (value < 0) return params_.unknown_as_occupied;
  return value >= params_.occ_threshold;
}

bool StaticMap2D::worldToMapUnlocked(const Eigen::Vector2d& p,
                                     int& mx, int& my) const {
  const double dx = p.x() - info_.origin.position.x;
  const double dy = p.y() - info_.origin.position.y;
  const double local_x =  cos_yaw_ * dx + sin_yaw_ * dy;
  const double local_y = -sin_yaw_ * dx + cos_yaw_ * dy;
  mx = static_cast<int>(std::floor(local_x / info_.resolution));
  my = static_cast<int>(std::floor(local_y / info_.resolution));
  return inBounds(mx, my);
}

Eigen::Vector2d StaticMap2D::mapToWorldUnlocked(int mx, int my) const {
  const double local_x = (mx + 0.5) * info_.resolution;
  const double local_y = (my + 0.5) * info_.resolution;
  return Eigen::Vector2d(
      info_.origin.position.x + cos_yaw_ * local_x - sin_yaw_ * local_y,
      info_.origin.position.y + sin_yaw_ * local_x + cos_yaw_ * local_y);
}

void StaticMap2D::update(const nav_msgs::OccupancyGrid& msg) {
  if (msg.info.width == 0 || msg.info.height == 0 || msg.info.resolution <= 0.0) {
    ROS_WARN_THROTTLE(2.0, "[rog_planner/static_map] invalid OccupancyGrid");
    return;
  }

  std::lock_guard<std::mutex> lck(mtx_);
  info_ = msg.info;
  const auto& q = info_.origin.orientation;
  yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                   1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  cos_yaw_ = std::cos(yaw_);
  sin_yaw_ = std::sin(yaw_);

  const size_t total = static_cast<size_t>(info_.width) * info_.height;
  occupied_.assign(total, 0);
  inflated_.assign(total, 0);

  for (int y = 0; y < static_cast<int>(info_.height); ++y) {
    for (int x = 0; x < static_cast<int>(info_.width); ++x) {
      const size_t addr = index(x, y);
      if (addr < msg.data.size() && isBlockedValue(msg.data[addr])) {
        occupied_[addr] = 1;
      }
    }
  }

  inflated_ = occupied_;
  const int radius_cells = static_cast<int>(std::ceil(
      std::max(0.0, params_.inflation_radius) / info_.resolution));
  if (radius_cells > 0) {
    const double radius_sq = params_.inflation_radius * params_.inflation_radius;
    for (int y = 0; y < static_cast<int>(info_.height); ++y) {
      for (int x = 0; x < static_cast<int>(info_.width); ++x) {
        if (!occupied_[index(x, y)]) continue;
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
          for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            const int nx = x + dx;
            const int ny = y + dy;
            if (!inBounds(nx, ny)) continue;
            const double dist_sq = (dx * dx + dy * dy) *
                                   info_.resolution * info_.resolution;
            if (dist_sq <= radius_sq + 1e-9) inflated_[index(nx, ny)] = 1;
          }
        }
      }
    }
  }

  ready_ = true;
  ROS_INFO_THROTTLE(5.0,
                    "[rog_planner/static_map] loaded %ux%u grid, res=%.3f, inflation=%.2fm",
                    info_.width, info_.height, info_.resolution,
                    params_.inflation_radius);
}

bool StaticMap2D::ready() const {
  std::lock_guard<std::mutex> lck(mtx_);
  return ready_;
}

bool StaticMap2D::isOccupied(const Eigen::Vector2d& p) const {
  std::lock_guard<std::mutex> lck(mtx_);
  if (!ready_) return false;
  int mx = 0, my = 0;
  if (!worldToMapUnlocked(p, mx, my)) return params_.outside_as_occupied;
  return inflated_[index(mx, my)] != 0;
}

double StaticMap2D::clearance(const Eigen::Vector2d& p, double max_dist,
                              Eigen::Vector2d* grad) const {
  if (grad) *grad = Eigen::Vector2d::Zero();
  std::lock_guard<std::mutex> lck(mtx_);
  if (!ready_ || max_dist <= 0.0) return max_dist;

  int mx = 0, my = 0;
  if (!worldToMapUnlocked(p, mx, my)) {
    if (!params_.outside_as_occupied) return max_dist;
    Eigen::Vector2d center = mapToWorldUnlocked(
        static_cast<int>(info_.width) / 2,
        static_cast<int>(info_.height) / 2);
    Eigen::Vector2d inward = center - p;
    if (grad && inward.norm() > 1e-9) *grad = -inward.normalized();
    return 0.0;
  }

  if (inflated_[index(mx, my)]) {
    if (grad) *grad = Eigen::Vector2d(1.0, 0.0);
    return 0.0;
  }

  const int radius_cells = static_cast<int>(std::ceil(max_dist / info_.resolution));
  double best_sq = max_dist * max_dist;
  bool found = false;
  Eigen::Vector2d nearest = p;
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const int nx = mx + dx;
      const int ny = my + dy;
      if (!inBounds(nx, ny) || !inflated_[index(nx, ny)]) continue;
      Eigen::Vector2d c = mapToWorldUnlocked(nx, ny);
      double dsq = (p - c).squaredNorm();
      if (dsq < best_sq) {
        best_sq = dsq;
        nearest = c;
        found = true;
      }
    }
  }

  if (!found) return max_dist;
  const double d = std::sqrt(best_sq);
  if (grad) {
    Eigen::Vector2d away = p - nearest;
    *grad = away.norm() > 1e-9 ? away.normalized() : Eigen::Vector2d(1.0, 0.0);
  }
  return d;
}

}  // namespace rog_planner