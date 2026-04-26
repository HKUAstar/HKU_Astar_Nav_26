// rog_map_to_grid_node.cpp
//
// Stage 2 bridge: ROG-Map inflated occupancy cloud + /odom  ->  nav_msgs::OccupancyGrid
// for D*-Lite's `dynamic_map_topic_name` slot.
//
// Subscribes:
//   ~cloud_in  (sensor_msgs/PointCloud2, default remap to /rog_map_node/rog_map/inf_occ)
//   ~odom_in   (nav_msgs/Odometry,        default remap to /odom)
//
// Publishes:
//   ~grid_out  (nav_msgs/OccupancyGrid,   default remap to /rog_grid)
//
// Algorithm (per publish trigger: cloud_cb OR 1 Hz timer):
//   1. Snapshot latest robot xy from odom and the cached cloud under mtx.
//   2. Build grid OUTSIDE the critical section.
//   3. Origin snapped to the world-anchored 0.2 m voxel lattice so each
//      ROG-Map voxel paints a clean 2x2 cell square.
//   4. Stale-cloud (>cloud_timeout) path publishes an all-zero grid; D*-Lite
//      merges via max(0, static) so the static map is preserved.
//
// Pre-launch guard: waitForMessage on cloud_topic with 5 s timeout. If Stage 1
// (rog_map_node) is not running, ROS_FATAL and exit cleanly. Bridge launch sets
// required="false" so this does not bring down dstarlite/hdl_localization.
//
// Threading: ros::AsyncSpinner(1). Single worker thread serializes ALL
// callbacks (cloud, odom, timer). Mutex is defensive only.

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <cmath>
#include <mutex>
#include <string>

namespace {

class RogMapToGridBridge {
public:
  RogMapToGridBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param("cloud_timeout", cloud_timeout_, 0.5);
    pnh.param("z_min",         z_min_,         0.05);
    pnh.param("z_max",         z_max_,         2.0);
    // res MUST equal the static map (PGM) resolution. Mismatch creates a
    // striped projection in dstarlite's internal grid (every other cell left
    // untouched), letting paths slip through obstacles.
    pnh.param("res",           res_,           0.05);
    pnh.param("window_m",      window_m_,      10.0);
    pnh.param("voxel_lattice", voxel_lattice_, 0.2);   // ROG-Map inflation_resolution
    pnh.param("heartbeat_hz",  heartbeat_hz_,  1.0);
    // Soft inflation rim painted with value `soft_value` around each voxel.
    // 0 disables. Adds extra deterrent cost to dstar so paths don't graze.
    pnh.param("soft_rim_cells", soft_rim_cells_, 2);
    pnh.param("soft_rim_value", soft_rim_value_, 60);

    paint_n_ = std::max(1, static_cast<int>(std::round(voxel_lattice_ / res_)));
    N_ = static_cast<int>(std::round(window_m_ / res_));
    if (N_ <= 0 || res_ <= 0 || voxel_lattice_ <= 0) {
      ROS_FATAL("[rog_map_to_grid_node] invalid params: res=%.3f window_m=%.3f N=%d",
                res_, window_m_, N_);
      ros::shutdown();
      return;
    }

    cloud_sub_ = nh.subscribe("cloud_in", 1, &RogMapToGridBridge::cloudCb, this);
    odom_sub_  = nh.subscribe("odom_in",  10, &RogMapToGridBridge::odomCb,  this);
    grid_pub_  = nh.advertise<nav_msgs::OccupancyGrid>("grid_out", 1);

    timer_ = nh.createTimer(ros::Duration(1.0 / heartbeat_hz_),
                            &RogMapToGridBridge::timerCb, this);

    ROS_INFO("[rog_map_to_grid_node] ready: %dx%d cells @ %.3f m, window=%.1f m, "
             "paint=%dx%d, soft_rim=%d@%d, z=[%.2f,%.2f], cloud_timeout=%.2fs",
             N_, N_, res_, window_m_, paint_n_, paint_n_,
             soft_rim_cells_, soft_rim_value_, z_min_, z_max_, cloud_timeout_);
  }

private:
  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg) {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      last_cloud_       = msg;
      last_cloud_stamp_ = msg->header.stamp;
    }
    publishGrid(/*cloud_driven=*/true);
  }

  void odomCb(const nav_msgs::OdometryConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    last_robot_x_ = msg->pose.pose.position.x;
    last_robot_y_ = msg->pose.pose.position.y;
    have_odom_    = true;
  }

  void timerCb(const ros::TimerEvent&) {
    publishGrid(/*cloud_driven=*/false);
  }

  void publishGrid(bool cloud_driven) {
    // Snapshot under lock.
    sensor_msgs::PointCloud2ConstPtr cloud_ptr;
    ros::Time cloud_stamp;
    double rx, ry;
    bool   have_odom;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      cloud_ptr   = last_cloud_;
      cloud_stamp = last_cloud_stamp_;
      rx          = last_robot_x_;
      ry          = last_robot_y_;
      have_odom   = have_odom_;

      // Cloud-driven dedup: if same stamp as last published, skip.
      if (cloud_driven && cloud_ptr && cloud_stamp == last_published_stamp_) {
        return;
      }
    }

    if (!have_odom) {
      // Bridge waits for first odom; publish nothing until we know where the robot is.
      return;
    }

    const ros::Time now = ros::Time::now();
    const bool stale = (!cloud_ptr) ||
                       ((now - cloud_stamp).toSec() > cloud_timeout_);

    // Origin snapped to the WORLD-anchored voxel lattice (default 0.2 m).
    // ROG-Map's posToGlobalIndex uses round(pos/res) with no local-origin
    // offset, so voxel centers sit on multiples of voxel_lattice_ in world
    // coords. Snapping the bridge origin to the same lattice guarantees
    // (vc - origin)/res_ is an integer and the 2x2 paint covers exactly
    // [vc - res_, vc + res_] on each axis.
    const double half = window_m_ * 0.5;
    const double origin_x = std::floor((rx - half) / voxel_lattice_) * voxel_lattice_;
    const double origin_y = std::floor((ry - half) / voxel_lattice_) * voxel_lattice_;

    nav_msgs::OccupancyGrid g;
    g.header.frame_id = "world";   // ROG-Map hardcodes "world" in its viz cloud
                                   // headers; Stage 1's static map<->world TF
                                   // shim makes this transformable to "map".
    g.header.stamp = stale ? now : cloud_stamp;
    g.info.map_load_time = g.header.stamp;
    g.info.resolution = static_cast<float>(res_);
    g.info.width  = static_cast<uint32_t>(N_);
    g.info.height = static_cast<uint32_t>(N_);
    g.info.origin.position.x = origin_x;
    g.info.origin.position.y = origin_y;
    g.info.origin.position.z = 0.0;
    g.info.origin.orientation.w = 1.0;  // identity
    g.data.assign(static_cast<size_t>(N_) * N_, 0);

    if (!stale) {
      paintCloud(*cloud_ptr, origin_x, origin_y, g.data);
    }

    grid_pub_.publish(g);

    if (!stale) {
      std::lock_guard<std::mutex> lk(mtx_);
      last_published_stamp_ = cloud_stamp;
    }
  }

  void paintCloud(const sensor_msgs::PointCloud2& cloud,
                  double origin_x, double origin_y,
                  std::vector<int8_t>& data) const {
    sensor_msgs::PointCloud2ConstIterator<float> ix(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(cloud, "z");
    const int N = N_;
    const double res = res_;
    const int paint_n = paint_n_;
    const int lo = -paint_n / 2;            // e.g. paint_n=4 -> lo=-2, hi=1
    const int hi = lo + paint_n - 1;
    const int rim = soft_rim_cells_;
    const int soft = soft_rim_value_;
    const int rim_lo = lo - rim;
    const int rim_hi = hi + rim;
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const float z = *iz;
      if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(z)) continue;
      if (z < z_min_ || z > z_max_) continue;
      // Origin is snapped so (vc - origin)/res is an integer for vc on the
      // voxel lattice. round() picks the nearest cell boundary.
      const int cx = static_cast<int>(std::lround((*ix - origin_x) / res));
      const int cy = static_cast<int>(std::lround((*iy - origin_y) / res));
      // Soft rim first (lower priority — overwritten by hard 100 below).
      if (rim > 0) {
        for (int dy = rim_lo; dy <= rim_hi; ++dy) {
          const int y = cy + dy;
          if (y < 0 || y >= N) continue;
          const bool y_in_core = (dy >= lo && dy <= hi);
          for (int dx = rim_lo; dx <= rim_hi; ++dx) {
            const int x = cx + dx;
            if (x < 0 || x >= N) continue;
            const bool x_in_core = (dx >= lo && dx <= hi);
            if (y_in_core && x_in_core) continue; // core painted next
            const size_t idx = static_cast<size_t>(y) * N + x;
            if (data[idx] < soft) data[idx] = static_cast<int8_t>(soft);
          }
        }
      }
      // Hard core: paint [cx+lo .. cx+hi] x [cy+lo .. cy+hi] = exactly the
      // voxel footprint (paint_n*res = voxel_lattice).
      for (int dy = lo; dy <= hi; ++dy) {
        const int y = cy + dy;
        if (y < 0 || y >= N) continue;
        for (int dx = lo; dx <= hi; ++dx) {
          const int x = cx + dx;
          if (x < 0 || x >= N) continue;
          data[static_cast<size_t>(y) * N + x] = 100;
        }
      }
    }
  }

  // Subs / pubs / timer.
  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher  grid_pub_;
  ros::Timer      timer_;

  // Params.
  double cloud_timeout_{0.5};
  double z_min_{0.05}, z_max_{2.0};
  double res_{0.05};
  double window_m_{10.0};
  double voxel_lattice_{0.2};
  double heartbeat_hz_{1.0};
  int    soft_rim_cells_{2};
  int    soft_rim_value_{60};
  int    paint_n_{4};
  int    N_{200};

  // Shared state.
  std::mutex mtx_;
  sensor_msgs::PointCloud2ConstPtr last_cloud_;
  ros::Time last_cloud_stamp_;
  ros::Time last_published_stamp_;
  double last_robot_x_{0.0}, last_robot_y_{0.0};
  bool   have_odom_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "rog_map_to_grid_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Resolve cloud topic AFTER remap so the wait-for-message guard hits the
  // correct topic. ros::names::resolve(name) honours <remap from="cloud_in" .../>.
  const std::string cloud_topic = ros::names::resolve("cloud_in");

  double startup_wait_s;
  pnh.param("startup_wait_s", startup_wait_s, 5.0);

  ROS_INFO("[rog_map_to_grid_node] waiting up to %.1fs for first message on '%s'...",
           startup_wait_s, cloud_topic.c_str());
  auto first = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
      cloud_topic, nh, ros::Duration(startup_wait_s));
  if (!first) {
    ROS_FATAL("[rog_map_to_grid_node] Stage 1 (rog_map_node) not publishing %s; "
              "refusing to start. Re-launch with enable_rog_map_observer:=true",
              cloud_topic.c_str());
    return 1;
  }
  ROS_INFO("[rog_map_to_grid_node] first cloud OK on '%s' (%u pts); entering spin.",
           cloud_topic.c_str(), first->width * first->height);

  RogMapToGridBridge bridge(nh, pnh);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
