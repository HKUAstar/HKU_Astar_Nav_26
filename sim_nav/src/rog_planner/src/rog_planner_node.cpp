// Phase 3: rog_planner — main node.
//
// Owns:
//   - SentryMap (subclass of rog_map::ROGMap) with ESDF enabled.
//   - Front-end A* (rog_planner::AStar3D)
//   - Back-end B-spline + L-BFGS smoother
//   - 100 Hz tracker that emits /cmd_vel (virtual_frame) + /dstar_status (Bool)
//
// Topics IN:
//   /clicked_point        (PointStamped, map frame)   — goal from BehaviorTree
//   /aligned_points       (PointCloud2)               — to ROGMap (via internal callbacks)
//   /odom                 (Odometry)                  — to ROGMap (via internal callbacks)
//
// Topics OUT:
//   /cmd_vel              (Twist, virtual_frame)      — 100 Hz, MCU bridge
//   /dstar_status         (Bool)                      — 100 Hz, BehaviorTree
//   ~rog_planner/path     (Path, map frame)           — front-end, viz
//   ~rog_planner/trajectory (Path, map frame)         — sampled smoothed traj

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Core>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>

#include <rog_planner/sentry_map.h>
#include <rog_planner/astar.h>
#include <rog_planner/bspline_smoother.h>
#include <rog_planner/static_map_2d.h>

namespace rog_planner {

class PlannerNode {
public:
  PlannerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh),
        tf_buf_(),
        tf_listener_(tf_buf_) {
    // ----------------- Params -----------------
    pnh_.param("map_frame",      map_frame_,      std::string("map"));
    pnh_.param("robot_frame",    robot_frame_,    std::string("virtual_frame"));
    pnh_.param("chassis_height", chassis_h_,      0.3);
    pnh_.param("safe_dist",      safe_dist_,      0.4);
    pnh_.param("v_max",          v_max_,          2.5);
    pnh_.param("a_max",          a_max_,          4.0);
    pnh_.param("arrival_radius", arrival_radius_, 0.25);
    pnh_.param("replan_rate",    replan_rate_,    5.0);
    pnh_.param("tracker_rate",   tracker_rate_,   100.0);
    pnh_.param("w_smooth",       w_smooth_,       10.0);
    pnh_.param("w_obs",          w_obs_,          10000.0);
    pnh_.param("w_dyn",          w_dyn_,          100.0);
    pnh_.param("blend_time",     blend_time_,     0.1);
    pnh_.param("hard_safe_dist", hard_safe_dist_, 0.30);
    pnh_.param("collision_d_min",collision_d_min_, hard_safe_dist_);
    // Live tracker safety threshold. Only used by commandSegmentIsSafe
    // to reject the *next* short horizon. Set lower than hard_safe_dist
    // (which is the planner's conservative margin) so a path that
    // nominally hugs the planner margin doesn't get the trajectory
    // rejected on every tick — the live check only triggers on a
    // genuinely-newer-and-closer obstacle.
    pnh_.param("tracker_safe_dist", tracker_safe_dist_, 0.18);
    pnh_.param("track_kp",       track_kp_,       0.45);
    pnh_.param("track_lookahead_time", track_lookahead_time_, 0.0);
    pnh_.param("track_search_dt", track_search_dt_, 0.02);
    pnh_.param("astar_resolution", astar_resolution_, 0.1);
    pnh_.param("static_map_enable", static_map_enabled_, true);
    pnh_.param("static_map_topic", static_map_topic_, std::string("/map"));
    pnh_.param("static_map_occ_threshold", static_map_occ_threshold_, 65);
    pnh_.param("static_map_inflation_radius", static_map_inflation_radius_, 0.05);
    pnh_.param("static_map_safe_dist", static_map_safe_dist_, 0.25);
    pnh_.param("static_map_unknown_as_occupied", static_map_unknown_as_occupied_, false);
    pnh_.param("static_map_outside_as_occupied", static_map_outside_as_occupied_, true);
    pnh_.param("w_static_obs", w_static_obs_, 50000.0);

    // ----------------- Map (ESDF-enabled) -----------------
    // ROGMap reads its config under nh's private namespace ~rog_map/...
    map_.reset(new SentryMap(pnh_));

    if (static_map_enabled_) {
      StaticMapParams smp;
      smp.occ_threshold = static_map_occ_threshold_;
      smp.inflation_radius = static_map_inflation_radius_;
      smp.unknown_as_occupied = static_map_unknown_as_occupied_;
      smp.outside_as_occupied = static_map_outside_as_occupied_;
      static_map_.reset(new StaticMap2D(smp));
      static_map_sub_ = nh_.subscribe(static_map_topic_, 1,
                                      &PlannerNode::staticMapCallback, this);
    }

    // ----------------- Planner / smoother -----------------
    AStarParams ap;
    ap.chassis_height = chassis_h_;
    ap.resolution     = astar_resolution_;
    ap.safe_dist      = safe_dist_;
    ap.hard_safe_dist = hard_safe_dist_;
    ap.static_safe_dist = static_map_safe_dist_;
    ap.static_hard_safe_dist = static_map_safe_dist_;
    astar_.reset(new AStar3D(map_.get(), ap, static_map_.get()));

    BSplineParams bp;
    bp.v_max          = v_max_;
    bp.a_max          = a_max_;
    bp.safe_dist      = safe_dist_;
    bp.static_safe_dist = static_map_safe_dist_;
    bp.w_smooth       = w_smooth_;
    bp.w_obs          = w_obs_;
    bp.w_static_obs   = w_static_obs_;
    bp.w_dyn          = w_dyn_;
    bp.chassis_height = chassis_h_;
    // Allow tuning smoother work budget from launch. Defaults bumped from
    // (50, 0.20 s) so red-zone (low-ESDF) optimizations have enough budget
    // to converge instead of returning empty/best-iterate paths.
    pnh_.param("smoother_max_iter",  bp.max_iter,    120);
    pnh_.param("smoother_max_time_s", bp.max_time_s, 0.30);
    smoother_.reset(new BSplineSmoother(map_.get(), bp, static_map_.get()));

    // ----------------- Pubs / subs -----------------
    cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    status_pub_    = nh_.advertise<std_msgs::Bool>("/dstar_status", 1);
    path_pub_      = pnh_.advertise<nav_msgs::Path>("path", 1);
    traj_pub_      = pnh_.advertise<nav_msgs::Path>("trajectory", 1);
    goal_sub_      = nh_.subscribe("/clicked_point", 1,
                                   &PlannerNode::goalCallback, this);

    // ----------------- Timers -----------------
    planner_timer_ = nh_.createTimer(ros::Duration(1.0 / replan_rate_),
                                     &PlannerNode::onPlannerTick, this);
    tracker_timer_ = nh_.createTimer(ros::Duration(1.0 / tracker_rate_),
                                     &PlannerNode::onTrackerTick, this);

    ROS_INFO("[rog_planner] initialized (planner@%.1fHz, tracker@%.1fHz, static_map=%s)",
         replan_rate_, tracker_rate_, static_map_enabled_ ? "on" : "off");
  }

private:
  // ----------------- State -----------------
  ros::NodeHandle nh_, pnh_;
  tf2_ros::Buffer            tf_buf_;
  tf2_ros::TransformListener tf_listener_;

  std::shared_ptr<SentryMap>       map_;
  std::shared_ptr<StaticMap2D>      static_map_;
  std::shared_ptr<AStar3D>         astar_;
  std::shared_ptr<BSplineSmoother> smoother_;

  ros::Publisher  cmd_vel_pub_, status_pub_, path_pub_, traj_pub_;
  ros::Subscriber goal_sub_, static_map_sub_;
  ros::Timer      planner_timer_, tracker_timer_;

  // Params
  std::string map_frame_, robot_frame_;
  double chassis_h_, safe_dist_, hard_safe_dist_, v_max_, a_max_, arrival_radius_;
  double replan_rate_, tracker_rate_;
  double w_smooth_, w_obs_, w_dyn_;
  double blend_time_;
  double collision_d_min_;
  double tracker_safe_dist_;
  double track_kp_, track_lookahead_time_, track_search_dt_;
  double astar_resolution_;
  bool static_map_enabled_;
  std::string static_map_topic_;
  int static_map_occ_threshold_;
  double static_map_inflation_radius_, static_map_safe_dist_, w_static_obs_;
  bool static_map_unknown_as_occupied_, static_map_outside_as_occupied_;

  // Goal state
  std::mutex      goal_mtx_;
  Eigen::Vector3d goal_;
  bool            has_goal_ = false;
  bool            goal_dirty_ = false;

  // Active trajectory (mutex-guarded; tracker reads, planner writes)
  std::mutex                 traj_mtx_;
  BSplineTrajectory          traj_active_, traj_pending_;
  ros::Time                  t_active_start_;
  ros::Time                  t_blend_start_;
  bool                       has_active_ = false;
  bool                       has_pending_ = false;

  // Tracker debounce: number of consecutive unsafe-segment ticks. Trajectory
  // is cleared only when this exceeds tracker_unsafe_ticks_to_clear_, so a
  // single-frame map flicker (ghost briefly appearing on the path) does not
  // drop the active plan.
  int                        unsafe_consecutive_ = 0;
  static constexpr int       tracker_unsafe_ticks_to_clear_ = 10;  // 100 ms @ 100 Hz

  // ----------------- Goal handling -----------------
  void goalCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lck(goal_mtx_);
    Eigen::Vector3d g(msg->point.x, msg->point.y, msg->point.z);
    if (!has_goal_ || (g - goal_).norm() > 0.05) {
      goal_       = g;
      has_goal_   = true;
      goal_dirty_ = true;
      ROS_INFO("[rog_planner] new goal (%.2f, %.2f, %.2f)",
               g.x(), g.y(), g.z());
    }
  }

  void staticMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    if (static_map_) static_map_->update(*msg);
  }

  // ----------------- Planner timer (5 Hz) -----------------
  void onPlannerTick(const ros::TimerEvent&) {
    Eigen::Vector3d goal;
    bool dirty;
    {
      std::lock_guard<std::mutex> lck(goal_mtx_);
      if (!has_goal_) return;
      goal  = goal_;
      dirty = goal_dirty_;
      // NOTE: goal_dirty_ is cleared only after a successful install
      // (end of this function), so a failed A* / smoother retry on the
      // next tick.
    }

    // Robot pose from ROGMap's internal RobotState.
    auto rs = map_->getRobotState();
    if (!rs.rcv ||
        (ros::Time::now().toSec() - rs.rcv_time) > 0.5) {
      // Stale odom — emergency stop is handled in tracker via status=false.
      ROS_WARN_THROTTLE(2.0, "[rog_planner] stale RobotState; skip plan");
      return;
    }
    Eigen::Vector3d start = rs.p;

    if (static_map_enabled_ && (!static_map_ || !static_map_->ready())) {
      ROS_WARN_THROTTLE(2.0, "[rog_planner] waiting for static /map before planning");
      return;
    }

    // Replan trigger: dirty (new goal), no active, expired, or projected collision.
    bool need_replan = dirty || !has_active_;
    bool active_invalid = false;
    {
      std::lock_guard<std::mutex> tlck(traj_mtx_);
      if (has_active_) {
        double t_now = (ros::Time::now() - t_active_start_).toSec();
        if (t_now > traj_active_.tEnd()) {
          need_replan = true;
          active_invalid = true;
        }
        // Projected collision check on remaining trajectory. Hold the
        // ESDF writer mutex so the 1 kHz updateESDF3D() does not race
        // our reads.
        std::lock_guard<std::mutex> elck(map_->getEsdfMutex());
        for (double t = t_now; t < traj_active_.tEnd(); t += 0.1) {
          Eigen::Vector3d p = traj_active_.pos(t);
          double d = map_->dist(p);
          if (d < collision_d_min_) {
            need_replan = true;
            active_invalid = true;
            break;
          }
          if (static_map_ &&
              static_map_->clearance(Eigen::Vector2d(p.x(), p.y()),
                                     static_map_safe_dist_, nullptr) < static_map_safe_dist_) {
            need_replan = true;
            active_invalid = true;
            break;
          }
        }
      }
    }
    if (!need_replan) return;

    // ---- Front-end A* ----
    AStar3D::Path waypoints;
    if (!astar_->plan(start, goal, waypoints)) {
      ROS_WARN_THROTTLE(2.0, "[rog_planner] A* failed");
      if (active_invalid) clearActiveTrajectory("A* failed after active trajectory became invalid");
      return;
    }
    publishPath(path_pub_, waypoints);

    // ---- Back-end smooth ----
    BSplineTrajectory traj;
    auto t0 = std::chrono::steady_clock::now();
    if (!smoother_->smooth(waypoints, traj)) {
      ROS_WARN_THROTTLE(2.0, "[rog_planner] smoother rejected trajectory; not installing fallback");
      if (active_invalid) clearActiveTrajectory("smoother rejected after active trajectory became invalid");
      return;
    }
    if (!trajectoryIsSafe(traj)) {
      ROS_WARN_THROTTLE(2.0, "[rog_planner] planned trajectory is unsafe; not installing");
      if (active_invalid) clearActiveTrajectory("validation failed after active trajectory became invalid");
      return;
    }
    auto dt_smooth = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    ROS_INFO_THROTTLE(1.0, "[rog_planner] replan ok: A*=%d expns, smooth=%.1fms, cost=%.2f",
                      astar_->expansions(), 1000 * dt_smooth, smoother_->lastCost());

    // ---- Install as pending trajectory; tracker performs blending ----
    {
      std::lock_guard<std::mutex> lck(traj_mtx_);
      traj_active_     = traj;
      has_active_      = true;
      has_pending_     = false;
      t_active_start_  = ros::Time::now();
    }

    publishTrajectorySamples(traj);

    // Successful install — clear the dirty bit so we don't re-plan
    // immediately on the next tick for the same goal.
    {
      std::lock_guard<std::mutex> lck(goal_mtx_);
      goal_dirty_ = false;
    }
  }

  // ----------------- Tracker timer (100 Hz) -----------------
  void onTrackerTick(const ros::TimerEvent&) {
    geometry_msgs::Twist cmd;
    std_msgs::Bool       arrived_msg;
    arrived_msg.data = false;

    // Robot state (for cross-track + arrival check + safety)
    auto rs = map_->getRobotState();
    bool stale = !rs.rcv ||
                 (ros::Time::now().toSec() - rs.rcv_time) > 0.5;
    if (stale) {
      cmd_vel_pub_.publish(cmd);
      arrived_msg.data = false;
      status_pub_.publish(arrived_msg);
      return;
    }

    bool have_traj = false;
    Eigen::Vector3d pos_des, vel_des;
    {
      std::lock_guard<std::mutex> lck(traj_mtx_);
      if (has_active_) {
        have_traj = true;
        double t_nom = (ros::Time::now() - t_active_start_).toSec();
        double t_end = traj_active_.tEnd();
        if (t_nom > t_end) t_nom = t_end - 1e-3;

        // Strict path following: track the nearest point on the active
        // trajectory, not a time-indexed point that may be several steps
        // ahead when the robot lags. This removes pure-pursuit corner cuts.
        double best_t = 0.0;
        double best_d2 = std::numeric_limits<double>::infinity();
        double dt = std::max(0.005, track_search_dt_);
        for (double tt = 0.0; tt < t_end; tt += dt) {
          Eigen::Vector3d p = traj_active_.pos(tt);
          double d2 = (p.head<2>() - rs.p.head<2>()).squaredNorm();
          if (d2 < best_d2) { best_d2 = d2; best_t = tt; }
        }
        double end_d2 = (traj_active_.pos(t_end).head<2>() - rs.p.head<2>()).squaredNorm();
        if (end_d2 < best_d2) best_t = t_end;

        double t_ref = std::min(t_end - 1e-3,
                                best_t + std::max(0.0, track_lookahead_time_));
        pos_des = traj_active_.pos(t_ref);
        vel_des = traj_active_.vel(t_ref);
      }
    }

    // Arrival check vs current goal (xy only — /clicked_point z is
    // typically 0, while rs.p.z reflects hdl_localization odom and
    // sits ~chassis_height above; a 3-D distance check would never
    // trip arrival_radius=0.25 m).
    {
      std::lock_guard<std::mutex> lck(goal_mtx_);
      if (has_goal_ &&
          (rs.p.head<2>() - goal_.head<2>()).norm() < arrival_radius_) {
        arrived_msg.data = true;
        cmd_vel_pub_.publish(cmd);   // zero cmd
        status_pub_.publish(arrived_msg);
        return;
      }
    }

    if (!have_traj) {
      cmd_vel_pub_.publish(cmd);
      status_pub_.publish(arrived_msg);
      return;
    }

    // Cross-track correction (small)
    Eigen::Vector3d v_cmd = vel_des + track_kp_ * (pos_des - rs.p);

    // Cap to v_max.
    double v_norm = v_cmd.head<2>().norm();
    if (v_norm > v_max_) v_cmd.head<2>() *= (v_max_ / v_norm);

    if (!commandSegmentIsSafe(rs.p, v_cmd)) {
      // Debounce: a single-frame map glitch (ghost flicker on the path)
      // should not nuke the active trajectory. Only clear after several
      // consecutive unsafe ticks.
      if (++unsafe_consecutive_ >= tracker_unsafe_ticks_to_clear_) {
        clearActiveTrajectory("tracker command segment unsafe");
        unsafe_consecutive_ = 0;
      }
      cmd_vel_pub_.publish(cmd);
      status_pub_.publish(arrived_msg);
      return;
    }
    unsafe_consecutive_ = 0;

    // Transform into virtual_frame (rotation-only TF).
    geometry_msgs::TransformStamped T;
    try {
      T = tf_buf_.lookupTransform(robot_frame_, map_frame_, ros::Time(0));
    } catch (tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(2.0, "[rog_planner] TF map->%s: %s",
                        robot_frame_.c_str(), ex.what());
      cmd_vel_pub_.publish(cmd);
      status_pub_.publish(arrived_msg);
      return;
    }
    geometry_msgs::Vector3 v_in, v_out;
    v_in.x = v_cmd.x(); v_in.y = v_cmd.y(); v_in.z = 0.0;
    geometry_msgs::TransformStamped Trot = T;
    Trot.transform.translation.x = 0;
    Trot.transform.translation.y = 0;
    Trot.transform.translation.z = 0;
    geometry_msgs::Vector3Stamped vs_in, vs_out;
    vs_in.vector = v_in;
    tf2::doTransform(vs_in, vs_out, Trot);
    v_out = vs_out.vector;

    // ---- Z-tilt safety scaling (ported from dstarlite.cpp:523-525) ----
    double xy_len = std::hypot(v_out.x, v_out.y);
    if (xy_len < 1e-6) {
      cmd.linear.x = 0; cmd.linear.y = 0;
    } else {
      double z_angle = std::atan2(v_out.z, xy_len);
      double k       = z_angle / (15.0 / 180.0 * M_PI);
      double sx      = std::min(1.0 - k, 1.7);
      double sy      = std::max(1.0 - k, 0.7);
      cmd.linear.x = v_out.x * sx;
      cmd.linear.y = v_out.y * sy;
    }
    cmd.linear.z = 0; cmd.angular.x = cmd.angular.y = cmd.angular.z = 0;

    cmd_vel_pub_.publish(cmd);
    status_pub_.publish(arrived_msg);
  }

  // ----------------- Visualization helpers -----------------
  void publishPath(ros::Publisher& pub,
                   const std::vector<Eigen::Vector3d>& path) {
    nav_msgs::Path msg;
    msg.header.stamp    = ros::Time::now();
    msg.header.frame_id = map_frame_;
    msg.poses.reserve(path.size());
    for (const auto& p : path) {
      geometry_msgs::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = p.x();
      ps.pose.position.y = p.y();
      ps.pose.position.z = p.z();
      ps.pose.orientation.w = 1.0;
      msg.poses.push_back(ps);
    }
    pub.publish(msg);
  }

  void publishTrajectorySamples(const BSplineTrajectory& traj) {
    nav_msgs::Path msg;
    msg.header.stamp    = ros::Time::now();
    msg.header.frame_id = map_frame_;
    double dt = 0.05;
    for (double t = 0; t < traj.tEnd(); t += dt) {
      Eigen::Vector3d p = traj.pos(t);
      geometry_msgs::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = p.x();
      ps.pose.position.y = p.y();
      ps.pose.position.z = p.z();
      ps.pose.orientation.w = 1.0;
      msg.poses.push_back(ps);
    }
    traj_pub_.publish(msg);
  }

  void clearActiveTrajectory(const char* reason) {
    std::lock_guard<std::mutex> lck(traj_mtx_);
    has_active_ = false;
    has_pending_ = false;
    ROS_WARN("[rog_planner] cleared active trajectory: %s", reason);
  }

  bool trajectoryIsSafe(const BSplineTrajectory& traj) {
    std::lock_guard<std::mutex> elck(map_->getEsdfMutex());
    if (traj.tEnd() <= 0.0) return false;

    auto pointSafe = [&](const Eigen::Vector3d& p) {
      if (map_->dist(p) < collision_d_min_) return false;
      if (static_map_ &&
          static_map_->clearance(Eigen::Vector2d(p.x(), p.y()),
                                 static_map_safe_dist_, nullptr) < static_map_safe_dist_) return false;
      return true;
    };
    auto segmentSafe = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
      double len = (b - a).head<2>().norm();
      int samples = std::max(1, static_cast<int>(std::ceil(len / 0.025)));
      for (int i = 0; i <= samples; ++i) {
        double u = static_cast<double>(i) / samples;
        Eigen::Vector3d p = (1.0 - u) * a + u * b;
        if (!pointSafe(p)) return false;
      }
      return true;
    };

    const double dt = 0.02;
    Eigen::Vector3d prev = traj.pos(0.0);
    if (!pointSafe(prev)) return false;
    for (double t = dt; t < traj.tEnd(); t += dt) {
      Eigen::Vector3d cur = traj.pos(t);
      if (!segmentSafe(prev, cur)) return false;
      prev = cur;
    }
    if (!segmentSafe(prev, traj.pos(traj.tEnd()))) return false;
    return true;
  }

  bool commandSegmentIsSafe(const Eigen::Vector3d& robot_pos,
                            const Eigen::Vector3d& v_cmd) {
    double speed = v_cmd.head<2>().norm();
    if (speed < 1e-6) return true;

    Eigen::Vector3d dir(v_cmd.x(), v_cmd.y(), 0.0);
    dir.normalize();
    double horizon = std::min(0.45, std::max(0.15, speed * 0.25));
    int samples = std::max(1, static_cast<int>(std::ceil(horizon / 0.025)));

    std::lock_guard<std::mutex> elck(map_->getEsdfMutex());
    for (int i = 1; i <= samples; ++i) {
      double s = horizon * static_cast<double>(i) / samples;
      Eigen::Vector3d p = robot_pos + dir * s;
      p.z() = chassis_h_;
      // Use the relaxed live-tracker threshold (smaller than the planner's
      // hard_safe_dist) so paths that nominally hug the planner margin do
      // NOT get rejected on every tick. Real new obstacles still trip this.
      if (map_->dist(p) < tracker_safe_dist_) return false;
      if (static_map_ &&
          static_map_->clearance(Eigen::Vector2d(p.x(), p.y()),
                                 static_map_safe_dist_, nullptr) < static_map_safe_dist_) {
        return false;
      }
    }
    return true;
  }
};

}  // namespace rog_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "rog_planner_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  rog_planner::PlannerNode node(nh, pnh);
  // 4 worker threads so the 100 Hz tracker is not starved by:
  //   (a) the 5 Hz planner running L-BFGS for ~200 ms,
  //   (b) ROGMap's /aligned_points + /odom callbacks (~tens of ms each),
  //   (c) ROGMap's 1 kHz update timer.
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
