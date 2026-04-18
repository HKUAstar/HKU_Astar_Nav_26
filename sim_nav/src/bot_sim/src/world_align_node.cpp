#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <XmlRpcValue.h>
#include <cmath>
#include <string>
#include <vector>

struct RefPoint2D {
  double x;
  double y;
};

class WorldAligner {
public:
  bool enabled = false;
  double theta = 0.0;
  double shift_x = 0.0;
  double shift_y = 0.0;

  void setByThetaShift(double t, double sx, double sy) {
    theta = t;
    shift_x = sx;
    shift_y = sy;
    enabled = true;
  }

  bool solveFromRefPoints(const std::vector<RefPoint2D>& map_pts,
                          const std::vector<RefPoint2D>& world_pts) {
    if (map_pts.size() < 2 || map_pts.size() != world_pts.size()) {
      return false;
    }

    const int n = static_cast<int>(map_pts.size());
    double mx_cx = 0.0, mx_cy = 0.0;
    double wx_cx = 0.0, wx_cy = 0.0;
    for (int i = 0; i < n; ++i) {
      mx_cx += map_pts[i].x;
      mx_cy += map_pts[i].y;
      wx_cx += world_pts[i].x;
      wx_cy += world_pts[i].y;
    }
    mx_cx /= n; mx_cy /= n;
    wx_cx /= n; wx_cy /= n;

    double a = 0.0;
    double b = 0.0;
    for (int i = 0; i < n; ++i) {
      const double mx = map_pts[i].x - mx_cx;
      const double my = map_pts[i].y - mx_cy;
      const double wx = world_pts[i].x - wx_cx;
      const double wy = world_pts[i].y - wx_cy;
      a += mx * wx + my * wy;
      b += mx * wy - my * wx;
    }

    theta = std::atan2(b, a);
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    shift_x = wx_cx - (c * mx_cx - s * mx_cy);
    shift_y = wx_cy - (s * mx_cx + c * mx_cy);
    enabled = true;
    return true;
  }

  std::pair<double, double> apply(double map_x, double map_y) const {
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    return {
      c * map_x - s * map_y + shift_x,
      s * map_x + c * map_y + shift_y
    };
  }
};

static bool parsePointListParam(ros::NodeHandle& nh,
                                const std::string& full_param_name,
                                std::vector<RefPoint2D>& out_points) {
  XmlRpc::XmlRpcValue raw;
  if (!nh.getParam(full_param_name, raw)) {
    return false;
  }
  if (raw.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("Param %s must be an array", full_param_name.c_str());
    return false;
  }

  out_points.clear();
  for (int i = 0; i < raw.size(); ++i) {
    if (raw[i].getType() != XmlRpc::XmlRpcValue::TypeArray || raw[i].size() != 2) {
      ROS_ERROR("Param %s[%d] must be [x, y]", full_param_name.c_str(), i);
      return false;
    }

    RefPoint2D p;
    if (raw[i][0].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      p.x = static_cast<int>(raw[i][0]);
    } else {
      p.x = static_cast<double>(raw[i][0]);
    }
    if (raw[i][1].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      p.y = static_cast<int>(raw[i][1]);
    } else {
      p.y = static_cast<double>(raw[i][1]);
    }

    out_points.push_back(p);
  }
  return true;
}

class WorldAlignNode {
public:
  WorldAlignNode() : pnh_("~") {
    pnh_.param<std::string>("input_topic", input_topic_, std::string("/sentinel_nav_position"));
    pnh_.param<std::string>("output_topic", output_topic_, std::string("/sentinel_world_position"));
    pnh_.param<std::string>("target_world_frame", target_world_frame_, std::string("world"));
    pnh_.param<std::string>("source_map_frame", source_map_frame_, std::string("map"));
    pnh_.param<bool>("publish_world_to_map_tf", publish_world_to_map_tf_, false);

    bool use_ref_points = false;
    pnh_.param<bool>("use_ref_points", use_ref_points, false);

    if (use_ref_points) {
      std::vector<RefPoint2D> map_ref_points;
      std::vector<RefPoint2D> world_ref_points;
      const bool got_map_ref = parsePointListParam(pnh_, "map_ref_points", map_ref_points);
      const bool got_world_ref = parsePointListParam(pnh_, "world_ref_points", world_ref_points);
      if (!got_map_ref || !got_world_ref || !aligner_.solveFromRefPoints(map_ref_points, world_ref_points)) {
        ROS_FATAL("world_align_node: invalid ref points. Need >=2 pairs and same size.");
        ros::shutdown();
        return;
      }
      ROS_INFO("world_align_node: solved by ref points, theta=%.6f shift=(%.6f, %.6f), count=%zu",
               aligner_.theta, aligner_.shift_x, aligner_.shift_y, map_ref_points.size());
    } else {
      double theta = 0.0;
      double shift_x = 0.0;
      double shift_y = 0.0;
      pnh_.param<double>("theta", theta, 0.0);
      pnh_.param<double>("shift_x", shift_x, 0.0);
      pnh_.param<double>("shift_y", shift_y, 0.0);
      aligner_.setByThetaShift(theta, shift_x, shift_y);
      ROS_INFO("world_align_node: using theta/shift, theta=%.6f shift=(%.6f, %.6f)",
               aligner_.theta, aligner_.shift_x, aligner_.shift_y);
    }

    sub_nav_pos_ = nh_.subscribe(input_topic_, 50, &WorldAlignNode::onNavPos, this);
    pub_world_pos_ = nh_.advertise<geometry_msgs::PointStamped>(output_topic_, 50);
    ROS_INFO("world_align_node ready: %s -> %s", input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void onNavPos(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if (!aligner_.enabled) {
      return;
    }

    const auto xy_world = aligner_.apply(msg->point.x, msg->point.y);

    geometry_msgs::PointStamped world_pos;
    world_pos.header.stamp = msg->header.stamp;
    world_pos.header.frame_id = target_world_frame_;
    world_pos.point.x = xy_world.first;
    world_pos.point.y = xy_world.second;
    world_pos.point.z = msg->point.z;
    pub_world_pos_.publish(world_pos);

    if (publish_world_to_map_tf_) {
      geometry_msgs::TransformStamped tf_world_map;
      tf_world_map.header.stamp = msg->header.stamp;
      tf_world_map.header.frame_id = target_world_frame_;
      tf_world_map.child_frame_id = source_map_frame_;
      tf_world_map.transform.translation.x = aligner_.shift_x;
      tf_world_map.transform.translation.y = aligner_.shift_y;
      tf_world_map.transform.translation.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, aligner_.theta);
      tf_world_map.transform.rotation.x = q.x();
      tf_world_map.transform.rotation.y = q.y();
      tf_world_map.transform.rotation.z = q.z();
      tf_world_map.transform.rotation.w = q.w();
      tf_broadcaster_.sendTransform(tf_world_map);
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_nav_pos_;
  ros::Publisher pub_world_pos_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  WorldAligner aligner_;
  std::string input_topic_;
  std::string output_topic_;
  std::string target_world_frame_;
  std::string source_map_frame_;
  bool publish_world_to_map_tf_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "world_align_node");
  WorldAlignNode node;
  ros::spin();
  return 0;
}
