#include <cmath>
#include <sstream>
#include <string>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class CmdVelMarkerPublisher {
public:
    CmdVelMarkerPublisher()
        : private_nh_("~"),
          has_cmd_vel_(false),
          linear_scale_(1.0),
          arrow_shaft_diameter_(0.08),
          arrow_head_diameter_(0.16),
          z_offset_(0.25),
          text_z_offset_(0.6),
          text_size_(0.22),
          publish_rate_(15.0),
          stale_timeout_(0.5) {
        private_nh_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
        private_nh_.param("marker_topic", marker_topic_, std::string("/cmd_vel_marker"));
        private_nh_.param("frame_id", frame_id_, std::string("virtual_frame"));
        private_nh_.param("linear_scale", linear_scale_, linear_scale_);
        private_nh_.param("arrow_shaft_diameter", arrow_shaft_diameter_, arrow_shaft_diameter_);
        private_nh_.param("arrow_head_diameter", arrow_head_diameter_, arrow_head_diameter_);
        private_nh_.param("z_offset", z_offset_, z_offset_);
        private_nh_.param("text_z_offset", text_z_offset_, text_z_offset_);
        private_nh_.param("text_size", text_size_, text_size_);
        private_nh_.param("publish_rate", publish_rate_, publish_rate_);
        private_nh_.param("stale_timeout", stale_timeout_, stale_timeout_);

        marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, 10);
        cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic_, 10, &CmdVelMarkerPublisher::cmdVelCallback, this);

        const double clamped_publish_rate = std::max(1.0, publish_rate_);
        publish_timer_ = nh_.createTimer(
            ros::Duration(1.0 / clamped_publish_rate),
            &CmdVelMarkerPublisher::publishTimerCallback,
            this);
    }

private:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        latest_cmd_vel_ = *msg;
        latest_cmd_vel_stamp_ = ros::Time::now();
        has_cmd_vel_ = true;
    }

    void publishTimerCallback(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();
        const bool is_stale = has_cmd_vel_ && stale_timeout_ > 0.0 &&
            (now - latest_cmd_vel_stamp_).toSec() > stale_timeout_;

        publishArrowMarker(now, is_stale);
        publishTextMarker(now, is_stale);
    }

    void publishArrowMarker(const ros::Time& stamp, bool is_stale) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = stamp;
        marker.ns = "cmd_vel";
        marker.id = 0;
        marker.pose.orientation.w = 1.0;

        if (!has_cmd_vel_) {
            marker.action = visualization_msgs::Marker::DELETE;
            marker_pub_.publish(marker);
            return;
        }

        const double vx = latest_cmd_vel_.linear.x;
        const double vy = latest_cmd_vel_.linear.y;
        const double linear_speed = std::hypot(vx, vy);

        if (linear_speed < 1e-4) {
            marker.action = visualization_msgs::Marker::DELETE;
            marker_pub_.publish(marker);
            return;
        }

        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = arrow_shaft_diameter_;
        marker.scale.y = arrow_head_diameter_;
        marker.scale.z = arrow_head_diameter_ * 1.2;
        marker.color.a = 0.9;
        marker.color.r = is_stale ? 1.0f : 0.1f;
        marker.color.g = is_stale ? 0.45f : 0.9f;
        marker.color.b = 0.15f;

        geometry_msgs::Point start;
        start.x = 0.0;
        start.y = 0.0;
        start.z = z_offset_;

        geometry_msgs::Point end;
        end.x = vx * linear_scale_;
        end.y = vy * linear_scale_;
        end.z = z_offset_;

        marker.points.push_back(start);
        marker.points.push_back(end);
        marker_pub_.publish(marker);
    }

    void publishTextMarker(const ros::Time& stamp, bool is_stale) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = stamp;
        marker.ns = "cmd_vel";
        marker.id = 1;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.pose.position.z = text_z_offset_;
        marker.scale.z = text_size_;
        marker.color.a = 1.0;
        marker.color.r = is_stale ? 1.0f : 0.95f;
        marker.color.g = is_stale ? 0.6f : 0.95f;
        marker.color.b = is_stale ? 0.1f : 0.95f;

        std::ostringstream stream;
        stream.setf(std::ios::fixed);
        stream.precision(2);

        if (!has_cmd_vel_) {
            stream << "cmd_vel: waiting";
        } else {
            stream << "vx=" << latest_cmd_vel_.linear.x
                   << " vy=" << latest_cmd_vel_.linear.y
                   << " wz=" << latest_cmd_vel_.angular.z;
            if (is_stale) {
                stream << " stale";
            }
        }

        marker.text = stream.str();
        marker_pub_.publish(marker);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher marker_pub_;
    ros::Timer publish_timer_;

    geometry_msgs::Twist latest_cmd_vel_;
    ros::Time latest_cmd_vel_stamp_;
    bool has_cmd_vel_;

    std::string cmd_vel_topic_;
    std::string marker_topic_;
    std::string frame_id_;
    double linear_scale_;
    double arrow_shaft_diameter_;
    double arrow_head_diameter_;
    double z_offset_;
    double text_z_offset_;
    double text_size_;
    double publish_rate_;
    double stale_timeout_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cmd_vel_marker");
    CmdVelMarkerPublisher publisher;
    ros::spin();
    return 0;
}