// gz_pose_tf.cc — Gazebo (Fortress) model poses -> ROS TF, by model name.
//
// RViz can only draw what is on ROS topics. The world's static fixtures come from
// the SDF (scripts/scene_markers.py), but objects that move — the jerry cans the
// robot carries — need their live pose. gz-sim publishes those on
// /world/<world>/dynamic_pose/info, but ros_gz_bridge drops the entity names from
// that message (it only takes child_frame_id from header metadata, which the
// scene broadcaster doesn't set), and the gz-transport Python bindings on this
// system are Harmonic's, which can't talk to Fortress. So this node subscribes to
// Fortress's ign-transport directly and broadcasts one TF frame per model:
//
//   <parent_frame> -> <model name>     for every model whose name starts with
//                                      <model_prefix> (all models if empty)
//
// Parameters: world_name, model_prefix, parent_frame (default gz_world), rate (Hz).
// A static <robot world frame> -> gz_world transform ties it into the robot's tree.

#include <map>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

class GzPoseTf : public rclcpp::Node
{
public:
  GzPoseTf()
  : Node("gz_pose_tf")
  {
    const auto world = declare_parameter<std::string>("world_name", "moz1_jerrycan");
    prefix_ = declare_parameter<std::string>("model_prefix", "");
    parent_ = declare_parameter<std::string>("parent_frame", "gz_world");
    const double rate = declare_parameter<double>("rate", 20.0);

    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    const std::string topic = "/world/" + world + "/dynamic_pose/info";
    if (!gz_.Subscribe(topic, &GzPoseTf::OnPoses, this)) {
      RCLCPP_ERROR(get_logger(), "could not subscribe to gz topic %s", topic.c_str());
    }
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate),
                               [this] { Publish(); });
    RCLCPP_INFO(get_logger(), "%s -> TF '%s' -> '%s*'", topic.c_str(), parent_.c_str(),
                prefix_.c_str());
  }

private:
  void OnPoses(const ignition::msgs::Pose_V & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & p : msg.pose()) {
      if (p.name().rfind(prefix_, 0) == 0) {
        latest_[p.name()] = p;
      }
    }
  }

  void Publish()
  {
    std::vector<geometry_msgs::msg::TransformStamped> out;
    const auto stamp = now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & [name, p] : latest_) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = parent_;
        t.child_frame_id = name;
        t.transform.translation.x = p.position().x();
        t.transform.translation.y = p.position().y();
        t.transform.translation.z = p.position().z();
        t.transform.rotation.x = p.orientation().x();
        t.transform.rotation.y = p.orientation().y();
        t.transform.rotation.z = p.orientation().z();
        t.transform.rotation.w = p.orientation().w();
        out.push_back(t);
      }
    }
    if (!out.empty()) {
      tf_->sendTransform(out);
    }
  }

  ignition::transport::Node gz_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  std::map<std::string, ignition::msgs::Pose> latest_;
  std::string prefix_;
  std::string parent_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzPoseTf>());
  rclcpp::shutdown();
  return 0;
}
