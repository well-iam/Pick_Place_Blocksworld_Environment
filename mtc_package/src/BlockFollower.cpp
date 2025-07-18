#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>  // per ros_gz_interfaces::msg::Entity
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // Usa questa al posto del .h obsoleto
#include <thread>
#include <chrono>
#include <memory>

class BlockFollower {
public:
  BlockFollower(rclcpp::Node::SharedPtr node, const std::string& block_name)
  : node_(node), block_name_(block_name), running_(false),
    tf_buffer_(node->get_clock()), tf_listener_(tf_buffer_)
  {
   client_ = node_->create_client<ros_gz_interfaces::srv::SetEntityPose>("/world/blocksworld/set_pose");
  }

  ~BlockFollower() {
  //stop();  // will join if needed, or no‐op if already stopped

  running_ = false;
  if (follow_thread_.joinable())
    follow_thread_.join();
  }

  void start() {
    running_ = true;
    follow_thread_ = std::thread([this]() { this->run(); });
  }

  void stop() {

    //Ending pose management: ensure block has same world orientation once posed
    geometry_msgs::msg::TransformStamped tf;
    tf = tf_buffer_.lookupTransform("world", "flange", tf2::TimePointZero);
    const auto& trans = tf.transform.translation;
    const auto& rot   = tf.transform.rotation;
    geometry_msgs::msg::Pose pose;
    tf2::Quaternion q(rot.x, rot.y, rot.z, rot.w);
    tf2::Vector3 local_offset(0.07, 0.0, 0.0);

    // 3) Rotate the local offset into world frame
    tf2::Vector3 world_offset = tf2::quatRotate(q, local_offset);

    //Same position as before
    pose.position.x  = trans.x + world_offset.x();
    pose.position.y  = trans.y + world_offset.y();
    pose.position.z  = trans.z + world_offset.z();

    //Switch the orientation
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    auto req = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
    //Fai ros2 interface show ros_gz_interfaces/srv/SetEntityPose  per capirci qualcosa
    req->entity.name = block_name_;  // nome dell'entità da muovere
    req->entity.type = ros_gz_interfaces::msg::Entity::MODEL;  // o il tipo corretto (MODEL, LINK, ecc.)
    req->pose = pose;  
    client_->async_send_request(req);
    running_ = false;
    if (follow_thread_.joinable())
      follow_thread_.join();

  }

private:
  void run() {
    if (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(node_->get_logger(), "Service /set_pose not available");
      return;
    }
    try {
      while (running_ && rclcpp::ok()) {
        geometry_msgs::msg::TransformStamped tf;
        geometry_msgs::msg::TransformStamped tf3;
        try {
          tf = tf_buffer_.lookupTransform("world", "flange", tf2::TimePointZero);
          tf3 = tf_buffer_.lookupTransform("world", "wrist_2_link", tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
          RCLCPP_WARN(node_->get_logger(), "TF error: %s", ex.what());
          continue;
        }

        // 1) Extract the quaternion and translation
        const auto& trans = tf.transform.translation;
        const auto& rot   = tf.transform.rotation;

        //Add a offset along the x axis of tool0 to avoid compenetration
        // 2) Build a tf2::Quaternion and a local offset vector
        tf2::Quaternion q(rot.x, rot.y, rot.z, rot.w);
        tf2::Vector3 local_offset(0.07, 0.0, 0.0);

        // 3) Rotate the local offset into world frame
        tf2::Vector3 world_offset = tf2::quatRotate(q, local_offset);

        // 4) Fill your Pose
        geometry_msgs::msg::Pose pose;
        pose.position.x  = trans.x + world_offset.x();
        pose.position.y  = trans.y + world_offset.y();
        pose.position.z  = trans.z + world_offset.z();//+0.01;

        //pose.orientation = tf3.transform.rotation;;  // keep the same orientation

        auto req = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
        //Fai ros2 interface show ros_gz_interfaces/srv/SetEntityPose  per capirci qualcosa
        req->entity.name = block_name_;  // nome dell'entità da muovere
        req->entity.type = ros_gz_interfaces::msg::Entity::MODEL;  // o il tipo corretto (MODEL, LINK, ecc.)
        req->pose = pose;  


        client_->async_send_request(req);
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); //Can be at most the one of the pose_relay_plugin
        }
      }
      catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(),
                   "BlockFollower thread threw: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(node_->get_logger(),
                   "BlockFollower thread threw unknown exception");
    }
  }


  rclcpp::Node::SharedPtr node_;
  std::string block_name_;
  bool running_;
  std::thread follow_thread_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr client_;
};
