#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <ignition/plugin/Register.hh> 
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ignition/msgs/pose_v.pb.h>
#include "my_sim_plugins/msg/block_pose_array.hpp" //Self-Made msg Inclusion
#include <memory>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>

namespace my_sim_plugins
{
  class PoseRelayPlugin : public ignition::gazebo::System,
                          public ignition::gazebo::ISystemConfigure,
                          public ignition::gazebo::ISystemPostUpdate
  {
  public:
    PoseRelayPlugin()
    {
      std::cout << "PoseRelayPlugin: Constructor" << std::endl;
    }
    
    void Configure(const ignition::gazebo::Entity &,
                   const std::shared_ptr<const sdf::Element> &,  
                   ignition::gazebo::EntityComponentManager &,  
                   ignition::gazebo::EventManager &) override
    {
      // Initialize ROS if needed
      if (!rclcpp::ok()) rclcpp::init(0, nullptr);
      rclcpp_node_ = std::make_shared<rclcpp::Node>("pose_relay_node");
      publisher_ = rclcpp_node_->create_publisher<my_sim_plugins::msg::BlockPoseArray>("/object_pose", 10);
      // Subscribe immediately to Gazebo topic
      ign_node_.Subscribe("/world/blocksworld/pose/info", &PoseRelayPlugin::OnPoseMsg, this);
      // Create a ROS timer to republish at 10 Hz
      timer_ = rclcpp_node_->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&PoseRelayPlugin::PublishLatest, this)
      );
    }
    
    void PostUpdate(const ignition::gazebo::UpdateInfo &,
                    const ignition::gazebo::EntityComponentManager &) override
    {
      // Run ROS callbacks
      if (rclcpp_node_ && rclcpp::ok()) rclcpp::spin_some(rclcpp_node_);
    }
    
    ~PoseRelayPlugin()
    {
      std::cout << "PoseRelayPlugin: Destructor" << std::endl;
      rclcpp_node_.reset();
    }
    
  private:
    // Callback: store the latest message
    void OnPoseMsg(const ignition::msgs::Pose_V &msg)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_msg_ = msg;  // copy incoming data
    }

    // Timer callback: publish stored latest_msg_
    void PublishLatest()
    {
      ignition::msgs::Pose_V msg;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        msg = latest_msg_;
      }
      if (msg.pose_size() == 0) return;

      my_sim_plugins::msg::BlockPoseArray array_msg;
      array_msg.header.stamp = rclcpp_node_->now();
      array_msg.header.frame_id = "world";
      for (const auto &pose : msg.pose()) {
        if (pose.name().rfind("block", 0) != 0) continue;
        my_sim_plugins::msg::BlockPose bp;
        bp.name = pose.name();
        bp.pose.position.x = pose.position().x();
        bp.pose.position.y = pose.position().y();
        bp.pose.position.z = pose.position().z();
        bp.pose.orientation.x = pose.orientation().x();
        bp.pose.orientation.y = pose.orientation().y();
        bp.pose.orientation.z = pose.orientation().z();
        bp.pose.orientation.w = pose.orientation().w();
        array_msg.blocks.push_back(std::move(bp));
      }
      if (!array_msg.blocks.empty()) publisher_->publish(array_msg);
    }

    ignition::transport::Node ign_node_;
    std::shared_ptr<rclcpp::Node> rclcpp_node_;
    rclcpp::Publisher<my_sim_plugins::msg::BlockPoseArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex mutex_;
    ignition::msgs::Pose_V latest_msg_;
  };
  
} 

IGNITION_ADD_PLUGIN(
  my_sim_plugins::PoseRelayPlugin,
  ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure,
  ignition::gazebo::ISystemPostUpdate);