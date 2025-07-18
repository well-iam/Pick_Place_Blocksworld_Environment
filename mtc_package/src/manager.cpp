#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <string>
#include <vector>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include "my_sim_plugins/msg/block_pose_array.hpp"

class MtcNodeManager : public rclcpp::Node
{
public:
  MtcNodeManager()
  : Node("mtc_node_manager"), child_pid_(-1)
  {
    simulation_step=0;

    check_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "check_states",
    std::bind(&MtcNodeManager::on_check, this, std::placeholders::_1, std::placeholders::_2));


    //Subsription to Plugin Topic
    subblocks = this->create_subscription<my_sim_plugins::msg::BlockPoseArray>(
    "/object_pose", 10,
    [this](const my_sim_plugins::msg::BlockPoseArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(msg_mutex_);
        last_msg_ = msg;
    }); //Auto-Unlock


    // Subscribe to pyhop_plan topic: each String msg is one action
    substates = this->create_subscription<std_msgs::msg::String>(
    "/expected_states",  // topic name
    100,              // buffer size to store sequence
    [this](const std_msgs::msg::String::SharedPtr msg) {
      expectedstates.push_back(msg->data);
      RCLCPP_INFO(this->get_logger(), "Expected State'%s' added to state history",
                  msg->data.c_str());
      });
    RCLCPP_INFO(this->get_logger(), "mtc_node_manager ready");

    spawnTaskNode(); //Finally, start mtc_node (by running its launch file)
  }

private:
  // PID del processo figlio
  pid_t child_pid_;

  // Service server
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr check_srv_; //check service
  
  rclcpp::Subscription<my_sim_plugins::msg::BlockPoseArray>::SharedPtr subblocks;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr substates;
  std::mutex msg_mutex_;                                            //Used to manage the resource when accessed
  std::shared_ptr<my_sim_plugins::msg::BlockPoseArray> last_msg_; 
  std::map<std::string, std::string> pos;

  std::vector<std::string> expectedstates; 
  int simulation_step;

  // Callback spawn
  bool spawnTaskNode() {
    if (child_pid_ > 0)
      return false;  // already running

    pid_t pid = fork();
    if (pid < 0) return false;
    if (pid == 0) {
      execlp("ros2","ros2","launch","mtc_package","pick_place.launch.py",nullptr);
      _exit(127);
    }
    child_pid_ = pid;
    RCLCPP_INFO(get_logger(), "Spawned MTCTaskNode pid=%d", pid);
    return true;
  }

  //Callback kill
  bool killTaskNode() {
    if (child_pid_ <= 0) return false;
    kill(child_pid_, SIGINT);
    int status = 0;
    waitpid(child_pid_, &status, 0);
    RCLCPP_INFO(get_logger(), "Killed MTCTaskNode pid=%d, status=%d",
                child_pid_, WEXITSTATUS(status));
    child_pid_ = -1;
    return true;
  }

  void computeCurrentState() {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    if (!last_msg_ || last_msg_->blocks.empty()) {
      RCLCPP_WARN(this->get_logger(), "No block poses received yet.");
      return;
    }

    // pos: block → support (table or other block)
    //std::map<std::string, bool> clear;        // block → true if no block on top

    // Compute pos map: where each block is placed
    for (const auto& block : last_msg_->blocks) {
      const std::string& name = block.name;
      const auto& pose = block.pose;

      if (name.rfind("block", 0) != 0)  // skip if name doesn't start with "block"
        continue;

      if (pose.position.z < 0.053) {
        pos[name] = "table";
      } else {
        std::string under = "table";
        for (const auto& other : last_msg_->blocks) {
          if (other.name == name || other.name.rfind("block", 0) != 0)
            continue;

          double dz = pose.position.z - other.pose.position.z;
          double dx = std::abs(pose.position.x - other.pose.position.x);
          double dy = std::abs(pose.position.y - other.pose.position.y);

          if (dz < 0.088 && dz > 0 && dx < 0.018 && dy < 0.018) {
            under = other.name;
            break;
          }
        }
        pos[name] = under;
      }
    }

    // Print the state (optional)
    RCLCPP_INFO(this->get_logger(), "=== Current Stack State ===");
    for (const auto& [block, support] : pos) 
      RCLCPP_INFO(this->get_logger(), "Block %s is on %s", block.c_str(), support.c_str());
    
}


  void on_check(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {

    (void)request; //req is empty. is just a call
    computeCurrentState();


    // 1) Build the serialization exactly matching your '/expected_states' format:
    std::ostringstream oss;
    oss << "{";
    bool first = true;
    for (const auto & [block, support] : pos) {
      if (!first) {
        oss << ", ";
      }
      oss << "'" << block << "': '" << support << "'";
      first = false;
    }
    oss << "}";
    std::string actual_state_str = oss.str();

    // 2) Look up the expected state string at the same index
    if (simulation_step < expectedstates.size()) {
      const std::string & expected_str = expectedstates[simulation_step];

      // 3) Compare them
      if (actual_state_str == expected_str) { //Comparing string states
        RCLCPP_INFO(this->get_logger(),
                    "State %zu matches expected: %s", simulation_step, actual_state_str.c_str());
        //In this case, mtc_node can continue its routine.
        simulation_step+=2;
        res->success = true;
        res->message = "States match";
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "State %zu mismatch:\n  expected: %s\n  actual  : %s",
                     simulation_step, expected_str.c_str(), actual_state_str.c_str());
        //In this case, mtc_node has to be killed, resetted and respawned propely.

        //1) Kill
        if(killTaskNode()){
           RCLCPP_INFO(this->get_logger(), "mtc_node has been killed properly. Resetting...");

           //2) Wait
            const auto wait_ms = 500;  // milliseconds to wait before respawn
            RCLCPP_INFO(this->get_logger(), "Waiting %d ms before respawn", wait_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));

           //3) Spawn
            if(spawnTaskNode())
              RCLCPP_INFO(this->get_logger(), "mtc_node has been re-spawned properly. Restarting...");
            res->success = false;
            res->message = "States differ";
          }

        simulation_step=0;   
      }
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "No expected state available for index %zu", simulation_step);
    }

    
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto manager = std::make_shared<MtcNodeManager>();
  rclcpp::spin(manager);
  rclcpp::shutdown();
  return 0;
}
