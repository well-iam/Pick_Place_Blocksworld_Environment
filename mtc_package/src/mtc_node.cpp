#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <std_msgs/msg/string.hpp>
#include "my_sim_plugins/msg/block_pose_array.hpp" //To recognise  msgtype of subscribed topic

#include "BlockFollower.cpp" //Plugin Implemented to "attach" gazebo blocks to the End-Effector

// C++ Standard Library (Plan decomposition)
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include <cctype>
#include <sstream>
#include <random>  

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_package");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask(const std::string&, const std::string&); //Added input args

  void setupPlanningScene();

  void clearOldScene();

  bool hasReceivedMsg();    //Debug method

  //Plan decomposition methods:

  std::vector<std::pair<std::vector<std::string>, std::vector<std::string>>>
    splitIntoPairs(const std::vector<std::vector<std::string>>& tokenized_plan);

  static std::string clean(const std::string& );

  static std::vector<std::string> splitTokens(const std::string& , char );

void extractPickPlace(
  const std::pair<std::vector<std::string>, std::vector<std::string>>& action_pair,
  std::string& out_block,
  std::string& out_target
);
  std::vector<std::string> plan_actions_; //To access it in the main

  //Helper for thread to follow blocks
  void stopWatcher();


private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask(const std::string&, const std::string&); //Added input args
  // mtc::Task task_; //It was a class member. To ensure clear variable along the pair action, let it be local variable
  rclcpp::Node::SharedPtr node_;

  //Added member to use setupPlanningScene
  rclcpp::Subscription<my_sim_plugins::msg::BlockPoseArray>::SharedPtr sub_blocks_;
  std::mutex msg_mutex_;                                            //Used to lock resource of the msg
  std::shared_ptr<my_sim_plugins::msg::BlockPoseArray> last_msg_;    //Ptr of msg type of topic /object_pose

  //Added member to manage plan execution
  // Subscription to receive sequence of action names (each msg is a string)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_plan_;

  //Members needed to use the BlockFollower Plugin
  std::shared_ptr<BlockFollower> block_follower_;
  std::thread watcher_thread_;            // default‐constructed, not joinable
  std::atomic<bool> watcher_done_{false}; //IF true, stop as soon as you can
  
};

 void MTCTaskNode::stopWatcher() {
    if (watcher_thread_.joinable()) {
      watcher_done_ = true;
      watcher_thread_.join();
      watcher_done_ = false;
    }
  }

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  RCLCPP_INFO(node_->get_logger(), "Start Constructor"); 

  //Subscription to /object_pose topic
  sub_blocks_ = node_->create_subscription<my_sim_plugins::msg::BlockPoseArray>(
  "/object_pose", 10,
  [this](const my_sim_plugins::msg::BlockPoseArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    last_msg_ = msg;
  }); //Automatic Unlock

  
  //Subscription to /pyhop_plan topic: each String msg is one action
  sub_plan_ = node_->create_subscription<std_msgs::msg::String>(
    "/pyhop_plan",  
    100,              
    [this](const std_msgs::msg::String::SharedPtr msg) {
      plan_actions_.push_back(msg->data);
      RCLCPP_INFO(node_->get_logger(), "Appended action '%s' to plan (total: %lu)",
                  msg->data.c_str(), plan_actions_.size());
      });

  RCLCPP_INFO(node_->get_logger(), "Executed Constructor");
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

bool MTCTaskNode::hasReceivedMsg()
{
  std::lock_guard<std::mutex> lock(msg_mutex_);
  return (last_msg_ != nullptr);
}


void MTCTaskNode::setupPlanningScene()
{
  clearOldScene(); 
  RCLCPP_INFO(node_->get_logger(), "Resetted and updating the Planning Scene");
  std::shared_ptr<my_sim_plugins::msg::BlockPoseArray> msg_copy;
  {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    msg_copy = last_msg_;
  }

  moveit::planning_interface::PlanningSceneInterface psi;



  //Build the new planning scene
  std::vector<moveit_msgs::msg::CollisionObject> objs;
  objs.reserve(msg_copy->blocks.size());
  RCLCPP_INFO(node_->get_logger(), "Frame ID in msg_copy: '%s'", msg_copy->header.frame_id.c_str());
  for (const auto& block : msg_copy->blocks) {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = block.name;
    obj.header.frame_id = msg_copy->header.frame_id;

    obj.primitives.resize(1);
    obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    obj.primitives[0].dimensions = { 0.1, 0.1, 0.1 }; //Must Match the Size in the  model.sdf

    obj.pose = block.pose;
    objs.push_back(std::move(obj));
    RCLCPP_INFO(node_->get_logger(), "Block name: %s", block.name.c_str());
  }
  psi.applyCollisionObjects(objs);
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  RCLCPP_INFO(node_->get_logger(), "Planning scene updated with %lu blocks", objs.size());

}


void MTCTaskNode::clearOldScene()
{
  moveit::planning_interface::PlanningSceneInterface psi;

  // 1) get every object in the scene
  const auto world_objs = psi.getKnownObjectNames(); 
  if (world_objs.empty())
    return;
  RCLCPP_INFO(node_->get_logger(),
            "Scene before clear: %zu world-objects",
            world_objs.size());

  // 2a) remove ALL old world objects
  std::vector<std::string> to_remove;
  to_remove.reserve(world_objs.size());
  for (auto& kv : world_objs){
    
    moveit_msgs::msg::CollisionObject co;
    co.id        = kv;
    co.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    psi.applyCollisionObject(co);
    to_remove.push_back(std::move(co.id));
  }
 
  if (!to_remove.empty())
  {
    RCLCPP_INFO(node_->get_logger(),
                "Removed %zu stale world objects", to_remove.size());
  }

  std::this_thread::sleep_for(std::chrono::seconds(3));
}

//UTILITY FUNCTION TO ELABORATE THE PLAN -----------------------------------------------

// Remove spaces, quote marks, and parentheses
static std::string clean(const std::string& s) {
  std::string t = s;
  t.erase(std::remove_if(t.begin(), t.end(),
                         [](char c){ return c=='('||c==')'||c=='\''; }),
          t.end());
  t.erase(std::remove_if(t.begin(), t.end(),
                         [](char c){ return std::isspace(c); }),
          t.end());
  return t;
}

// Split the string 'PRIMITIVE,BLOCK_ID,..' into a vector of tokens:[PRIMITIVE,BLOCK_ID,..]
static std::vector<std::string> splitTokens(const std::string& s, char delim=',') {
  std::vector<std::string> tokens;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    tokens.push_back(item);
  }
  return tokens;
}

// Utility to split cleaned & tokenized plan into consecutive pick/place pairs (The Task Constructor exectues sequences of pick and place)
std::vector<std::pair<std::vector<std::string>, std::vector<std::string>>>
MTCTaskNode::splitIntoPairs(const std::vector<std::vector<std::string>>& tokenized_plan)
{
  std::vector<std::pair<std::vector<std::string>, std::vector<std::string>>> pairs;

  for (size_t i = 0; i + 1 < tokenized_plan.size(); i += 2) {
    pairs.emplace_back(tokenized_plan[i], tokenized_plan[i + 1]);
  }

  return pairs;
}


// Extracts the BLOCK_ID to be picked and the placement TARGET_ID from a tokenized action pair
void MTCTaskNode::extractPickPlace(const std::pair<std::vector<std::string>, std::vector<std::string>>& action_pair,
                      std::string& out_block, std::string& out_target)
{  // action pair = [ ["unstack", "block_a", "block_c"] , ["putdown", "block_a"] ] 
  const auto& pick_tokens = action_pair.first;   // e.g., ["unstack", "block_a", "block_c"]
  const auto& place_tokens = action_pair.second; // e.g., ["putdown", "block_a"]

  // For the pick task, the BLOCK_ID of the object to pick is always the second token in the first action of the pair (there will be no difference beetwen unstack and pickup)
  out_block = pick_tokens[1]; //block_id

 // For the place task, the TARGET_ID changes wheater the action to be performed is "stack" or "place"
  if (place_tokens[0] == "stack" && place_tokens.size() >= 3) { //(stack case)
    // Target is another block
    out_target = place_tokens[2]; 
  } else { //(putdown case)
    out_target = "table";
  }
}

//-------------------------------------------------------------------------------


//Modified to implement max_attempts run of the assigned task
void MTCTaskNode::doTask(const std::string& block_id, const std::string& target_id)
{
  const int max_attempts = 5;
  bool planning_success = false;
  mtc::Task task_;  //Local task
  for (int attempt = 1; attempt <= max_attempts; ++attempt)
  {
    RCLCPP_INFO(LOGGER, "Task planning attempt %d of %d", attempt, max_attempts);

    task_ = createTask(block_id, target_id);

    try
    {
      task_.init();
    }
    catch (mtc::InitStageException& e)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Task initialization failed: " << e);
      return;
    }

    if (task_.plan(1))  // Try to find at least one solution
    {
      planning_success = true;
      RCLCPP_INFO(LOGGER, "Task planning succeeded on attempt %d", attempt);
      break;
    }
    else
    {
      RCLCPP_WARN(LOGGER, "Task planning failed on attempt %d", attempt);
    }
  }

  if (!planning_success)
  {
    RCLCPP_ERROR(LOGGER, "Task planning failed after %d attempts", max_attempts);
    return;
  }

  // RViz Visualization
  task_.introspection().publishSolution(*task_.solutions().front());

    // 1) Create the follower, but don't start it yet:
  block_follower_ = std::make_shared<BlockFollower>(node_, block_id);

    watcher_done_ = false;
    watcher_thread_ = std::thread([this, block_id]() {
      try {
        moveit::planning_interface::PlanningSceneInterface psi;
        bool started = false;
        while (!watcher_done_ && rclcpp::ok()) {
          auto attached = psi.getAttachedObjects({block_id});
          bool is_attached = !attached.empty();
          if (is_attached && !started) {
            block_follower_->start();
            started = true;
          }
          if (!is_attached && started) {
            block_follower_->stop();
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(node_->get_logger(), "Watcher exception: %s", e.what());
      } catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "Watcher unknown exception");
      }
    });

  auto result = task_.execute(*task_.solutions().front());
  stopWatcher();
  block_follower_.reset();
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }
}


mtc::Task MTCTaskNode::createTask(const std::string& block_id, const std::string& target_id)
{


  mtc::Task task;                       //creates a task
  task.stages()->setName("Pick-Place Task");  //set the task name to 
  task.loadRobotModel(node_);           //load the robot model (from launch file u get the info)

  //groups and frames used for planning
  const auto& arm_group_name = "ur5e_arm";
  const auto& hand_group_name = "hand";   
  const auto& hand_frame = "flange";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name); 
  task.setProperty("ik_frame", hand_frame);

  //CURRENT STATE: Captures current Robot State
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator (USed after)
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current"); //New stage
  current_state_ptr = stage_state_current.get(); //Pointer to the new stage (used afeter)
  task.add(std::move(stage_state_current));


  // MoveIt Task Constructor has three options for solvers:
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  //Set the cartesian solver (Modified step size and Timeout)
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.2);
  cartesian_planner->setMaxAccelerationScalingFactor(0.2);
  cartesian_planner->setStepSize(.003);

  auto slow_cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  slow_cartesian_planner->setMaxVelocityScalingFactor(0.1);
  slow_cartesian_planner->setMaxAccelerationScalingFactor(0.1);
  slow_cartesian_planner->setStepSize(.002);

  auto very_slow_cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  very_slow_cartesian_planner->setMaxVelocityScalingFactor(0.003);
  very_slow_cartesian_planner->setMaxAccelerationScalingFactor(0.003);
  very_slow_cartesian_planner->setStepSize(.0001);


 //CONNECT: STAGE: creates full plan for the pick by bridging current state and one of the possible grasp generated pose.
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));
  mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator

  //Pick Object: Serial Container that contains a sequence of suborinate stages for operating the pick
  {
  auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
  task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
  grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                        { "eef", "group", "ik_frame" });

    {
      //Approach Object: approaches the target object along a specified direction
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.02, 0.15);


      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.x = 1.0;       // Set hand forward direction
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }    


    {
// GENERATE GRASP POSE: Generates different poses that correspond to the act of grasping
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject(block_id);
      stage->setAngleDelta(M_PI/8);//Explores different samples differing from such angular value.
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state

      Eigen::Isometry3d grasp_frame_transform; //4x4 matrix.
      //.linear() gives you direct access to that 3×3 rotation block.
      //.translation() accesses the 3×1 translation column 

      // 1) Define the face normal you want (in block frame)
      Eigen::Vector3d face_normal(0, 0, 1);
      face_normal.normalize();

      // 2) Compute quaternion rotating tool’s Z into that normal
      Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(
          Eigen::Vector3d::UnitX(),   // the gripper’s local Z axis
          face_normal                // desired direction on block face
      );
      grasp_frame_transform.linear() = q.matrix(); //q.matrix() converts the quaternion into its equivalent 3×3 rotation matrix

      // Move out along face normal by half the block depth + clearance
      grasp_frame_transform.translation() = Eigen::Vector3d(0.07, 0.0, 0);

      // Compute IK for the generated poses in the previous stages
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8); //set the maximum number of IK solution to search
      wrapper->setMinSolutionDistance(0.1); //forces distance between solution in configuration space
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }
    

      //COLLISION MANAGEMENT: BLOCK-BLOCK, BLOCK-GROUND
      {
      // Build a list of all environment IDs (other blocks + “ground”)
      std::vector<std::string> env_ids;
      {
        std::lock_guard<std::mutex> lock(msg_mutex_);
        for (const auto& b : last_msg_->blocks)
          if (b.name != block_id)    // skip the one we just grabbed
            env_ids.push_back(b.name);
      }
      env_ids.push_back("ground_plane");   // your table or floor collision ID

      auto allow_env = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow block-environment collisions");
      // Whitelist them in the ACM
      allow_env->allowCollisions(block_id, env_ids, true);
      grasp->insert(std::move(allow_env));
    }


    {
   //ATTACH OBJECT: creates a fixed tf in the planning scene between the grasped object and the gripper
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(block_id, hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }


    { //LIFT OBJECT:
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", slow_cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.35);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }


   //COLLISION MANAGEMENT
  {
    // Re‑use the exact same list of IDs you whitelisted before
    std::vector<std::string> env_ids;
    {
      std::lock_guard<std::mutex> lock(msg_mutex_);
      for (const auto& b : last_msg_->blocks)
        if (b.name != block_id)
          env_ids.push_back(b.name);
    }
    env_ids.push_back("ground_plane");

    auto forbid_env = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "forbid block-environment collisions");
    forbid_env->allowCollisions(block_id, env_ids, false);
    grasp->insert(std::move(forbid_env));
  }

  task.add(std::move(grasp));
  } 

 ///---------------------------------------------------------------------------

  { //CONNECT Stage: same as for the move to pick stage, connect the post-lift configuration to one of the possible generated configuration computed to perfom place stage. 
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                  { hand_group_name, interpolation_planner } });
    stage_move_to_place->setTimeout(8.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }


  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    
    {
       

      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(block_id);
      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "world";
      // Decide between table putdown or stacking on another block
      if (target_id == "table") {
         // 1) Define table bounds and heights
        //    You can load these from parameters instead of hard‑coding.
        const double off = 0.007;        // height of tabletop [m]
        const double half_block = 0.05;
        const double place_z = off + half_block;

        const double min_x = 0, max_x = 1;   // example x-bounds of table surface
        const double min_y = -1, max_y = 1;  // example y-bounds

        // 2) Prepare random sampling in the XY box
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dist_x(min_x, max_x);
        std::uniform_real_distribution<> dist_y(min_y, max_y);

        // 3) Prepare RobotState and JointModelGroup for IK
          const moveit::core::RobotModelConstPtr& robot_model = task.getRobotModel();
          moveit::core::RobotState           robot_state(robot_model);
          const auto*                        joint_model_group =
              robot_model->getJointModelGroup(arm_group_name);

          // 4) Sample until a reachable pose is found (or timeout)
          const int max_trials = 50;
          for (int trial = 0; trial < max_trials; ++trial) {
           double x = dist_x(gen), y = dist_y(gen);
          // reject any sample inside the central 0.2×0.2m square
           if (std::fabs(x) < 0.2 && std::fabs(y) < 0.2)
             continue;
      
            geometry_msgs::msg::PoseStamped candidate;
            candidate.header.frame_id = "world";
            candidate.pose.position.x  = x;
            candidate.pose.position.y  = y;
            candidate.pose.position.z  = place_z;
            // Identity quaternion → aligned with world frame
            candidate.pose.orientation.x = 0.0;
            candidate.pose.orientation.y = 0.0;
            candidate.pose.orientation.z = 0.0;
            candidate.pose.orientation.w = 1.0;

            // Try IK, check for existence of IK solution for that pose.
            bool IK_exist = robot_state.setFromIK(
              joint_model_group,
              candidate.pose,
              hand_frame,    // end‑effector link
              0.1            // 100ms timeout
            );
            if (IK_exist) {
              // Finally, check if it is too close to other blocks
              bool min_block_distance=true;
              {
                std::lock_guard<std::mutex> lock(msg_mutex_);
                for (const auto& block : last_msg_->blocks)
                    if(std::fabs(block.pose.position.x-x)<0.15 && std::fabs(block.pose.position.y-y)<0.15)
                      min_block_distance=false;
                  }

                  if(min_block_distance){
                    stage->setPose(candidate);
                    break;
                  }
              } 
          }
        
      } else { 
        // Stack onto an existing block: find its current pose
        const double off = 0.007;        // height of tabletop [m]
        const double block = 0.1;
        const double place_z = off + block;
        
        {
          // Lock access to last_msg_, which holds all block poses
          std::lock_guard<std::mutex> lock(msg_mutex_);
          for (const auto& block : last_msg_->blocks) {
            if (block.name == target_id) {
              // Copy the block’s pose and offset in Z by half the block’s height
              target_pose_msg.pose = block.pose;
              target_pose_msg.pose.position.z += place_z; //0.001 di margine
              //IF this value is too high when approcchin you vibrate
              stage->setPose(target_pose_msg);
              break;
            }
          }
        }
      }

      
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(block_id);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
    // Add a settling stage before detaching for stacking
      auto settle_stage = std::make_unique<mtc::stages::MoveRelative>("settle before detach", very_slow_cartesian_planner);
      settle_stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      settle_stage->setMinMaxDistance(0.0059, 0.006);  // Tiny movement to allow settling
      settle_stage->setIKFrame(hand_frame);
      settle_stage->properties().set("marker_ns", "settle");
      // 
      // Very small downward movement
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = -1.0;
      settle_stage->setDirection(vec);
      place->insert(std::move(settle_stage));
  }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(block_id, hand_frame);
      place->insert(std::move(stage));
    }

    

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.2); 
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0; //changed to retrive with z direction after realead the block
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    task.add(std::move(place));
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("home");
    task.add(std::move(stage));
  }
  
  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  // Wait for first message from /object_pose, timeout 5 seconds
  {
    const int max_wait_ms = 5000;
    int waited = 0;
    const int wait_step = 100; // ms
    while (rclcpp::ok() && !mtc_task_node->hasReceivedMsg() && waited < max_wait_ms) {
      std::this_thread::sleep_for(std::chrono::milliseconds(wait_step));
      waited += wait_step;
    }
    if (!mtc_task_node->hasReceivedMsg()) {
      RCLCPP_ERROR(LOGGER, "Timeout waiting for block pose messages");
      return 1;
    }
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));

  //For now, the plan is a vector of string in the form "(‘unstack’, ‘block_a’, ‘block_c’)" or "(‘putdown’, ‘block_a’)"
   RCLCPP_INFO(LOGGER, "Waiting for the Plan..");

  std::vector<std::vector<std::string>> CleanSplitPlan; //vector for Token actions
  const int plan_wait_ms = 10000;
  int waited      = 0;
  const int pause = 100;

  //Wait for first message from /phyop_plan, timeout 10 seconds
  while (rclcpp::ok() && mtc_task_node->plan_actions_.size() < 2 && waited < plan_wait_ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(pause));
  waited += pause;
  }
   // 1)Calls Clean on all action of the plan to clean and Tokenise it
  for (size_t i = 0; i < mtc_task_node->plan_actions_.size(); ++i) {
    std::string cleaned = clean(mtc_task_node->plan_actions_[i]);
    std::vector<std::string> tokens = splitTokens(cleaned);
    CleanSplitPlan.push_back(tokens);
    
  }

  //Now, every string in the plan has become a vector of string which elements are [unstack, block_a, block_c]

  //2) Build pair of task, in order to call the doTask method to perform a pick-place pair of the plan
  //Assumtion: Plan contains pick (that could be pickup or unstack ) and place (putdown or stack) action, in order.

  auto pairs = mtc_task_node->splitIntoPairs(CleanSplitPlan);

  //Now, every element of pairs is a pair of task, that is a sequence pick-place (that is a vector of string).
  //3)Identify block_id for the pick stage (who needs to be picked?), and a pose for the place stage (where needs to be placed?)

  // Loop over each pair and extract block and target, executing the pick-place associated task
  for (const auto& pair : pairs) {


    RCLCPP_INFO(LOGGER, "Elaborating Pair of action");
    std::string block_id;
    std::string target_id;
    mtc_task_node->extractPickPlace(pair, block_id, target_id);

    mtc_task_node->setupPlanningScene();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    mtc_task_node->doTask(block_id,target_id);

    //Ensure the task is phisically ended:
    std::this_thread::sleep_for(std::chrono::seconds(2));

    
  }


  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
//Called by pick_place_demo.launch.py