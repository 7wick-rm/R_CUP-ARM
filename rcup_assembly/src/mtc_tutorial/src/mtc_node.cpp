#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include "link_attatcher/srv/attach.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <std_msgs/msg/empty.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  explicit MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask(const std::string& object_id,const std::vector<std::double> & pose_position,const std::vector<std::double> & pose_orientation,const std::vector<std::string> & addingcollisions);

  void setupPlanningScene();
  void publishattatch(const std::string& model1_name,const std::string& link1_name,const std::string& model2_name,const std::string& link2_name);

private:
  mtc::Task createTask(const std::string& object_id,const std::vector<std::double> & pose_position,const std::vector<std::double> & pose_orientation,const std::vector<std::string> & addingcollisions);
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

void MTCTaskNode::setupPlanningScene()
{

  std::vector<moveit_msgs::msg::CollisionObject> objects; 

  moveit_msgs::msg::CollisionObject object1;
  object1.id = "block1";
  object1.header.frame_id = "base_link";
  object1.primitives.resize(1);
  object1.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object1.primitives[0].dimensions = { 0.0317, 0.0317,0.024};

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.25;
  pose.position.y = 0.0;
  pose.position.z = 0.012;   
  pose.orientation.w = 1.0;
  object1.pose = pose;

  objects.push_back(object1);

  moveit_msgs::msg::CollisionObject object2=object1;
  object2.id="block2";
  object2.primitives[0].dimensions = { 0.063, 0.0317,0.024};
  geometry_msgs::msg::Pose pose2;
  pose2.position.x=0.25;
  pose2.position.y=0.10;
  pose2.position.z=0.012;
  pose2.orientation.w=1.0;
  object2.pose=pose2;

  objects.push_back(object2);

  moveit_msgs::msg::CollisionObject object3=object1;
  object3.id="block3";
  object3.primitives[0].dimensions = { 0.0317, 0.0317,0.024};

  geometry_msgs::msg::Pose pose3;
  pose3.position.x=0.25;
  pose3.position.y=-0.10;
  pose3.position.z=0.012;
  pose3.orientation.w=1.0;
  object3.pose=pose3;

  objects.push_back(object3);



  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObjects(objects);
}
void MTCTaskNode::publishattatch(const std::string& model1_name, const std::string& link1_name,
                                  const std::string& model2_name, const std::string& link2_name)
{
  rclcpp::Client<link_attatcher::srv::Attach>::SharedPtr client =
      node_->create_client<link_attatcher::srv::Attach>("/link_attacher/attach");

  if (!client->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(node_->get_logger(), "Attach service not available");
    return;
  }

  auto request = std::make_shared<link_attatcher::srv::Attach::Request>();
  request->model1_name = model1_name;
  request->model2_name = model2_name;
  request->link1_name  = link1_name;
  request->link2_name  = link2_name;

  auto future = client->async_send_request(request);
  auto status = future.wait_for(std::chrono::seconds(5));

  if (status == std::future_status::ready)
  {
    auto result = future.get();
    RCLCPP_INFO(
      node_->get_logger(),
      "Attach response: success=%s, message=%s",
      result->success ? "true" : "false",
      result->message.c_str());
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call attach service (timeout)");
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}



void MTCTaskNode::doTask(const std::string& object_id,const std::vector<std::double> & pose_position,const std::vector<std::double> & pose_orientation,const std::vector<std::string> & addingcollisions)
{
  task_ = createTask(object_id,pose_position,pose_orientation,addingcollisions);


  try
  {
    task_.init();
  }

  
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}




mtc::Task MTCTaskNode::createTask(const std::string& object_id,const std::vector<std::double> & pose_position,const std::vector<std::double> & pose_orientation,const std::vector<std::string> & addingcollisions){
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "manipulator";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "tcp";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", "eff");
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.6);
  cartesian_planner->setMaxAccelerationScalingFactor(0.6);
  cartesian_planner->setStepSize(.01);

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));
  
  {
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>("move to pick",mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage_move_to_pick->setTimeout(5.0);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_pick));
  }

  mtc::Stage* attach_object_stage =
    nullptr;  // Forward attach_object_stage to place pose generator


  {

  auto grasp = std::make_unique<mtc::SerialContainer>("Pick Object");
  task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
  grasp->properties().configureInitFrom(mtc::Stage::PARENT,{ "eef", "group", "ik_frame" });
  {

    auto stage=std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "grasp_pose");
    stage->setObject(object_id);
    stage->setPreGraspPose("open");
    stage->setAngleDelta(M_PI/2);
    stage->setMonitoredStage(current_state_ptr);

    Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
    grasp_frame_transform.translation().z() = 0.05;
    grasp_frame_transform.linear()=Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitX()).toRotationMatrix();

  auto wrapper =
      std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
  wrapper->setMaxIKSolutions(8);
  wrapper->setMinSolutionDistance(1.0);
  wrapper->setIKFrame(grasp_frame_transform, hand_frame);
  wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  grasp->insert(std::move(wrapper));



  }

  {
    auto stage=std::make_unique<mtc::stages::ModifyPlanningScene>("allow collisions");
    stage->allowCollisions(object_id,task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),true);
    grasp->insert(std::move(stage));

  }

  {
  auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
  stage->setGroup(hand_group_name);
  stage->setGoal("closed");
  grasp->insert(std::move(stage));
}

  {
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
  stage->attachObject(object_id, hand_frame);
  attach_object_stage = stage.get();
  grasp->insert(std::move(stage));
  }
  

  {
  auto stage =
      std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.05, 0.3);
  stage->setIKFrame(hand_frame);
  stage->properties().set("markerpublishatt_ns", "lift_object");

  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "base_link";
  vec.vector.z = 1.0;
  stage->setDirection(vec);
  grasp->insert(std::move(stage));
}
  task.add(std::move(grasp));
  }

  {
  auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner }
                                                 });
  stage_move_to_place->setTimeout(5.0);
  stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_place));
}
{
  auto place = std::make_unique<mtc::SerialContainer>("place object");
  task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
  place->properties().configureInitFrom(mtc::Stage::PARENT,
                                        { "eef", "group", "ik_frame" });


  {
    auto stage=std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision between objects");
    for (const auto& other : addingcollisions )
    {
      stage->allowCollisions(object_id,other,true);

    } 
    place->insert(std::move(stage));
  }

  


  {
  auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  stage->properties().set("marker_ns", "place_pose");
  stage->setObject(object_id);

  geometry_msgs::msg::PoseStamped target_pose_msg;
  target_pose_msg.header.frame_id = "base_link";
  for (auto &pose : )
  stage->setPose(target_pose_msg);
  stage->setMonitoredStage(attach_object_stage); 

  auto wrapper =
      std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
  wrapper->setMaxIKSolutions(8);
  wrapper->setMinSolutionDistance(1.0);
  wrapper->setIKFrame(object_id);
  wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  place->insert(std::move(wrapper));
}
{
  auto stage=std::make_unique<mtc::stages::MoveRelative>("Approach down",cartesian_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT,  { "group" });
  stage->setMinMaxDistance(0.00,0.002);
  
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "base_link";
  vec.vector.z = -1.0;
  stage->setDirection(vec);
  place->insert(std::move(stage));
  
}


{
  auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage->setGroup(hand_group_name);
  stage->setGoal("open");
  place->insert(std::move(stage));
}

{
  auto stage =
      std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
  stage->allowCollisions(object_id,
                        task.getRobotModel()
                            ->getJointModelGroup(hand_group_name)
                            ->getLinkModelNamesWithCollisionGeometry(),
                        false);
  place->insert(std::move(stage));
}

{
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
  stage->detachObject(object_id, hand_frame);
  place->insert(std::move(stage));
}
{
  auto stage=std::make_unique<mtc::stages::ModifyPlanningScene>("disallow collisions");
  for (const auto & other : addingcollisions )
  {
    stage->allowCollisions(other,object_id,false);
    stage->allowCollisions(other,
      task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
      true);
  }
  place->insert(std::move(stage));
}

{
  auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.01, 0.3);
  stage->setIKFrame(hand_frame);
  stage->properties().set("marker_ns", "retreat");

  // Set retreat direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "base_link";
  vec.vector.z = 0.1;
  stage->setDirection(vec);
  place->insert(std::move(stage));
}

  task.add(std::move(place));
}

{
  auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setGoal("forward");
  task.add(std::move(stage));
}
{
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("close", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("closed");
  task.add(std::move(stage_open_hand));
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

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask("block2",0.25,0.0,0.039,{"block1"});
  mtc_task_node->publishattatch("lego_2x2_yellow","link1","lego_4x2_green","link3");
  mtc_task_node->doTask("block2",0.25,0.1,0.039,{"block1"});
  mtc_task_node->doTask("block3",0.25,0.1,0.063,{"block3","block2","block1"});
  mtc_task_node->publishattatch("lego_4x2_green","link3","lego_2x2_green","link2");
  mtc_task_node->doTask("block3",0.25,0.0,0.063,{"block3","block2","block1"});
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}