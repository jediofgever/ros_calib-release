/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-28 10:12:38
 * @modify date 2019-11-28 10:12:38
 * @desc [description]
 */
#include <common/RobotController.h>

/**
 * @brief Construct a new Robot Controller:: Robot Controller object
 *
 */
RobotController::RobotController(/* args */) {
    // ros node handle pointer
    node_handle_ptr_ = new ros::NodeHandle();
    visual_tools_ptr_.reset(new moveit_visual_tools::MoveItVisualTools("base_link", "/moveit_visual_markers"));
}

/**
 * @brief Destroy the Robot Controller:: Robot Controller object
 *
 */
RobotController::~RobotController() { delete node_handle_ptr_; }

/**
 * @brief moves robot tcp in joint space, a straight line following is not guarenteed
 *
 * @param robot_tcp_goal_in_joint_space
 * @param move_group_ptr_
 * @return when it finishs the task
 */
bool RobotController::moveEndEffectortoGoalinJointSpace(
    geometry_msgs::Pose robot_tcp_goal_in_joint_space,
    moveit::planning_interface::MoveGroupInterface *move_group_ptr_) {
    bool returnMoveitResult = false;
    // set pose target for tcp
    move_group_ptr_->setPoseTarget(robot_tcp_goal_in_joint_space);

    // Now, we call the planner to compute the plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // chechk wheter a sucessfull plan was found
    bool success = (move_group_ptr_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // stream some info about this sucess state
    ROS_INFO_NAMED("Motion Plan for End-Effector %s", success ? "SUCCESSED" : "FAILED");

    // publish a path to visualize planned path
    publishRobotTrajectory(move_group_ptr_, my_plan.trajectory_);

    // chechk if any of joints will rotate more than a reasonable amount, return a false and not execute path if there
    // is some creazy plan found
    std::vector<double> min_joint_positions = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
    std::vector<double> max_joint_positions = {-100.0, -100.0, -100.0, -100.0, -100.0, -100.0};

    std::vector<double> joint_roation_limits = {M_PI, 3 * M_PI_4, 3 * M_PI_4, M_PI, M_PI, 2 * M_PI};

    for (size_t i = 0; i < my_plan.trajectory_.joint_trajectory.points.size(); i++) {
        trajectory_msgs::JointTrajectoryPoint this_point = my_plan.trajectory_.joint_trajectory.points[i];
        // float joint_1_pos, joint_2_pos, joint_3_pos, joint_4_pos, joint_5_pos, joint_6_pos;
        for (size_t j = 0; j < this_point.positions.size(); j++) {
            if (this_point.positions[j] < min_joint_positions[j]) {
                min_joint_positions[j] = this_point.positions[j];
            }
            if (this_point.positions[j] > max_joint_positions[j]) {
                max_joint_positions[j] = this_point.positions[j];
            }
        }
    }
    for (size_t i = 0; i < min_joint_positions.size(); i++) {
        double biggest_rotation_in_this_joint = std::abs(max_joint_positions[i] - min_joint_positions[i]);
        if (biggest_rotation_in_this_joint > joint_roation_limits[i]) {
            ROS_ERROR("THIS PLAN WILL VIOLATE THE DEFINED JOINT LIMITS OF %d, SKIPPING THE PLAN AND NOT EXECUTING!!!",
                      i);
            return false;
        }
    }

    if (success) {
        // Move the manipulator according to Updated End-Effector goal pose
        returnMoveitResult =
            (move_group_ptr_->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    } else {
        ROS_ERROR("Could not find a valid path plan to the target pose!!!");
    }

    // retun result of plan execution
    return returnMoveitResult;
}

/**
 * @brief moves robot tcp in joint space, a straight line following is  guarenteed
 *
 * @param robot_tcp_goal_in_cartesian_space
 * @param move_group_ptr_
 * @return when it finishs the task
 */
bool RobotController::moveEndEffectortoGoalinCartesianSpace(
    geometry_msgs::Pose robot_tcp_goal_in_cartesian_space,
    moveit::planning_interface::MoveGroupInterface *move_group_ptr_) {
    bool returnMoveitResult = false;
    // get current pose ob tcp, because we will calculate a linear path, which needs current and target poses
    geometry_msgs::Pose current_pose = move_group_ptr_->getCurrentPose().pose;

    // We have two way point to calculate a linear path in between
    std::vector<geometry_msgs::Pose> waypoints;

    // Push current pose and target pose to way points
    waypoints.push_back(current_pose);
    waypoints.push_back(robot_tcp_goal_in_cartesian_space);

    // compute a cartesian(linear) line between current and target pose
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_ptr_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    // stream how much percent of this cartesian path can be achieved
    ROS_INFO("Visualizing plan for (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // We dont actually plan, but we need to set the trajectory of plan to calculated cartesian path
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    my_plan.trajectory_ = trajectory;

    publishRobotTrajectory(move_group_ptr_, my_plan.trajectory_);

    // Move the manipulator according to calculated cartesian(linear) path
    returnMoveitResult = (move_group_ptr_->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    return returnMoveitResult;
}

/**
 * @brief moves robot along End-effector(TOOL) link frames
 *
 * @param robot_tcp_goal_in_tool_space
 * @param move_group_ptr_
 * @param listener_
 * @return when it finishs the task
 */
bool RobotController::moveEndEffectortoGoalinToolSpace(geometry_msgs::Pose robot_tcp_goal_in_tool_space,
                                                       moveit::planning_interface::MoveGroupInterface *move_group_ptr_,
                                                       tf::TransformListener *listener_) {
    /******************************** READ FUNCTION DESCRIPTION BELOW   ***********************************/
    /*
    This function moves robot tcp in tool link according to given amount of distance by user/program
    The given goal is in tool_link, this goal is transformed to base_link and then a cartesian/linear path is calculated
     */

    bool returnMoveitResult = false;
    // get given goal postion(in tool_link frame)
    geometry_msgs::Pose msg_robot_goal_in_tool_link;
    msg_robot_goal_in_tool_link.position = robot_tcp_goal_in_tool_space.position;

    // we will trasnform the given goal from tool_link to base_link
    tf::Pose tf_robot_goal_in_tool_link;
    tf::poseMsgToTF(msg_robot_goal_in_tool_link, tf_robot_goal_in_tool_link);

    // get transform between tool_link and base_link
    tf::StampedTransform tool_to_base_link_transform;
    // lookup transform (this should be cached, since it’s probably static)
    listener_->lookupTransform("base_link", "tool0", ros::Time(0.0f), tool_to_base_link_transform);

    // do the transform by multipliying transfrom matrice with goal pose in tool_link
    tf::Pose tf_robot_goal_in_base_link;
    tf_robot_goal_in_base_link = tool_to_base_link_transform * tf_robot_goal_in_tool_link;

    // MOVEIT uses geometry_msgs type , so convert transformed goal pose to geometry_msgs
    geometry_msgs::Pose msg_robot_goal_in_base_link;
    tf::poseTFToMsg(tf_robot_goal_in_base_link, msg_robot_goal_in_base_link);

    // we will do a linear plan between current pose and given target pose, so push the current pose to waypoint vector
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(move_group_ptr_->getCurrentPose().pose);

    // Since we are moving along tool_link , the orintation of robot tcp shall not change
    msg_robot_goal_in_base_link.orientation = move_group_ptr_->getCurrentPose().pose.orientation;
    waypoints.push_back(msg_robot_goal_in_base_link);

    // compute a cartesian path between target pose(in base_link) and current pose
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_ptr_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    // stream how much percent of this cartesian path can be achieved
    ROS_INFO("Visualizing plan for (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // We dont actually plan, but we need to set the trajectory of plan to calculated cartesian path
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    my_plan.trajectory_ = trajectory;

    publishRobotTrajectory(move_group_ptr_, my_plan.trajectory_);

    // Move the manipulator according to calculated cartesian(linear) path
    returnMoveitResult = (move_group_ptr_->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    return returnMoveitResult;
}

/**
 * @brief moves each individual joint to given target position
 *
 * @param robot_joint_states
 * @param move_group_ptr_
 * @return when it finishs the task
 */
bool RobotController::moveJointstoTargetPositions(std::vector<double> robot_joint_states,
                                                  moveit::planning_interface::MoveGroupInterface *move_group_ptr_) {
    bool returnMoveitResult = false;
    // Set new joint states with modified Vector
    move_group_ptr_->setJointValueTarget(robot_joint_states);

    // Now, we call the planner to compute the plan
    moveit::planning_interface::MoveGroupInterface::Plan move_joint_plan;

    // check if a valid plan was found for this joint targets
    bool success = (move_group_ptr_->plan(move_joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("Motion Plan for Joint %s", success ? "SUCCESSED" : "FAILED");

    publishRobotTrajectory(move_group_ptr_, move_joint_plan.trajectory_);

    if (success) {
        // Move the manipulator according to Updated Joint States
        returnMoveitResult =
            (move_group_ptr_->execute(move_joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    } else {
        ROS_ERROR("Couldnot make a path plan to the target pose!!!");
    }

    return returnMoveitResult;
}

/**
 * @brief not implemented yet, but will open the robot gripper
 *
 */
void RobotController::openGripper() {
    /*** THIS FUNCTION IS USED FOR SIMULATOR ONLY ***/
    /*** THIS FUNCTION WILL BE MODIFIED ****/
}

/**
 * @brief not implemneted yet , but will close robots hand
 *
 */
void RobotController::closeGripper() {
    /*** THIS FUNCTION IS USED FOR SIMULATOR ONLY ***/
    /*** THIS FUNCTION WILL BE MODIFIED ****/
}
/**
 * @brief stops current execution of any trajectory
 *
 * @param move_group_ptr_
 */
void RobotController::stopRobotTrajectrory(moveit::planning_interface::MoveGroupInterface *move_group_ptr_) {
    // Stops current path executions , if any
    move_group_ptr_->stop();
}

/**
 * @brief stops current execution of any trajectory but keeps memory of goal, so the execution can be continued
 *
 * @param move_group_ptr_
 */
void RobotController::pauseRobotTrajectrory(moveit::planning_interface::MoveGroupInterface *move_group_ptr_) {
    // Pauses current path executions , if any
    move_group_ptr_->stop();
}

/**
 * @brief publishes the planned path of robot, and visualizes it in RVIZ
 *
 */
void RobotController::publishRobotTrajectory(moveit::planning_interface::MoveGroupInterface *move_group_ptr_,
                                             moveit_msgs::RobotTrajectory trajectory) {
    // Whicj joint model group's trajectory you would like to visualize ? , in our case "manipulator"
    joint_model_group_ptr_ = move_group_ptr_->getCurrentState()->getJointModelGroup("manipulator");

    // Delete any marker publushed by visual_tools_
    visual_tools_ptr_->deleteAllMarkers();

    // Convert trejectory in joint states to EEF poses
    robot_trajectory::RobotTrajectoryPtr robot_trajectory(
        new robot_trajectory::RobotTrajectory(move_group_ptr_->getRobotModel(), joint_model_group_ptr_->getName()));

    // set the trajectory
    robot_trajectory->setRobotTrajectoryMsg(*move_group_ptr_->getCurrentState(), trajectory);

    // the path points will be pushed into Eigen container
    EigenSTL::vector_Vector3d path;
    // go through each way point of our trajectory
    for (std::size_t i = 0; i < robot_trajectory->getWayPointCount(); ++i) {
        // tip pose W.R.T its parent link
        const Eigen::Isometry3d &tip_pose = robot_trajectory->getWayPoint(i).getGlobalLinkTransform("link_6");

        // Error Check
        if (tip_pose.translation().x() != tip_pose.translation().x()) {
            ROS_INFO("NAN DETECTED AT TRAJECTORY POINT i=");
            return;
        }

        // push this point into path , which krrps waypoints inside
        path.push_back(tip_pose.translation());

        // put a sphere on this way point
        visual_tools_ptr_->publishSphere(tip_pose, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);
    }

    // radius of path
    const double radius = 0.006;

    // publish the path
    visual_tools_ptr_->publishPath(path, rviz_visual_tools::RED, radius);

    // trigger action for moveit_tools_ markers
    visual_tools_ptr_->trigger();
}