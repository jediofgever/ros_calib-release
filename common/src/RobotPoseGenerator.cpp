#include <common/RobotPoseGenerator.h>

/**
 * @brief Construct a new Robot Pose Generator:: Robot Pose Generator object
 *
 */
RobotPoseGenerator::RobotPoseGenerator(/* args */) {
    nh_ = new ros::NodeHandle();
    robot_controller_ = new RobotController();
    static const std::string PLANNING_GROUP = "manipulator";
    move_group_ptr_ = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    random_generated_pose_publisher =
        nh_->advertise<geometry_msgs::PoseArray>("/handeye_calib/random_generated_poses", 1);
    std::cout << "CONSTRUCTED AN INSTANCE OF RobotPoseGenerator" << std::endl;
}

/**
 * @brief Destroy the Robot Pose Generator:: Robot Pose Generator object
 *
 */
RobotPoseGenerator::~RobotPoseGenerator() {
    delete nh_;
    delete robot_controller_;
    delete move_group_ptr_;
    std::cout << "DESTROYED AN INSTANCE OF RobotPoseGenerator" << std::endl;
}

/**
 * @brief RETRUN A RANDOM INTEGER BETWEEN GIVEN BOUNDRIES
 *
 * @param Min
 * @param Max
 * @return int
 */
int RobotPoseGenerator::randint(int Min, int Max) { return std::rand() % (Max + 1 - Min) + Min; }

/**
 * @brief given number variations, generates variations * 6 random poses
 *
 * @param number_of_variants
 */
void RobotPoseGenerator::generatePoses(int number_of_variants) {
    std::cout << "Generating random poses..." << std::endl;

    random_generated_poses_vector.clear();
    geometry_msgs::Pose start_pose = move_group_ptr_->getCurrentPose().pose;
    // note that angles are in radians
    std::vector<double> start_RPY = move_group_ptr_->getCurrentRPY();

    for (size_t i = 0; i < number_of_variants; i++) {
        double rand_translation = randint(kLOWERTHRESHOLDCM, kUPPERTHRESHOLDCM) / 100.0;

        translateAndRotateThroughSingleAxe(rand_translation, start_pose, start_RPY,
                                           RobotPoseGenerator::Signed_Axes_Enum::X_PLUS);
        translateAndRotateThroughSingleAxe(rand_translation, start_pose, start_RPY,
                                           RobotPoseGenerator::Signed_Axes_Enum::X_MINUS);
        translateAndRotateThroughSingleAxe(rand_translation, start_pose, start_RPY,
                                           RobotPoseGenerator::Signed_Axes_Enum::Y_PLUS);
        translateAndRotateThroughSingleAxe(rand_translation, start_pose, start_RPY,
                                           RobotPoseGenerator::Signed_Axes_Enum::Y_MINUS);
        random_generated_poses_vector.push_back(start_pose);
        translateAndRotateThroughDoubleAxes(rand_translation, start_pose, start_RPY,
                                            RobotPoseGenerator::Quadrant_Enum::ONE);
        translateAndRotateThroughDoubleAxes(rand_translation, start_pose, start_RPY,
                                            RobotPoseGenerator::Quadrant_Enum::TWO);
        translateAndRotateThroughDoubleAxes(rand_translation, start_pose, start_RPY,
                                            RobotPoseGenerator::Quadrant_Enum::THREE);
        translateAndRotateThroughDoubleAxes(rand_translation, start_pose, start_RPY,
                                            RobotPoseGenerator::Quadrant_Enum::FOUR);
        random_generated_poses_vector.push_back(start_pose);
    }
    geometry_msgs::PoseArray random_generated_poses_array;
    random_generated_poses_array.header.frame_id = "base_link";
    random_generated_poses_array.header.stamp = ros::Time::now();
    for (size_t i = 0; i < random_generated_poses_vector.size(); i++) {
        random_generated_poses_array.poses.push_back(random_generated_poses_vector[i]);
    }
    random_generated_pose_publisher.publish(random_generated_poses_array);

    std::cout << "Generated random poses:" << random_generated_poses_vector.size() << std::endl;
}

/**
 * @brief given euler angles in radian, returns geometry_msgs::Quaternion ROS type
 * @param robot_rx_radian
 * @param robot_ry_radian
 * @param robot_rz_radian
 * @return geometry_msgs::Quaternion
 */
geometry_msgs::Quaternion RobotPoseGenerator::eulertoQuaternion(double robot_rx_deg, double robot_ry_deg,
                                                                double robot_rz_deg) {
    tf2::Quaternion robot_goal_orientation_quat;
    robot_goal_orientation_quat.setRPY(robot_rx_deg, robot_ry_deg, robot_rz_deg);

    // Normalize the QUATs to make the squarred root sum of x,y,z,w equal to 1.0
    robot_goal_orientation_quat.normalize();

    // Robot Pose should be set in terms of ROS conventations geotmetry_msgs::Pose
    geometry_msgs::Quaternion robot_goal_orientation_geo_msg;

    // tf::Quaternions to geotmetry_msgs::Quaternion
    robot_goal_orientation_geo_msg = tf2::toMsg(robot_goal_orientation_quat);

    return robot_goal_orientation_geo_msg;
}

/**
 * @brief executes the randomly generated pose in random_generated_poses_vector with index of pose_index
 *
 * @param pose_index
 * @return true if pose at pose_index was reachable
 * @return false if pose at pose_index was reachable
 */
bool RobotPoseGenerator::executePose(int pose_index) {
    bool is_success = robot_controller_->moveEndEffectortoGoalinJointSpace(random_generated_poses_vector[pose_index],
                                                                           move_group_ptr_);
    if (is_success) {
        std::cout << "this random pose is valid we are exeuting it ..." << std::endl;
        return true;
    } else {
        ROS_ERROR("bad random pose not going to execute ... ");
        return false;
    }
}

/**
 * @brief   function to calculate the pose for robot TCP , considers the translation, calculates the
 * recorrected angles in order for robot TCP to constantly look at the Marker
 *
 * @param rand_translation
 * @param start_pose
 * @param start_RPY
 * @param signed_axe
 */
void RobotPoseGenerator::translateAndRotateThroughSingleAxe(double rand_translation, geometry_msgs::Pose start_pose,
                                                            std::vector<double> start_RPY, int signed_axe) {
    // translate through given axis but also reccorect the angle so that robot TCP keeps looking at Marker
    // get a temp copy of start pose
    geometry_msgs::Pose temp_random_pose = start_pose;
    // get a temp copy of start rolll pitch yaw angles
    std::vector<double> temp_recorrected_orientation_rpy = start_RPY;
    // get a temp copy of start pose orientation
    geometry_msgs::Quaternion temp_recorrected_orientation_quaternion = start_pose.orientation;
    // this is the angle that needs to be added or substracted to roll, pitch or yaw depending the translation through
    // which axe
    //
    double rotation_recorrection = std::atan2(start_pose.position.z, rand_translation);

    /*
    These are the frame coordinates of robot base_link, we cal calculate a pose with given random translation through
    each of the axes ; X+ ,X- ,Y+,Y-
    When the robot TCP translates thorgh the axes we need to make sure the robot keeps looking at the Marker, so with
    respect to the amount of translation we also calculate the angle to be extracted or added to roll pitch yaw of TCP

                    ^ +X
                    |
                    |
                    |
                    |
    +Y <--------------------------> -Y
                    |
                    |
                    |
                    |
                    |
                    -X
    */
    // This part is self explanatory
    switch (signed_axe) {
        case Signed_Axes_Enum::X_PLUS:
            temp_random_pose.position.x += rand_translation;
            temp_recorrected_orientation_rpy[1] += (1.57 - rotation_recorrection);
            break;

        case Signed_Axes_Enum::X_MINUS:
            temp_random_pose.position.x -= rand_translation;
            temp_recorrected_orientation_rpy[1] -= (1.57 - rotation_recorrection);
            break;

        case Signed_Axes_Enum::Y_PLUS:
            temp_random_pose.position.y += rand_translation;
            temp_recorrected_orientation_rpy[0] -= (1.57 - rotation_recorrection);
            break;

        case Signed_Axes_Enum::Y_MINUS:
            temp_random_pose.position.y -= rand_translation;
            temp_recorrected_orientation_rpy[0] += (1.57 - rotation_recorrection);
            break;
    }
    // finally store the calculated postion and orientation into  random_generated_poses_vector
    temp_recorrected_orientation_quaternion = eulertoQuaternion(
        temp_recorrected_orientation_rpy[0], temp_recorrected_orientation_rpy[1], temp_recorrected_orientation_rpy[2]);
    temp_random_pose.orientation = temp_recorrected_orientation_quaternion;
    random_generated_poses_vector.push_back(temp_random_pose);
}

/**
 * @brief   function to calculate the pose for robot TCP , considers the translation, calculates the
 * recorrected angles in order for robot TCP to constantly look at the Marker
 *
 * @param rand_translation
 * @param start_pose
 * @param start_RPY
 * @param quadrant
 */
void RobotPoseGenerator::translateAndRotateThroughDoubleAxes(double rand_translation, geometry_msgs::Pose start_pose,
                                                             std::vector<double> start_RPY, int quadrant) {
    // translate through given axis but also reccorect the angle so that robot TCP keeps looking at Marker
    // get a temp copy of start pose
    geometry_msgs::Pose temp_random_pose = start_pose;
    // get a temp copy of start rolll pitch yaw angles
    std::vector<double> temp_recorrected_orientation_rpy = start_RPY;
    // get a temp copy of start pose orientation
    geometry_msgs::Quaternion temp_recorrected_orientation_quaternion = start_pose.orientation;
    // this is the angle that needs to be added or substracted to roll, pitch or yaw depending the translation through
    // which axe
    //
    double rotation_recorrection = std::atan2(start_pose.position.z, rand_translation);
    /*
    This function translate and rotates the robot TCP through two axes at one time, each of this four quadrants is an
    combination of 2 axes at the same time.
    For example, in Quadrant ONE , robot translates though X+ and Y+ with amount of rand_translation, also the angles
    gets recorrected same amount

                    ^ +X
    Quadrant ONE    |     Quadrant FOUR
                    |
                    |
                    |
    +Y <--------------------------> -Y
                    |
                    |
    Quadrant TWO    |     Quadrant THREE
                    |
                    |
                      -X
    */

    switch (quadrant) {
        case Quadrant_Enum::ONE:
            temp_random_pose.position.x += rand_translation;
            temp_recorrected_orientation_rpy[1] += (1.57 - rotation_recorrection);
            temp_random_pose.position.y += rand_translation;
            temp_recorrected_orientation_rpy[0] -= (1.57 - rotation_recorrection);
            break;

        case Quadrant_Enum::TWO:
            temp_random_pose.position.x -= rand_translation;
            temp_recorrected_orientation_rpy[1] -= (1.57 - rotation_recorrection);
            temp_random_pose.position.y += rand_translation;
            temp_recorrected_orientation_rpy[0] -= (1.57 - rotation_recorrection);
            break;

        case Quadrant_Enum::THREE:
            temp_random_pose.position.x -= rand_translation;
            temp_recorrected_orientation_rpy[1] -= (1.57 - rotation_recorrection);
            temp_random_pose.position.y -= rand_translation;
            temp_recorrected_orientation_rpy[0] += (1.57 - rotation_recorrection);
            break;

        case Quadrant_Enum::FOUR:
            temp_random_pose.position.x += rand_translation;
            temp_recorrected_orientation_rpy[1] += (1.57 - rotation_recorrection);
            temp_random_pose.position.y -= rand_translation;
            temp_recorrected_orientation_rpy[0] += (1.57 - rotation_recorrection);
            break;
    }

    temp_recorrected_orientation_quaternion = eulertoQuaternion(
        temp_recorrected_orientation_rpy[0], temp_recorrected_orientation_rpy[1], temp_recorrected_orientation_rpy[2]);
    temp_random_pose.orientation = temp_recorrected_orientation_quaternion;
    random_generated_poses_vector.push_back(temp_random_pose);
}