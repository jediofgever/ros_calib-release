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
    random_generated_pose_index_publiher =
        nh_->advertise<visualization_msgs::MarkerArray>("/handeye_calib/random_generated_posse_nums", 1);
    std::cout << "CONSTRUCTED AN INSTANCE OF RobotPoseGenerator" << std::endl;
    listener_ = new tf::TransformListener();
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

    // note that angles are in radians
    std::vector<double> start_RPY = move_group_ptr_->getCurrentRPY();
    for (size_t i = 0; i < number_of_variants; i++) {
        double rand_translation = randint(kLOWERTHRESHOLDCM, kUPPERTHRESHOLDCM) / 100.0;
        translateAndRotateThroughDoubleAxes(rand_translation, RobotPoseGenerator::Quadrant_Enum::ONE);
        translateAndRotateThroughDoubleAxes(rand_translation, RobotPoseGenerator::Quadrant_Enum::TWO);
        translateAndRotateThroughDoubleAxes(rand_translation, RobotPoseGenerator::Quadrant_Enum::THREE);
        translateAndRotateThroughDoubleAxes(rand_translation, RobotPoseGenerator::Quadrant_Enum::FOUR);
    }

    random_generated_poses_array.header.frame_id = "base_link";
    random_generated_poses_array.header.stamp = ros::Time::now();

    for (size_t i = 0; i < random_generated_poses_vector.size(); i++) {
        geometry_msgs::PoseStamped generated_pose_in_base_link;

        listener_->transformPose("base_link", ros::Time(0.0f), random_generated_poses_vector[i], "tool0",
                                 generated_pose_in_base_link);

        random_generated_poses_vector[i] = generated_pose_in_base_link;
        random_generated_poses_array.poses.push_back(generated_pose_in_base_link.pose);
        visualization_msgs::Marker random_generated_pose_index;
        random_generated_pose_index.header.frame_id = "base_link";
        random_generated_pose_index.header.stamp = ros::Time::now();
        random_generated_pose_index.id = i;
        random_generated_pose_index.lifetime = ros::Duration(0.0);
        random_generated_pose_index.action = visualization_msgs::Marker::ADD;
        random_generated_pose_index.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        random_generated_pose_index.pose = generated_pose_in_base_link.pose;
        random_generated_pose_index.text = std::to_string(i);
        random_generated_pose_index.ns = "handeye";
        std_msgs::ColorRGBA color;
        color.g = 1.0;
        color.b = 1.0;
        color.a = 1.0;
        random_generated_pose_index.color = color;
        random_generated_pose_index.scale.z = 0.025;
        random_generated_pose_index_array.markers.push_back(random_generated_pose_index);
    }
    random_generated_pose_publisher.publish(random_generated_poses_array);
    random_generated_pose_index_publiher.publish(random_generated_pose_index_array);

    std::cout << "Generated random poses:" << random_generated_poses_vector.size() << std::endl;
}

/**
 * @brief
 *
 */
void RobotPoseGenerator::updatePoses() {
    random_generated_poses_array.poses.erase(random_generated_poses_array.poses.begin() + 0);
    random_generated_pose_index_array.markers.erase(random_generated_pose_index_array.markers.begin() + 0);
    random_generated_pose_publisher.publish(random_generated_poses_array);
    random_generated_pose_index_publiher.publish(random_generated_pose_index_array);
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
    bool is_success = robot_controller_->moveEndEffectortoGoalinJointSpace(
        random_generated_poses_vector[pose_index].pose, move_group_ptr_);
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

 * @param quadrant
 */
void RobotPoseGenerator::translateAndRotateThroughDoubleAxes(double rand_translation, int quadrant) {
    // translate through given axis but also reccorect the angle so that robot TCP keeps looking at Marker
    // get a temp copy of start pose
    geometry_msgs::PoseStamped temp_random_pose;
    temp_random_pose.header.frame_id = "tool0";
    temp_random_pose.header.stamp = ros::Time::now();

    // get a temp copy of start rolll pitch yaw angles
    std::vector<double> temp_recorrected_orientation_rpy = {0.0, 0.0, 0.0};

    // get a temp copy of start pose orientation
    geometry_msgs::Quaternion temp_recorrected_orientation_quaternion;
    // this is the angle that needs to be added or substracted to roll, pitch or yaw depending the translation through
    // which axe
    //
    double kDistanceinZ;
    ros::param::get("kDistanceinZ", kDistanceinZ);
    double rotation_recorrection = std::atan2(kDistanceinZ, rand_translation);
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
            temp_random_pose.pose.position.x += rand_translation;
            temp_recorrected_orientation_rpy[1] -= 1.5 * (1.57 - rotation_recorrection);
            temp_random_pose.pose.position.y += rand_translation;
            temp_recorrected_orientation_rpy[0] += 1.5 * (1.57 - rotation_recorrection);

            break;

        case Quadrant_Enum::TWO:
            temp_random_pose.pose.position.x -= rand_translation;
            temp_recorrected_orientation_rpy[1] += 1.5 * (1.57 - rotation_recorrection);
            temp_random_pose.pose.position.y += rand_translation;
            temp_recorrected_orientation_rpy[0] += 1.5 * (1.57 - rotation_recorrection);

            break;

        case Quadrant_Enum::THREE:
            temp_random_pose.pose.position.x -= rand_translation;
            temp_recorrected_orientation_rpy[1] += 1.5 * (1.57 - rotation_recorrection);
            temp_random_pose.pose.position.y -= rand_translation;
            temp_recorrected_orientation_rpy[0] -= 1.5 * (1.57 - rotation_recorrection);
            ;

            break;

        case Quadrant_Enum::FOUR:
            temp_random_pose.pose.position.x += rand_translation;
            temp_recorrected_orientation_rpy[1] -= 1.5 * (1.57 - rotation_recorrection);
            temp_random_pose.pose.position.y -= rand_translation;
            temp_recorrected_orientation_rpy[0] -= 1.5 * (1.57 - rotation_recorrection);
            break;
    }
    temp_recorrected_orientation_quaternion = eulertoQuaternion(
        temp_recorrected_orientation_rpy[0], temp_recorrected_orientation_rpy[1], temp_recorrected_orientation_rpy[2]);
    temp_random_pose.pose.orientation = temp_recorrected_orientation_quaternion;
    random_generated_poses_vector.push_back(temp_random_pose);
}

/**
 * @brief
 *
 * @param nh_
 * @param param_name
 * @param pose
 */
geometry_msgs::Pose RobotPoseGenerator::loadPosemsgsFromYAML(ros::NodeHandle *nh_, std::string param_name,
                                                             geometry_msgs::Pose &pose) {
    XmlRpc::XmlRpcValue config;

    if (nh_->hasParam(param_name)) {
        try {
            nh_->getParam(param_name, config);

            ROS_ASSERT(config.getType() == XmlRpc::XmlRpcValue::TypeArray);

            try {
                // These matrices can cause problems if all the types
                // aren't specified with decimal points. Handle that
                // using string streams.
                //  meters of position
                std::ostringstream ostr_x;
                ostr_x << config[0];
                std::istringstream istr_x(ostr_x.str());
                istr_x >> pose.position.x;

                std::ostringstream ostr_y;
                ostr_y << config[1];
                std::istringstream istr_y(ostr_y.str());
                istr_y >> pose.position.y;

                std::ostringstream ostr_z;
                ostr_z << config[2];
                std::istringstream istr_z(ostr_z.str());
                istr_z >> pose.position.z;

                ///////////Euler  angle of rotations

                std::ostringstream ostr_rx;
                ostr_rx << config[3];
                std::istringstream istr_rx(ostr_rx.str());

                std::ostringstream ostr_ry;
                ostr_ry << config[4];
                std::istringstream istr_ry(ostr_ry.str());

                std::ostringstream ostr_rz;
                ostr_rz << config[5];
                std::istringstream istr_rz(ostr_rz.str());
                double rx, ry, rz;
                istr_rx >> rx;
                istr_ry >> ry;
                istr_rz >> rz;

                pose.orientation = eulertoQuaternion(rx, ry, rz);

            } catch (XmlRpc::XmlRpcException &e) {
                throw e;
            } catch (...) {
                throw;
            }

            std::cout << param_name << "\n " << pose << "\n ";
        } catch (XmlRpc::XmlRpcException &e) {
            ROS_ERROR_STREAM("ERROR reading sensor config: " << e.getMessage() << " for " << param_name
                                                             << " (type: " << config.getType() << ")");
        }
    }
    return pose;
}

double RobotPoseGenerator::getDistanceBetweenPose(geometry_msgs::Pose a, geometry_msgs::Pose b) {
    double x_diff = a.position.x - b.position.x;
    double y_diff = a.position.y - b.position.y;
    double z_diff = a.position.z - b.position.z;
    double distance = std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff, 2) + std::pow(z_diff, 2));
    return distance;
}