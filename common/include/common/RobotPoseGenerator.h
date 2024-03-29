
#ifndef robot_pose_generator_H
#define robot_pose_generator_H
#include <XmlRpcException.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <tf/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <functional>
#include <vector>
#include "RobotController.h"

/**
 * @brief generates random number of poses with its method generatePoses()
 *        executes(moves) robot TCP to generated poses with its method  executePose()
 *
 */
class RobotPoseGenerator
{
  private:
    // node handler of ros might needed
    ros::NodeHandle *nh_;
    // drives robot TCP to desired pose
    RobotController *robot_controller_;
    // to plan , execute  given poses
    moveit::planning_interface::MoveGroupInterface *move_group_ptr_;
    // random generated poses vector
    std::vector<geometry_msgs::PoseStamped> random_generated_poses_vector;

    // Max AND MIN translation in CM, through X AND Y axes
    int kLOWERTHRESHOLDCM = 4;
    int kUPPERTHRESHOLDCM = 8;

    enum Signed_Axes_Enum
    {
        X_PLUS  = 0,
        X_MINUS = 1,
        Y_PLUS  = 2,
        Y_MINUS = 3
    };
    enum Quadrant_Enum
    {
        ONE   = 0,
        TWO   = 1,
        THREE = 2,
        FOUR  = 3
    };

    ros::Publisher random_generated_pose_publisher;
    ros::Publisher random_generated_pose_index_publiher;

    geometry_msgs::PoseArray random_generated_poses_array;
    visualization_msgs::MarkerArray random_generated_pose_index_array;

    tf::TransformListener *listener_;

  public:
    /**
     * @brief Construct a new Robot Pose Generator object
     *
     */
    RobotPoseGenerator();

    /**
     * @brief Destroy the Robot Pose Generator object
     *
     */
    ~RobotPoseGenerator();

    /**
     * @brief given number variations, generates variations * 6 random poses
     *
     * @param number_of_variants
     */
    void generatePoses(int number_of_variants);

    /**
     * @brief given euler angles in radian, returns geometry_msgs::Quaternion ROS type
     * @param robot_rx_radian
     * @param robot_ry_radian
     * @param robot_rz_radian
     * @return geometry_msgs::Quaternion
     */
    static geometry_msgs::Quaternion eulertoQuaternion(double robot_rx_radian,
                                                       double robot_ry_radian,
                                                       double robot_rz_radian);

    /**
     * @brief executes the randomly generated pose in random_generated_poses_vector with index of
     * pose_index
     *
     * @param pose_index
     * @return true if pose at pose_index was reachable
     * @return false if pose at pose_index was reachable
     */
    bool executePose(int pose_index);

    /**
     * @brief
     *
     * @param
     */
    void updatePoses();

    /**
     * @brief   function to calculate the pose for robot TCP , considers the translation, calculates
     * the recorrected angles in order for robot TCP to constantly look at the Marker
     *
     * @param rand_translation
     * @param start_pose
     * @param start_RPY
     * @param quadrant
     */
    void translateAndRotateThroughDoubleAxes(double rand_translation, int quadrant);

    /**
     * @brief   function to calculate the pose for robot TCP , considers the translation, calculates
     * the recorrected angles in order for robot TCP to constantly look at the Marker
     *
     * @param rand_translation
     * @param start_pose
     * @param start_RPY
     * @param quadrant
     */
    void translateAndRotateThroughSingleAxe(double rand_translation, int quadrant);

    /**
     * @brief RETRUN A RANDOM INTEGER BETWEEN GIVEN BOUNDRIES
     *
     * @param Min
     * @param Max
     * @return int
     */
    int randint(int Min, int Max);

    /**
     * @brief
     *
     * @param nh_
     * @param param_name
     * @param pose
     */
    static geometry_msgs::Pose loadPosemsgsFromYAML(ros::NodeHandle *nh_,
                                                    std::string param_name,
                                                    geometry_msgs::Pose &pose);

    /**
     * @brief Get the Distance Between Pose object
     *
     * @param a
     * @param b
     * @return double
     */
    static double getDistanceBetweenPose(geometry_msgs::Pose a, geometry_msgs::Pose b);
};
#endif
