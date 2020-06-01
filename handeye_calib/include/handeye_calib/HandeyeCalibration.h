#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <yaml.h>

///
#include <visp/vpHandEyeCalibration.h>
#include <visp_bridge/3dpose.h>
#include <visp_hand2eye_calibration/TransformArray.h>
#include <visp_hand2eye_calibration/compute_effector_camera_quick.h>

//
#include <pwd.h>
#include <sys/types.h>
#include <unistd.h>
// std::pair, std::make_pair

#include <utility>

/**
 * @brief Handeye calibration class based on; vison_visp
 * https://github.com/lagadic/vision_visp
 *
 */
class HandeyeCalibration {
   private:
    std::string DIRECTORY;
    std::string calibration_file_name;

    // base link of the robot, in ros this will translate to "base_link"
    std::string robot_base_frame;
    // end_effector frame of robot  uusally "tool0" or "link_6" for 6DOF robots
    std::string robot_effector_frame;
    // frame that aruco marker is connected to , usually "camera_link"
    std::string tracking_base_frame;
    // aruco marker frame, usually "camera_marker"
    std::string tracking_marker_frame;

    // vision_visp service to
    ros::ServiceClient compute_effector_camera_quick_service_;
    // minimum transform pair samples for a valid calibration computation
    int MIN_SAMPLES = 4;
    // listens to tf tree and get trasnforms between given links
    tf::TransformListener *listener_;
    // handles ros stuff if any
    ros::NodeHandle *nh_;

    // stores collected transform pairs
    std::vector<std::pair<geometry_msgs::TransformStamped, geometry_msgs::TransformStamped>>
        transform_pair_samples_vector;

    bool eye_in_hand;

   public:
    /**
     * @brief Construct a new Handeye Calibration object
     *
     * @param nh
     */
    HandeyeCalibration(ros::NodeHandle *nh);

    /**
     * @brief Destroy the Handeye Calibration object
     *
     */
    ~HandeyeCalibration();

    /**
     * @brief Get the Transfrom Pair object
     *
     * @return a pair geometry_msgs::TransformStamped std::pair<geometry_msgs::TransformStamped,
     * geometry_msgs::TransformStamped>
     */
    std::pair<geometry_msgs::TransformStamped, geometry_msgs::TransformStamped> getTransfromPair();

    /**
     * @brief take a sample pair of geometry_msgs::TransformStamped and store it in transform_pair_samples_vector
     *
     */
    void takeSample();

    /**
     * @brief converts transform_pair_samples_vector to a pair of visp_hand2eye_calibration::TransformArray type
     *
     * @return std::pair<visp_hand2eye_calibration::TransformArray, visp_hand2eye_calibration::TransformArray>
     */
    std::pair<visp_hand2eye_calibration::TransformArray, visp_hand2eye_calibration::TransformArray> samples2Visp();

    /**
     * @brief given a pair of visp_hand2eye_calibration::TransformArray computes calibration
     *
     */
    void computeCalibration();
};
