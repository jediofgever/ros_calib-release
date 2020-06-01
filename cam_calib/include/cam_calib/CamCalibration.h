#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpImage.h>
#include <visp/vpPoint.h>
#include <boost/thread/thread.hpp>
#include <string>
#include <vector>

#include <pwd.h>
#include <sys/types.h>
#include <unistd.h>

#include <visp_camera_calibration/calibrateRequest.h>
#include <visp_camera_calibration/calibrateResponse.h>
#include <mutex>

/**
 * @brief Handeye calibration class based on; vison_visp
 * https://github.com/lagadic/vision_visp
 *
 */
class CamCalibration {
   private:
    // ROS types
    ros::NodeHandle *nh_;
    ros::Subscriber camera_raw_subscriber_;
    ros::Publisher point_correspondence_publisher_;
    ros::ServiceServer set_camera_info_bis_service_;
    ros::ServiceClient calibrate_service_;

    //
    int queue_size_;
    bool pause_image_;
    bool is_initialized_;

    // ROS topic name strings
    std::string camera_image_topic_name_;
    std::string set_camera_info_service_topic_name_;

    // VISP types
    vpImage<unsigned char> img_;
    std::vector<vpPoint> selected_points_;
    std::vector<vpPoint> model_points_;
    vpCameraParameters cam_param_;

    // mutex for camera image
    std::mutex image_mutex;

   public:
    //! service type declaration for calibrate service
    typedef boost::function<bool(sensor_msgs::SetCameraInfo::Request &, sensor_msgs::SetCameraInfo::Response &res)>
        set_camera_info_bis_service_callback_t;

    /**
     * @brief Construct a new Cam Calibration object
     *
     * @param nh
     */
    CamCalibration(ros::NodeHandle *nh);

    /**
     * @brief Destroy the Cam Calibration object
     *
     */
    virtual ~CamCalibration();

    /**
     * @brief callback corresponding to the raw_image topic.
      Computes a cMo from selected points on the image.
      Projects all points of the model to match the image of the grid.
      Add the obtained calibration object to an internal calibration list.
     *
     */
    void init();

    /**
     * @brief  camera Image callback, stroes the latest recieved image in visp type global variable "img_"
     *
     * @param image
     */
    void rawImageCallback(const sensor_msgs::Image::ConstPtr &image);

    /**
     * @brief Set the Camera Info Bis Callback object, writes calibration result to disk
     *
     * @param req
     * @param res
     * @return true
     * @return false
     */
    bool setCameraInfoBisCallback(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res);

    /**
     * @brief does actual collection of keypoints selected by user, publishes corresponding point for calibrator
     *        calibrate service is called through this function
     */
    void processCamCalib();
};
