#include <common/RobotController.h>
#include <common/RobotPoseGenerator.h>
#include <handeye_calib/HandeyeCalibration.h>
#include <QApplication>
#include <QDir>
#include <QMutex>
#include <QPixmap>
#include <QWaitCondition>
#include <QtWidgets/QMessageBox>
#include <condition_variable>  // std::condition_variable
#include <mutex>

QMutex mutex(QMutex::NonRecursive);
QWaitCondition waitCondition;

// latest recieved transfrom message(marker to camera transform)
geometry_msgs::TransformStamped latest_marker_to_camera_transform;
// protects latest_marker_to_camera_transform
std::mutex transfrom_mutex;

bool is_marker_pose_recieved = false;
// latest recieved transfrom message(marker to camera transform)
geometry_msgs::PoseStamped latest_marker_pose_in_camera_link;
// protects latest_marker_to_camera_transform
std::mutex pose_mutex;

/**
 * @brief callback function that recieves latest transfrom of marker to camera , and stores it in
 * latest_marker_to_camera_transform
 *
 * @param msg
 */
void marker2CamTransCallback(const geometry_msgs::TransformStampedConstPtr& msg) {
    // deny access to latest_marker_to_camera_transform while assigning
    const std::lock_guard<std::mutex> lock(transfrom_mutex);
    latest_marker_to_camera_transform = *msg;
    // the lock will be released after outta scope
}

/**
 * @brief callback function that recieves latest transfrom of marker to camera , and stores it in
 * latest_marker_to_camera_transform
 *
 * @param msg
 */
void markerPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    // deny access to latest_marker_to_camera_transform while assigning
    const std::lock_guard<std::mutex> lock(pose_mutex);
    latest_marker_pose_in_camera_link = *msg;
    // the lock will be released after outta scope
    is_marker_pose_recieved = true;
}

/**
 * @brief accepts a stamped transform, compares the accepted transfrom stamp with the current time, if the transform
 * was recieved more than 1 seconds ago, return false, if the transform is fresher than 1 second , returns true,
 * which makes this transform valid
 *
 * @param transform_msg
 * @return true
 * @return false
 */
bool isTransformValid(geometry_msgs::TransformStamped transform_msg) {
    ros::Time latest_transfrom_stamp = transform_msg.header.stamp;
    ros::Time time_now = ros::Time::now();
    ros::Duration elapsed_time_from_latest_transfrom = time_now - latest_transfrom_stamp;
    double elapsed_time_from_latest_transfrom_double = std::abs(elapsed_time_from_latest_transfrom.toSec());
    if (elapsed_time_from_latest_transfrom_double < 1.0) {
        return true;
    } else {
        return false;
    }
}

bool marker_pose_available() { return is_marker_pose_recieved; }

/**
 * @brief Constructs a ROS node for  HandeyeCalibration
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char** argv) {
    // init a new node with the name of calibrator_node
    ros::init(argc, argv, "calibrator_node");
    QApplication app(argc, argv);

    // ros node handler to for pubs and subs
    ros::NodeHandle node_handle;

    // define a loop rate
    int node_loop_rate = 30;
    ros::Rate loop_rate(node_loop_rate);
    // Multithreaded ROS spinner
    ros::AsyncSpinner spinner(4);
    // start the spinner
    spinner.start();
    // subscribe to "/aruco_tracker/transform" topic to get latest known transfrom from marker to camera
    ros::Subscriber transfrom_marker_to_camera_sub =
        node_handle.subscribe("/aruco_tracker/transform", 1, marker2CamTransCallback);

    ros::Subscriber marker_pose_in_camera_link_sub =
        node_handle.subscribe("/aruco_tracker/pose", 1, markerPoseCallback);
    // create an instance HandeyeCalibration, to collect samples and coimpute calibration between camera to robot
    // end-effector
    HandeyeCalibration hand_eye_node(&node_handle);
    // for generating random poses and executing them
    RobotPoseGenerator pose_generator;
    RobotController robot_contoller;
    tf::TransformListener listener;
    moveit::planning_interface::MoveGroupInterface* move_group_ptr_;
    static const std::string PLANNING_GROUP = "manipulator";
    move_group_ptr_ = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

    while (is_marker_pose_recieved == false) {
        ROS_INFO("waiting for aruco marker pose to come up .....\n");
        QMutexLocker locker(&mutex);
        waitCondition.wait(&mutex, 100);
    }
    ROS_INFO("aruco marker pose is up  approaching it..... \n");
    double kDistanceinZ = 0.30;
    // move robot on top of marker
    geometry_msgs::Pose marker_in_tool, distance_to_travel_in_tool;
    marker_in_tool.position.x = latest_marker_pose_in_camera_link.pose.position.z;
    marker_in_tool.position.y = latest_marker_pose_in_camera_link.pose.position.y;
    marker_in_tool.position.z = latest_marker_pose_in_camera_link.pose.position.x;

    distance_to_travel_in_tool.position.x = -marker_in_tool.position.x;
    distance_to_travel_in_tool.position.y = -marker_in_tool.position.y;
    distance_to_travel_in_tool.position.z = marker_in_tool.position.z - kDistanceinZ;

    robot_contoller.moveEndEffectortoGoalinToolSpace(distance_to_travel_in_tool, move_group_ptr_, &listener);

    QMessageBox calib_start_box;
    calib_start_box.setText(
        "YOU HAVE STARTED AUTO HANDEYE CALIBRATION NODE \n \n"
        "A SET OF RANDOM POSES WILL BE GENERATED AND AT EACH POSE AN ITERATION OF CALIBRATION WILL TAKE PLACE \n \n"
        "IF THE ALGORITHM IS UNABLE TO DETECT ARUCO MARKER THE USER WILL BE INFORMED \n \n"
        "THE SAMPLE IN THIS POSE WILL BE SKIPPED , AND IT WILL NOT BE INCLUDED IN CALIBRATION \n \n"
        "MAKE SURE YOU READ AND UNDERSTAND THIS, IF SO CLICK OK TO START THE PROCESS !!");
    calib_start_box.exec();

    // Each Pose variants will contin 10 poses , so the total generated poses will be num_random_pose_variants * 10
    int num_random_pose_variants;
    node_handle.getParam("num_random_pose_variants", num_random_pose_variants);
    pose_generator.generatePoses(num_random_pose_variants);
    // start from 0 and increment until all generated poses are visited
    int executed_poses_counter = 0;

    // ENTER the looping, make sure our god, our dear ROS is ok and all poses are not executed
    while (ros::ok() && (executed_poses_counter < num_random_pose_variants * 10)) {
        // if pose is reachable execute it, else raise the error and go to next pose;
        bool is_generated_pose_planable = pose_generator.executePose(executed_poses_counter);
        if (is_generated_pose_planable) {
            // Okay this pose is reachable and we have moved the robot to these pose
            ROS_INFO("ROBOT RECAHED TO THE GENERATED POSE, WAITING 3 SECONDS TO LET MARKER STABILIZE ... \n");
            // now sleep for 3 second
            sleep(3.0);

            //  check if marker is visiable and detected, (look at the definition of isTransformValid)
            if (isTransformValid(latest_marker_to_camera_transform)) {
                // here everything is fine, so take that goddamn sample and compute calibration
                hand_eye_node.takeSample();
                hand_eye_node.computeCalibration();
                // increment the counter of executed poses
                executed_poses_counter++;

            } else {
                QMessageBox Msgbox;
                Msgbox.setText(
                    "THE MARKER TRANSFROM IS OLDER THAN 1 SECOND , THIS LIKELY MEANS THAT THE MARKER WAS NOT "
                    "DETCETED, "
                    "SKIPPING THIS SAMPLE!!! \n");
                Msgbox.exec();

                ROS_ERROR(
                    "THE MARKER TRANSFROM IS OLDER THAN 1 SECOND , THIS LIKELY MEANS THAT THE MARKER WAS NOT "
                    "DETCETED, "
                    "SKIPPING THIS SAMPLE!!! \n");
                executed_poses_counter++;
                continue;
            }
        } else {
            ROS_ERROR(
                "RANDOMLY GENERATED POSE %d FOR CALIBRATION WAS UNREACHABLE/UNPLANABLE , WE ARE GOING TO SKIP THIS "
                "POSE!!! \n",
                executed_poses_counter);
            executed_poses_counter++;
            continue;
        }

        // spin the loop
        ros::spinOnce();
        // sleep to match with loop rate
        loop_rate.sleep();
    }
    // at this point node is dead

    QMessageBox finish_msg;
    finish_msg.setText("CALIBRATION DONE , LOOK FOR THE FILE UNDER YOUR HOME DIR");
    finish_msg.exec();
    ros::waitForShutdown();
    ros::shutdown();
    ROS_WARN("calibrator node SHUT DOWN BYE.");
    return 0;
}
