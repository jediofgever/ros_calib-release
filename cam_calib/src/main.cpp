#include <cam_calib/CamCalibration.h>
#include <common/RobotPoseGenerator.h>
#include <QApplication>
#include <QPixmap>
#include <QtWidgets/QMessageBox>
#include <mutex>

/**
 * @brief Constructs a ROS node for  Cam Calibration
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char** argv) {
    // init a new node with the name of calibrator_node
    ros::init(argc, argv, "calibrator_node");
    QApplication app(argc, argv);

    ros::NodeHandle nh;

    // define a loop rate
    int node_loop_rate = 30;
    ros::Rate loop_rate(node_loop_rate);
    // Multithreaded ROS spinner
    ros::AsyncSpinner spinner(8);
    // start the spinner
    spinner.start();

    // for generating random poses and executing them
    RobotPoseGenerator pose_generator;

    CamCalibration proc(&nh);
    ROS_INFO("Camera Calibration Has been Initilizaed..");

    // Each Pose variants will contin 10 poses , so the total generated poses will be num_pose_variants * 10
    int num_pose_variants = 5;
    pose_generator.generatePoses(5);
    // start from 0 and increment until all generated poses are visited
    int executed_poses = 0;

    // ENTER the looping, make sure our god, our dear ROS is ok and all poses are not executed
    while (ros::ok() && (executed_poses < num_pose_variants * 10)) {
        // if pose is reachable execute it, else raise the error and go to next pose;
        bool is_generated_pose_planable = pose_generator.executePose(executed_poses);
        if (is_generated_pose_planable) {
            // Okay this pose is reachable and we have moved the robot to these pose
            ROS_INFO("ROBOT RECAHED TO THE GENERATED POSE,TAKING A SHOT NOW ... \n");
            // here everything is fine, so take that goddamn sample , let the user select the keypoints and compute
            // calibration
            proc.processCamCalib();
            // increment the counter of executed poses
            executed_poses++;
        } else {
            // if the generated pose is not reachable , raise a messagebox from qt to inform user
            QMessageBox Msgbox;
            Msgbox.setText("COULD NOT REACH THIS RANDOM POSE JUMPING TO THE NEXT ONE !!! \n");
            Msgbox.exec();
            // raise a ros error
            ROS_ERROR("COULD NOT REACH THIS RANDOM POSE JUMPING TO THE NEXT ONE !!!  \n");
            // count these bad pose as executed and move to next one
            executed_poses++;
            continue;
        }
        // spin the loop
        ros::spinOnce();
        // sleep to match with loop rate
        loop_rate.sleep();
    }

    // at this point node is dead
    ros::waitForShutdown();
    ros::shutdown();
    ROS_WARN("calibrator node SHUT DOWN BYE.");
    return 0;
}
