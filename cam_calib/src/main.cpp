#include <cam_calib/CamCalibration.h>
#include <common/RobotPoseGenerator.h>
#include <QApplication>
#include <QMutex>
#include <QMutexLocker>
#include <QWaitCondition>

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
int main(int argc, char** argv)
{
  // init a new node with the name of calibrator_node
  ros::init(argc, argv, "calibrator_node");
  QApplication app(argc, argv);

  // ros node handler to for pubs and subs
  ros::NodeHandle node_handle;
  // define a loop rate
  int node_loop_rate = 30;
  ros::Rate loop_rate(node_loop_rate);
  // Multithreaded ROS spinner
  ros::AsyncSpinner spinner(8);
  // start the spinner
  spinner.start();

  // for generating random poses and executing them
  RobotPoseGenerator* pose_generator_ptr_;
  pose_generator_ptr_ = new RobotPoseGenerator();
  RobotController* robot_contoller_ptr_ = new RobotController();
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface* move_group_ptr_ =
      new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

  CamCalibration cam_calib(&node_handle);
  ROS_INFO("Camera Calibration Has been Initilizaed..");
  ROS_INFO("Waiting robot states to be available..");
  sleep(5.0);
  // move robot to a place where we clearly see marker
  geometry_msgs::Pose initial_pose;
  RobotPoseGenerator::loadPosemsgsFromYAML(&node_handle, "real_robot_initial_pose", initial_pose);
  double current_distance_to_initial_pose =
      RobotPoseGenerator::getDistanceBetweenPose(initial_pose, move_group_ptr_->getCurrentPose().pose);

  robot_contoller_ptr_->moveEndEffectortoGoalinJointSpace(initial_pose, move_group_ptr_);

  QMessageBox Msgbox;
  Msgbox.setText("YOU HAVE STARTED CAMERA INSTRISC CALIBRATION NODE \n \n"
                 "A SET OF RANDOM POSES WILL BE GENERATED AND AT EACH POSE AN IMAGE WILL SHOWN \n \n"
                 "CLICK THE KEY POINTS 1 2 3 4 RESPECTIVELY ON SHOWN IMAGE \n \n"
                 "AFTER SLEECTING ALL 4 KEYPOINTS THE ALGORITHM WILL DETECT ALL REMAINING POINTS \n \n"
                 "AND ONE ITERATION OF CALIBRATION WILL BE EXECUTED \n \n"
                 "IF THE ALGORITHM IS UNABLE TO DETECT ALL THE POINTS THAT POSE WILL BE SKIPPED AND NOT INCLUDED IN \n "
                 "\n"
                 "MAKE SURE YOU READ AND UNDERSTAND THIS, IF SO CLICK OK TO START THE PROCESS !!");
  Msgbox.exec();

  // Each Pose variants will contin 10 poses , so the total generated poses will be num_pose_variants * 10
  int num_random_pose_variants;
  node_handle.getParam("num_random_pose_variants", num_random_pose_variants);
  pose_generator_ptr_->generatePoses(num_random_pose_variants);
  // start from 0 and increment until all generated poses are visited
  int executed_poses_counter = 0;

  // ENTER the looping, make sure our god, our dear ROS is ok and all poses are not executed
  while (ros::ok() && (executed_poses_counter < num_random_pose_variants * 8))
  {
    // if pose is reachable execute it, else raise the error and go to next pose;
    bool is_generated_pose_planable = pose_generator_ptr_->executePose(executed_poses_counter);
    if (is_generated_pose_planable)
    {
      // Okay this pose is reachable and we have moved the robot to these pose
      ROS_INFO("ROBOT RECAHED TO THE GENERATED POSE,TAKING A SHOT NOW ... \n");
      // here everything is fine, so take that goddamn sample , let the user select the keypoints and compute
      // calibration
      cam_calib.processCamCalib(executed_poses_counter+1 , num_random_pose_variants * 8);
      // increment the counter of executed poses
      executed_poses_counter++;
    }
    else
    {
      // if the generated pose is not reachable , raise a messagebox from qt to inform user
      QMessageBox Msgbox;
      Msgbox.setText("COULD NOT REACH THIS RANDOM POSE JUMPING TO THE NEXT ONE !!! \n");
      Msgbox.exec();
      // raise a ros error
      ROS_ERROR("COULD NOT REACH THIS RANDOM POSE JUMPING TO THE NEXT ONE !!!  \n");
      // count these bad pose as executed and move to next one
      executed_poses_counter++;
      continue;
    }
    pose_generator_ptr_->updatePoses();

    // spin the loop
    ros::spinOnce();
    // sleep to match with loop rate
    loop_rate.sleep();
  }

  QMessageBox finish_msg;
  finish_msg.setText("CALIBRATION DONE , LOOK FOR THE FILE UNDER YOUR HOME DIR");
  finish_msg.exec();
  // at this point node is dead
  ros::waitForShutdown();
  ros::shutdown();
  ROS_WARN("calibrator node SHUT DOWN BYE.");
  return 0;
}
