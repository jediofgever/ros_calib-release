#include <handeye_calib/HandeyeCalibration.h>

/**
 * @brief Construct a new Handeye Calibration:: Handeye Calibration object
 *
 * @param nh
 */
HandeyeCalibration::HandeyeCalibration(ros::NodeHandle *nh) {
    // init pointers
    listener_ = new tf::TransformListener();
    nh_ = nh;

    ros::param::get("robot_base_frame", robot_base_frame);
    ros::param::get("robot_effector_frame", robot_effector_frame);
    ros::param::get("tracking_base_frame", tracking_base_frame);
    ros::param::get("tracking_marker_frame", tracking_marker_frame);
    ros::param::get("eye_in_hand", eye_in_hand);

    // print  passed args to constructer for debug purposes
    std::cout << "AN INSTANCE OF HANDEYECALIBRATION OBJECT HASD BEEN CREATED" << std::endl;
    std::cout << "robot_base_frame: " << robot_base_frame << std::endl;
    std::cout << "robot_effector_frame: " << robot_effector_frame << std::endl;
    std::cout << "tracking_base_frame: " << tracking_base_frame << std::endl;
    std::cout << "tracking_marker_frame: " << tracking_marker_frame << std::endl;
    std::cout << "eye_in_hand: " << eye_in_hand << std::endl;
}
/**
 * @brief Destroy the Handeye Calibration object
 *
 */
HandeyeCalibration::~HandeyeCalibration() {
    delete listener_;
    delete nh_;
    std::cout << "KILLING HANDEYECALIBRATION INSTANCE";
};

/**
 * @brief Get the Transfrom Pair object
 *
 * @return a pair geometry_msgs::TransformStamped std::pair<geometry_msgs::TransformStamped,
 * geometry_msgs::TransformStamped>
 */
std::pair<geometry_msgs::TransformStamped, geometry_msgs::TransformStamped> HandeyeCalibration::getTransfromPair() {
    geometry_msgs::TransformStamped rob_msg, opt_msg;

    // LISTEN TO TRANSFORM AND GET MATRIXES OF TRANSFORM
    tf::StampedTransform rob_transform, opt_transform;
    // lookup transform (this should be cached, since it’s probably static)
    try {
        if (eye_in_hand) {
            listener_->lookupTransform(robot_base_frame, robot_effector_frame, ros::Time(0.0f), rob_transform);
        } else {
            listener_->lookupTransform(robot_effector_frame, robot_base_frame, ros::Time(0.0f), rob_transform);
        }
        listener_->lookupTransform(tracking_base_frame, tracking_marker_frame, ros::Time(0.0f), opt_transform);
        tf::transformStampedTFToMsg(rob_transform, rob_msg);
        tf::transformStampedTFToMsg(opt_transform, opt_msg);

    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    // return catched  the pair of transforms (hand_world , camera_marker)
    std::pair<geometry_msgs::TransformStamped, geometry_msgs::TransformStamped> this_pair(rob_msg, opt_msg);
    return this_pair;
}

/**
 * @brief take a sample pair of geometry_msgs::TransformStamped and store it in transform_pair_samples_vector
 *
 */
void HandeyeCalibration::takeSample() {
    std::cout << "TAKING A SAMPLE";
    std::pair<geometry_msgs::TransformStamped, geometry_msgs::TransformStamped> a_sample_pair = getTransfromPair();
    transform_pair_samples_vector.push_back(a_sample_pair);
    // std::cout << "THE SAMPLE IS; " << std::endl;
    // std::cout << a_sample_pair.first << a_sample_pair.second << std::endl;
}

/**
 * @brief converts transform_pair_samples_vector to a pair of visp_hand2eye_calibration::TransformArray type
 *
 * @return std::pair<visp_hand2eye_calibration::TransformArray, visp_hand2eye_calibration::TransformArray>
 */
std::pair<visp_hand2eye_calibration::TransformArray, visp_hand2eye_calibration::TransformArray> HandeyeCalibration::samples2Visp() {
    // we need to convert std::vector<std::pair<geometry_msgs::TransformStamped, geometry_msgs::TransformStamped>> TO
    // std::pair<visp_hand2eye_calibration::TransformArray, visp_hand2eye_calibration::TransformArray>
    visp_hand2eye_calibration::TransformArray hand_world_samples;
    hand_world_samples.header.frame_id = tracking_base_frame;
    visp_hand2eye_calibration::TransformArray camera_marker_samples;
    camera_marker_samples.header.frame_id = tracking_base_frame;

    // push all transforms individually
    for (size_t i = 0; i < transform_pair_samples_vector.size(); i++) {
        hand_world_samples.transforms.push_back(transform_pair_samples_vector[i].first.transform);
        camera_marker_samples.transforms.push_back(transform_pair_samples_vector[i].second.transform);
    }
    std::pair<visp_hand2eye_calibration::TransformArray, visp_hand2eye_calibration::TransformArray> visp_samples(hand_world_samples, camera_marker_samples);

    // return the pair of TransformArray
    return visp_samples;
}
/**
 * @brief given a pair of visp_hand2eye_calibration::TransformArray computes calibration
 *
 */
void HandeyeCalibration::computeCalibration() {
    // Less than MIN_SAMPLES we do not process computation
    if (transform_pair_samples_vector.size() < MIN_SAMPLES) {
        ROS_ERROR("LESS THAN MIN_SAMPLES, SAMPLES NOT ENOUGH YET NOT GONNA COMPUTE A");
        return;
    }

    // convert collected samples transfrom pairs to visp using utility function  samples2Visp()
    std::pair<visp_hand2eye_calibration::TransformArray, visp_hand2eye_calibration::TransformArray> visp_samples = samples2Visp();

    // hand_world camera_marker transfroms samples should be equal
    if (visp_samples.first.transforms.size() != visp_samples.second.transforms.size()) {
        ROS_ERROR("Different numbers of hand-world and camera-marker samples!");
        return;
    }

    // the  calibrate from vpHandEyeCalibration uses a vpHomogeneousMatrix , we use visp_bridge to convert
    // TransformArray to a vector of this vpHomogeneousMatrix

    // camera_marker transfroms
    std::vector<vpHomogeneousMatrix> cMo_vec;
    // hand_world transforms
    std::vector<vpHomogeneousMatrix> wMe_vec;
    // result
    vpHomogeneousMatrix eMc;
    for (unsigned int i = 0; i < visp_samples.first.transforms.size(); i++) {
        wMe_vec.push_back(visp_bridge::toVispHomogeneousMatrix(visp_samples.first.transforms[i]));
        cMo_vec.push_back(visp_bridge::toVispHomogeneousMatrix(visp_samples.second.transforms[i]));
    }
    // try to calibrate with so far samples
    try {
        vpHandEyeCalibration::calibrate(cMo_vec, wMe_vec, eMc);
        vpPoseVector pose_vec(eMc);
        double kDistanceColor2Depth = 0.015 / 2.0;

        geometry_msgs::Transform trans = visp_bridge::toGeometryMsgsTransform(eMc);
        vpArray2D<double> trans_as_1D_array;
        trans_as_1D_array.resize(1, 7);
        trans_as_1D_array[0][0] = trans.translation.x;
        trans_as_1D_array[0][1] = trans.translation.y;
        trans_as_1D_array[0][2] = trans.translation.z;
        trans_as_1D_array[0][3] = trans.rotation.x;
        trans_as_1D_array[0][4] = trans.rotation.y;
        trans_as_1D_array[0][5] = trans.rotation.z;
        trans_as_1D_array[0][6] = trans.rotation.w;

        struct passwd *pw = getpwuid(getuid());
        const char *homedir = pw->pw_dir;
        std::string home_dir_str(homedir);
        std::string calibration_path;
        nh_->getParam("calibration_path", calibration_path);
        trans_as_1D_array.saveYAML(home_dir_str + "/" + calibration_path, trans_as_1D_array);
        ROS_INFO_STREAM("RESULTING EXTRINSIC CALIB RESULT IS: " << std::endl << trans);

    } catch (const std::exception &e) {
        // catch the error
        std::cerr << e.what() << '\n';
    }
}