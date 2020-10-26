#include <cam_calib/CamCalibration.h>
#include "visp/vpMouseButton.h"
#include "visp/vpTrackingException.h"
#include "visp_camera_calibration/CalibPoint.h"
#include "visp_camera_calibration/CalibPointArray.h"
#include "visp_camera_calibration/calibrate.h"

#include <visp_bridge/image.h>
#include "camera_calibration_parsers/parse.h"
#include "sensor_msgs/SetCameraInfo.h"

#include <boost/format.hpp>
#include <iostream>
#include "visp/vpCalibration.h"
#include "visp/vpDisplayX.h"
#include "visp/vpDot.h"
#include "visp/vpDot2.h"
#include "visp/vpHomogeneousMatrix.h"
#include "visp/vpImagePoint.h"
#include "visp/vpMeterPixelConversion.h"
#include "visp/vpPixelMeterConversion.h"
#include "visp/vpPose.h"

#include <ros/service_traits.h>

CamCalibration::CamCalibration(ros::NodeHandle *nh)
    : queue_size_(10),
      pause_image_(false),
      img_(960, 540, 128),
      cam_param_(600, 600, 0, 0),
      is_initialized_(false),
      nh_(nh)
{
    // Get parameters from config.yaml file
    std::string model_points_x_param("model_points_x");
    std::string model_points_y_param("model_points_y");
    std::string model_points_z_param("model_points_z");
    std::string selected_points_x_param("selected_points_x");
    std::string selected_points_y_param("selected_points_y");
    std::string selected_points_z_param("selected_points_z");
    ros::param::get("camera_image_topic_name", camera_image_topic_name_);
    ros::param::get("set_camera_info_service_topic_name", set_camera_info_service_topic_name_);

    // subscribe to raw image
    camera_raw_subscriber_ = nh_->subscribe(camera_image_topic_name_, queue_size_,
                                            &CamCalibration::rawImageCallback, this);
    // connect to calibrate service
    calibrate_service_ = nh_->serviceClient<visp_camera_calibration::calibrate>("/calibrate");
    // point correspondence publisher for camera calibrator
    point_correspondence_publisher_ = nh_->advertise<visp_camera_calibration::CalibPointArray>(
        "point_correspondence", queue_size_);
    // Parse the paraneters read from yaml file, append them to lists

    set_camera_info_bis_service_callback_t set_camera_info_bis_callback =
        boost::bind(&CamCalibration::setCameraInfoBisCallback, this, _1, _2);
    set_camera_info_bis_service_ =
        nh_->advertiseService("set_camera_info_bis", set_camera_info_bis_callback);

    XmlRpc::XmlRpcValue model_points_x_list;
    XmlRpc::XmlRpcValue model_points_y_list;
    XmlRpc::XmlRpcValue model_points_z_list;
    ros::param::get(model_points_x_param.c_str(), model_points_x_list);
    ros::param::get(model_points_y_param, model_points_y_list);
    ros::param::get(model_points_z_param, model_points_z_list);

    ROS_ASSERT(model_points_x_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(model_points_x_list.size() == model_points_y_list.size() &&
               model_points_x_list.size() == model_points_z_list.size());
    for (int i = 0; i < model_points_x_list.size(); i++)
    {
        ROS_ASSERT(model_points_x_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(model_points_y_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(model_points_z_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        double X = static_cast<double>(model_points_x_list[i]);
        double Y = static_cast<double>(model_points_y_list[i]);
        double Z = static_cast<double>(model_points_z_list[i]);
        vpPoint p;
        p.setWorldCoordinates(X, Y, Z);
        model_points_.push_back(p);
    }

    // define selected model points
    XmlRpc::XmlRpcValue selected_points_x_list;
    XmlRpc::XmlRpcValue selected_points_y_list;
    XmlRpc::XmlRpcValue selected_points_z_list;

    ros::param::get(selected_points_x_param, selected_points_x_list);
    ros::param::get(selected_points_y_param, selected_points_y_list);
    ros::param::get(selected_points_z_param, selected_points_z_list);
    ROS_ASSERT(selected_points_x_list.size() == selected_points_y_list.size() &&
               selected_points_x_list.size() == selected_points_z_list.size());

    for (int i = 0; i < selected_points_x_list.size(); i++)
    {
        ROS_ASSERT(selected_points_x_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(selected_points_y_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(selected_points_z_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        double X = static_cast<double>(selected_points_x_list[i]);
        double Y = static_cast<double>(selected_points_y_list[i]);
        double Z = static_cast<double>(selected_points_z_list[i]);
        vpPoint p;
        p.setWorldCoordinates(X, Y, Z);
        selected_points_.push_back(p);
    }
    ROS_INFO("CREATED CAMCALIBRATION INSTANCE ... \n");
}

void CamCalibration::init()
{
    if (!is_initialized_)
    {
        // init graphical interface
        vpDisplay *disp = new vpDisplayX();
        disp->init(img_);
        disp->setTitle("Image processing initialisation interface");
        vpDisplay::flush(img_);
        vpDisplay::display(img_);
        vpDisplay::displayCharString(img_, img_.getHeight() / 2 - 10, img_.getWidth() / 4,
                                     "Waiting for the camera feed.", vpColor::red);
        vpDisplay::displayCharString(
            img_, img_.getHeight() / 2 + 10, img_.getWidth() / 4,
            "If you are using the example camera, you should click on it's window", vpColor::red);

        vpDisplay::flush(img_);

        // init camera
        double px = cam_param_.get_px();
        double py = cam_param_.get_px();
        double u0 = img_.getWidth() / 2;
        double v0 = img_.getHeight() / 2;
        cam_param_.initPersProjWithoutDistortion(px, py, u0, v0);

        is_initialized_ = true;
    }
}

bool CamCalibration::setCameraInfoBisCallback(sensor_msgs::SetCameraInfo::Request &req,
                                              sensor_msgs::SetCameraInfo::Response &res)
{
    struct passwd *pw   = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    std::string home_dir_str(homedir);
    std::string calibration_path;
    nh_->getParam("calibration_path", calibration_path);
    ROS_INFO("saving calibration file to %s", (home_dir_str + "/" + calibration_path).c_str());
    camera_calibration_parsers::writeCalibration((home_dir_str + "/" + calibration_path),
                                                 camera_image_topic_name_, req.camera_info);
    return true;
}

void CamCalibration::processCamCalib(int curr_sample, int total_sample)
{
    ros::Rate loop_rate(200);
    double gray_level_precision;
    double size_precision;
    bool pause_at_each_frame = false;  // Wait for user input each time a new frame is recieved.

    ros::param::get("gray_level_precision", gray_level_precision);
    ros::param::get("size_precision", size_precision);
    ros::param::get("pause_at_each_frame", pause_at_each_frame);

    vpPose pose;
    vpCalibration calib;
    visp_camera_calibration::CalibPointArray calib_all_points;

    init();

    vpDisplay::display(img_);
    vpDisplay::flush(img_);

    if (!pause_at_each_frame)
    {
        try
        {
            vpImagePoint ip;
            vpDisplay::displayRectangle(img_, 0, 0, img_.getWidth(), 15, vpColor::black, true);
            vpDisplay::displayCharString(
                img_, 10, 10, "Click on the window to select the current image", vpColor::red);
            vpDisplay::flush(img_);
            if (pause_image_)
            {
                pause_image_ = false;
            }
            else
            {
                return;
            }
        }
        catch (...)
        {
            ROS_ERROR("Could not handle pause at each image key point slection");
        }
    }

    pose.clearPoint();
    calib.clearPoint();
    vpImagePoint ip;

    // lets the user select keypoints
    for (unsigned int i = 0; i < selected_points_.size(); i++)
    {
        try
        {
            vpDot2 d;
            d.setGrayLevelPrecision(gray_level_precision);
            d.setSizePrecision(size_precision);

            ROS_INFO("Click on point %d", i + 1);
            vpDisplay::displayRectangle(img_, 0, 0, img_.getWidth(), 15, vpColor::black, true);
            vpDisplay::displayCharString(
                img_, 10, 10, boost::str(boost::format("click on point %1%") % (i + 1)).c_str(),
                vpColor::red);
            vpDisplay::displayCharString(
                img_, img_.getHeight() - 10, 10,
                boost::str(boost::format("executed poses; %1%/%2%") % curr_sample % total_sample)
                    .c_str(),
                vpColor::red);
            vpDisplay::flush(img_);
            while (ros::ok() && !vpDisplay::getClick(img_, ip, false))
                ;

            d.initTracking(img_, ip);

            ip.set_ij(d.getCog().get_i(), d.getCog().get_j());
            double x = 0, y = 0;
            vpPixelMeterConversion::convertPoint(cam_param_, ip, x, y);
            selected_points_[i].set_x(x);
            selected_points_[i].set_y(y);
            pose.addPoint(selected_points_[i]);
            calib.addPoint(selected_points_[i].get_oX(), selected_points_[i].get_oY(),
                           selected_points_[i].get_oZ(), ip);

            vpDisplay::displayCross(img_, d.getCog(), 10, vpColor::red);
            vpDisplay::flush(img_);
        }
        catch (...)
        {
            ROS_ERROR(" FAILED TO SELECT KEYPOINTS ,  SELECT KEY POINTS CAREFULLY");
            QMessageBox Msgbox;
            Msgbox.setText(
                "SEVERE ERROR!!!,FAILED TO SELECT KEYPOINTS, SELECT KEY POINTS CAREFULLY \n");
            Msgbox.exec();
            return;
        }
    }
    vpHomogeneousMatrix cMo;

    try
    {
        pose.computePose(vpPose::LAGRANGE, cMo);
        pose.computePose(vpPose::VIRTUAL_VS, cMo);
    }
    catch (...)
    {
        ROS_ERROR("POSE COMPUTATION WITH LAGRANCE FAILED, SELECT KEYPOINTS MORE CAREFULLY !!!");
        QMessageBox Msgbox;
        Msgbox.setText(
            "SEVERE ERROR!!! ,LAGRANGE DIVIDED BY ZERO,  SELECT KEY POINTS CAREFULLY \n");
        Msgbox.exec();
        return;
    }

    vpHomogeneousMatrix cMoTmp = cMo;
    vpCameraParameters camTmp  = cam_param_;

    // compute local calibration to match the calibration grid with the image
    try
    {
        calib.computeCalibration(vpCalibration::CALIB_VIRTUAL_VS, cMoTmp, camTmp, false);
        ROS_DEBUG_STREAM("cMo=" << std::endl << cMoTmp << std::endl);
        ROS_DEBUG_STREAM("cam=" << std::endl << camTmp << std::endl);

        // project all points and track their corresponding image location
        for (std::vector<vpPoint>::iterator model_point_iter = model_points_.begin();
             model_point_iter != model_points_.end(); model_point_iter++)
        {
            // project each model point into image according to current calibration
            vpColVector _cP, _p;

            model_point_iter->changeFrame(cMoTmp, _cP);
            model_point_iter->projection(_cP, _p);
            vpMeterPixelConversion::convertPoint(camTmp, _p[0], _p[1], ip);
            if (10 < ip.get_u() && ip.get_u() < img_.getWidth() - 10 && 10 < ip.get_v() &&
                ip.get_v() < img_.getHeight() - 10)
            {
                try
                {
                    // track the projected point, look for match
                    vpDot2 md;
                    md.setGrayLevelPrecision(gray_level_precision);
                    md.setSizePrecision(size_precision);

                    md.initTracking(img_, ip);
                    if (!ros::ok())
                        return;

                    vpRect bbox      = md.getBBox();
                    vpImagePoint cog = md.getCog();
                    if (bbox.getLeft() < 5 || bbox.getRight() > (double)img_.getWidth() - 5 ||
                        bbox.getTop() < 5 || bbox.getBottom() > (double)img_.getHeight() - 5 ||
                        vpMath::abs(ip.get_u() - cog.get_u()) > 10 ||
                        vpMath::abs(ip.get_v() - cog.get_v()) > 10)
                    {
                        ROS_DEBUG("tracking failed[suspicious point location].");
                    }
                    else
                    {
                        // point matches
                        double x = 0, y = 0;
                        vpPixelMeterConversion::convertPoint(camTmp, cog, x, y);
                        model_point_iter->set_x(x);
                        model_point_iter->set_y(y);

                        md.display(img_, vpColor::yellow, 2);
                        visp_camera_calibration::CalibPoint cp;
                        cp.i = cog.get_i();
                        cp.j = cog.get_j();
                        cp.X = model_point_iter->get_oX();
                        cp.Y = model_point_iter->get_oY();
                        cp.Z = model_point_iter->get_oZ();

                        calib_all_points.points.push_back(cp);

                        model_point_iter->display(img_, cMoTmp, camTmp);
                        loop_rate.sleep();  // To avoid refresh problems
                        vpDisplay::flush(img_);
                    }
                }
                catch (...)
                {
                    ROS_DEBUG("tracking failed.");
                }
            }
            else
            {
                ROS_DEBUG("bad projection.");
            }
        }
        if (calib_all_points.points.size() < 36)
        {
            ROS_ERROR("ALL CIRCLERS HAS NOT BEEN DETECTED !!!, THIS POSE WILL BE SKIPPED");
            QMessageBox Msgbox;
            Msgbox.setText("ALL CIRCLERS HAS NOT BEEN DETECTED !!!, THIS POSE WILL BE SKIPPED \n");
            Msgbox.exec();
            return;
        }

        ROS_INFO("Left click on the interface window to continue, right click to restart");
        vpDisplay::displayRectangle(img_, 0, 0, img_.getWidth(), 15, vpColor::black, true);
        vpDisplay::displayCharString(
            img_, 10, 10, "Left click on the interface window to continue, right click to restart",
            vpColor::red);
        vpDisplay::flush(img_);

        vpMouseButton::vpMouseButtonType btn;
        while (ros::ok() && !vpDisplay::getClick(img_, ip, btn, false))
            ;

        if (btn == vpMouseButton::button1)
        {
            point_correspondence_publisher_.publish(calib_all_points);
            ROS_INFO_STREAM("PUBLISHING CORRESPONDE POINTS");
        }
        else
        {
            processCamCalib(curr_sample, total_sample);
            return;
        }

        visp_camera_calibration::calibrate calibrate_comm;
        calibrate_comm.request.method        = vpCalibration::CALIB_VIRTUAL_VS_DIST;
        calibrate_comm.request.sample_width  = img_.getWidth();
        calibrate_comm.request.sample_height = img_.getHeight();
        if (calibrate_service_.call(calibrate_comm))
        {
            ROS_INFO("service called successfully");

            ROS_INFO("standard deviation with distorsion:");
            for (std::vector<double>::iterator i = calibrate_comm.response.stdDevErrs.begin();
                 i != calibrate_comm.response.stdDevErrs.end(); i++)
                ROS_INFO("%f", *i);
        }
        else
        {
            ROS_ERROR("Failed to call service");
        }
    }
    catch (...)
    {
        ROS_ERROR("calibration failed.");
        QMessageBox Msgbox;
        Msgbox.setText(
            "SEVERE ERROR!!! ,CALIBRATION FAILED, MAKE SURE THE CALIBRATION BOARD IS CLEARLY "
            "VISIBLE IN CAMERA AND ALL "
            "KEYPOINTS ARE SELECTED IN AN ORDER\n");
        Msgbox.exec();
        return;
    }
}

void CamCalibration::rawImageCallback(const sensor_msgs::Image::ConstPtr &image)
{
    const std::lock_guard<std::mutex> lock(image_mutex);
    img_ = visp_bridge::toVispImage(*image);
    // the lock will be released after outta scope
}

CamCalibration::~CamCalibration()
{
    // TODO Auto-generated destructor stub
}