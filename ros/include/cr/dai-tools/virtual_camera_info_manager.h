#pragma once

/***
 * Note: This is due to http://wiki.ros.org/camera_info_manager; and is largely a fork/copy of what is in
 * https://github.com/ros-perception/image_common/commit/e9c8c3246007d7717708b5267e1e2e909e4c0694. Which we could use
 * directly EXCEPT that we can't quite be assured that that version is present; so we just do it as a shim
 */
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

#include <ros/macros.h>

namespace shim {
    namespace camera_info_manager
    {
        class ROS_HELPER_EXPORT CameraInfoManager
                {
                        public:

                        CameraInfoManager(ros::NodeHandle nh,
                        const std::string &cname="camera",
                        const std::string &url="");
                        virtual ~CameraInfoManager();

                        virtual sensor_msgs::CameraInfo getCameraInfo(void);
                        virtual bool isCalibrated(void);
                        virtual bool loadCameraInfo(const std::string &url);
                        virtual std::string resolveURL(const std::string &url,
                        const std::string &cname);
                        virtual bool setCameraName(const std::string &cname);
                        virtual bool setCameraInfo(const sensor_msgs::CameraInfo &camera_info);
                        virtual bool validateURL(const std::string &url);

                        protected:

                        // recognized URL types
                        typedef enum
                        {
                            // supported URLs
                            URL_empty = 0,             // empty string
                                    URL_file,                  // file:
                                    URL_package,               // package:
                                    // URLs not supported
                                    URL_invalid,               // anything >= is invalid
                                    URL_flash,                 // flash:
                        } url_type_t;

                        // private methods
                        std::string getPackageFileName(const std::string &url);
                        virtual bool loadCalibration(const std::string &url,
                        const std::string &cname);
                        virtual bool loadCalibrationFile(const std::string &filename,
                        const std::string &cname);
                        virtual bool loadCalibrationFlash(const std::string &flashURL,
                        const std::string &cname);
                        url_type_t parseURL(const std::string &url);
                        virtual bool saveCalibration(const sensor_msgs::CameraInfo &new_info,
                        const std::string &url,
                        const std::string &cname);
                        virtual bool saveCalibrationFile(const sensor_msgs::CameraInfo &new_info,
                        const std::string &filename,
                        const std::string &cname);
                        virtual bool saveCalibrationFlash(const sensor_msgs::CameraInfo &new_info,
                        const std::string &flashURL,
                        const std::string &cname);
                        virtual bool setCameraInfoService(sensor_msgs::SetCameraInfo::Request &req,
                        sensor_msgs::SetCameraInfo::Response &rsp);

                        /** @brief mutual exclusion lock for private data
                         *
                         *  This non-recursive mutex is only held for a short time while
                         *  accessing or changing private class variables.  To avoid
                         *  deadlocks and contention, it is never held during I/O or while
                         *  invoking a callback.  Most private methods operate on copies of
                         *  class variables, keeping the mutex hold time short.
                         */
                        boost::mutex mutex_;

                        // private data
                        ros::NodeHandle nh_;                  ///< node handle for service
                        ros::ServiceServer info_service_;     ///< set_camera_info service
                        std::string camera_name_;             ///< camera name
                        std::string url_;                     ///< URL for calibration data
                        sensor_msgs::CameraInfo cam_info_;    ///< current CameraInfo
                        bool loaded_cam_info_;                ///< cam_info_ load attempted

                }; // class CameraInfoManager

    } // namespace camera_info_manager
}