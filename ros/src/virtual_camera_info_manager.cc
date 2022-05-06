#include "cr/dai-tools/virtual_camera_info_manager.h"

#include <string>
#include <stdlib.h>
#include <sys/stat.h>
#ifdef _WIN32
#ifndef S_ISDIR
    #define S_ISDIR(mode) (((mode) & S_IFMT) == S_IFDIR)
  #endif
#else
#include <unistd.h>
#endif

#include "cr/dai-tools/ros_headers.h"

namespace shim {
    namespace camera_info_manager
    {

/** URL to use when no other is defined. */
        const std::string
                default_camera_info_url = "file://${ROS_HOME}/camera_info/${NAME}.yaml";

/** Constructor
 *
 * @param nh node handle, normally for the driver's streaming name
 *           space ("camera").  The service name is relative to this
 *           handle.  Nodes supporting multiple cameras may use
 *           subordinate names, like "left/camera" and "right/camera".
 * @param cname default camera name
 * @param url default Uniform Resource Locator for loading and saving data.
 */
        CameraInfoManager::CameraInfoManager(ros_impl::Node nh,
                                             const std::string &cname,
                                             const std::string &url):
                nh_(nh),
                camera_name_(cname),
                url_(url),
                loaded_cam_info_(false)
        {
            // register callback for camera calibration service request
            ros_impl::create_service<ros_impl::sensor_msgs::SetCameraInfo>(nh_, "set_camera_info", &CameraInfoManager::setCameraInfoService, this);
        }

/** Get the current CameraInfo data.
 *
 * If CameraInfo has not yet been loaded, an attempt must be made
 * here.  To avoid that, ensure that loadCameraInfo() ran previously.
 * If the load is attempted but fails, an empty CameraInfo will be
 * supplied.
 *
 * The matrices are all zeros if no calibration is available. The
 * image pipeline handles that as uncalibrated data.
 *
 * @warning The caller @em must fill in the message Header of the
 *          CameraInfo returned.  The time stamp and frame_id should
 *          normally be the same as the corresponding Image message
 *          Header fields.
 */
        ros_impl::sensor_msgs::CameraInfo CameraInfoManager::getCameraInfo(void)
        {
            {
                std::string cname;
                std::string url;
                {
                    std::lock_guard lock_(mutex_);
                    if (loaded_cam_info_)
                    {
                        //assert(cam_info_.header.frame_id != "");
                        return cam_info_;           // all done
                    }

                    // load being attempted now
                    loaded_cam_info_ = true;

                    // copy the name and URL strings
                    url = url_;
                    cname = camera_name_;

                } // release the lock

                // attempt load without the lock, it is not recursive
                if(loadCalibration(url, cname)) {
                    return cam_info_;           // all done
                }
            }

            return ros_impl::sensor_msgs::CameraInfo();
        }

/** Is the current CameraInfo calibrated?
 *
 * If CameraInfo has not yet been loaded, an attempt must be made
 * here.  To avoid that, ensure that loadCameraInfo() ran previously.
 * If the load failed, CameraInfo will be empty and this predicate
 * will return false.
 *
 * @return true if the current CameraInfo is calibrated.
 */
        bool CameraInfoManager::isCalibrated(void)
        {
            {
                std::string cname;
                std::string url;
                {
                    std::lock_guard lock_(mutex_);
                    if (loaded_cam_info_)
                    {
                        return (cam_info_.K[0] != 0.0);
                    }

                    // load being attempted now
                    loaded_cam_info_ = true;

                    // copy the name and URL strings
                    url = url_;
                    cname = camera_name_;

                } // release the lock

                // attempt load without the lock, it is not recursive
                loadCalibration(url, cname);
            }
            return (cam_info_.K[0] != 0.0);
        }

/** Load CameraInfo calibration data (if any).
 *
 * @pre mutex_ unlocked
 *
 * @param url a copy of the Uniform Resource Locator
 * @param cname is a copy of the camera_name_
 * @return true if URL contains calibration data.
 *
 * sets cam_info_, if successful
 */
        bool CameraInfoManager::loadCalibration(const std::string &url,
                                                const std::string &cname)
        {
            bool success = false;                 // return value

            const std::string resURL(resolveURL(url, cname));
            url_type_t url_type = parseURL(resURL);

            if (url_type != URL_empty)
            {
                ROS_IMPL_INFO(this->nh_, "camera calibration URL: %s", resURL.c_str());
            }

            switch (url_type)
            {
                case URL_empty:
                {
                    success = loadCalibration(default_camera_info_url, cname);
                    break;
                }
                case URL_file:
                {
                    success = loadCalibrationFile(resURL.substr(7), cname);
                    break;
                }
                case URL_flash:
                {
                    success = loadCalibrationFlash(resURL.substr(8), cname);
                    break;
                }
                default:
                {
                    ROS_IMPL_ERROR(nh_, "Invalid camera calibration URL: %s", url_.c_str() );
                    break;
                }
            }

            return success;
        }

        bool CameraInfoManager::loadCalibrationFlash(const std::string &flashURL,
                                                     const std::string &cname) {
            ROS_IMPL_WARN(nh_, "[CameraInfoManager] reading from flash not implemented yet");
            return false;
        }

/** Load CameraInfo calibration data from a file.
 *
 * @pre mutex_ unlocked
 *
 * @param filename containing CameraInfo to read
 * @param cname is a copy of the camera_name_
 * @return true if URL contains calibration data.
 *
 * Sets cam_info_, if successful
 */
        bool CameraInfoManager::loadCalibrationFile(const std::string &filename,
                                                    const std::string &cname)
        {
            bool success = false;

            std::string cam_name;
            ros_impl::sensor_msgs::CameraInfo cam_info;

            if (::camera_calibration_parsers::readCalibration(filename, cam_name, cam_info))
            {
                cam_info.header.frame_id = cname;
                success = true;
                {
                    // lock only while updating cam_info_
                    std::lock_guard lock(mutex_);
                    cam_info_ = cam_info;
                }
            }
            else
            {
                ROS_IMPL_WARN(nh_, "Camera calibration file %s not found.", filename.c_str());
            }

            return success;
        }

/** Set a new URL and load its calibration data (if any).
 *
 * If multiple threads call this method simultaneously with different
 * URLs, there is no guarantee which will prevail.
 *
 * @param url new Uniform Resource Locator for CameraInfo.
 * @return true if new URL contains calibration data.
 *
 * @post @c loaded_cam_info_ true (meaning a load was attempted, even
 *       if it failed); @c cam_info_ updated, if successful.
 */
        bool CameraInfoManager::loadCameraInfo(const std::string &url)
        {
            std::string cname;
            {
                std::lock_guard lock(mutex_);
                url_ = url;
                cname = camera_name_;
                loaded_cam_info_ = true;
            }

            // load using copies of the parameters, no need to hold the lock
            return loadCalibration(url, cname);
        }


/** Resolve Uniform Resource Locator string.
 *
 * @param url a copy of the Uniform Resource Locator, which may
 *            include <tt>${...}</tt> substitution variables.
 * @param cname is a copy of the camera_name_
 *
 * @return a copy of the URL with any variable information resolved.
 */
        std::string CameraInfoManager::resolveURL(const std::string &url,
                                                  const std::string &cname)
        {
            std::string resolved;
            size_t rest = 0;

            while (true)
            {
                // find the next '$' in the URL string
                size_t dollar  = url.find('$', rest);

                if (dollar >= url.length())
                {
                    // no more variables left in the URL
                    resolved += url.substr(rest);
                    break;
                }

                // copy characters up to the next '$'
                resolved += url.substr(rest, dollar-rest);

                if (url.substr(dollar+1, 1) != "{")
                {
                    // no '{' follows, so keep the '$'
                    resolved += "$";
                }
                else if (url.substr(dollar+1, 6) == "{NAME}")
                {
                    // substitute camera name
                    resolved += cname;
                    dollar += 6;
                }
                else if (url.substr(dollar+1, 10) == "{ROS_HOME}")
                {
                    // substitute $ROS_HOME
                    std::string ros_home;
                    char *ros_home_env;
                    if ((ros_home_env = getenv("ROS_HOME")))
                    {
                        // use environment variable
                        ros_home = ros_home_env;
                    }
                    else if ((ros_home_env = getenv("HOME")))
                    {
                        // use "$HOME/.ros"
                        ros_home = ros_home_env;
                        ros_home += "/.ros";
                    }
                    resolved += ros_home;
                    dollar += 10;
                }
                else
                {
                    // not a valid substitution variable
                    ROS_IMPL_ERROR(nh_, "[CameraInfoManager]"
                                     " invalid URL substitution (not resolved): %s", url.c_str());
                    resolved += "$";            // keep the bogus '$'
                }

                // look for next '$'
                rest = dollar + 1;
            }

            return resolved;
        }

/** Parse calibration Uniform Resource Locator.
 *
 * @param url string to parse
 * @return URL type
 *
 * @note Recognized but unsupported URL types have enum values >= URL_invalid.
 */
        CameraInfoManager::url_type_t CameraInfoManager::parseURL(const std::string &url)
        {
            if (url == "")
            {
                return URL_empty;
            }
            if ((url.substr(0, 8) == "file:///"))
            {
                return URL_file;
            }
            if (url.substr(0, 9) == "flash:///")
            {
                return URL_flash;
            }
            if (url.substr(0, 10) == "package://")
            {
                // look for a '/' following the package name, make sure it is
                // there, the name is not empty, and something follows it
                size_t rest = url.find('/', 10);
                if (rest < url.length()-1 && rest > 10)
                    return URL_package;
            }
            return URL_invalid;
        }

/** Save CameraInfo calibration data.
 *
 * @pre mutex_ unlocked
 *
 * @param new_info contains CameraInfo to save
 * @param url is a copy of the URL storage location (if empty, use
 *            @c file://${ROS_HOME}/camera_info/${NAME}.yaml)
 * @param cname is a copy of the camera_name_
 * @return true, if successful
 */
        bool
        CameraInfoManager::saveCalibration(const ros_impl::sensor_msgs::CameraInfo &new_info,
                                           const std::string &url,
                                           const std::string &cname)
        {
            bool success = false;

            const std::string resURL(resolveURL(url, cname));

            switch (parseURL(resURL))
            {
                case URL_empty:
                {
                    // store using default file name
                    success = saveCalibration(new_info, default_camera_info_url, cname);
                    break;
                }
                case URL_file:
                {
                    success = saveCalibrationFile(new_info, resURL.substr(7), cname);
                    break;
                }
                case URL_flash:
                {
                    success = saveCalibrationFlash(new_info, resURL.substr(8), cname);
                    break;
                }
                default:
                {
                    // invalid URL, save to default location
                    ROS_IMPL_ERROR(nh_, "invalid url: %s (ignored)", url.c_str());
                    success = saveCalibration(new_info, default_camera_info_url, cname);
                    break;
                }
            }

            return success;
        }

        bool CameraInfoManager::saveCalibrationFlash(const ros_impl::sensor_msgs::CameraInfo &new_info,
                                                     const std::string &flashURL,
                                                     const std::string &cname) {
            return saveCalibration(new_info, default_camera_info_url, cname);
        }

/** Save CameraInfo calibration data to a file.
 *
 * @pre mutex_ unlocked
 *
 * @param new_info contains CameraInfo to save
 * @param filename is local file to store data
 * @param cname is a copy of the camera_name_
 * @return true, if successful
 */
        bool
        CameraInfoManager::saveCalibrationFile(const ros_impl::sensor_msgs::CameraInfo &new_info,
                                               const std::string &filename,
                                               const std::string &cname)
        {
            // isolate the name of the containing directory
            size_t last_slash = filename.rfind('/');
            if (last_slash >= filename.length())
            {
                // No slash in the name.  This should never happen, the URL
                // parser ensures there is at least one '/' at the beginning.
                return false;                     // not a valid URL
            }

            // make sure the directory exists and is writable
            std::string dirname(filename.substr(0, last_slash+1));
            struct stat stat_data;
            int rc = stat(dirname.c_str(), &stat_data);
            if (rc != 0)
            {
                if (errno == ENOENT)
                {
                    // directory does not exist, try to create it and its parents
                    std::string command("mkdir -p " + dirname);
                    rc = system(command.c_str());
                    if (rc != 0)
                    {
                        // mkdir failed
                        return false;
                    }
                }
                else
                {
                    // not accessible, or something screwy
                   return false;
                }
            }
            else if (!S_ISDIR(stat_data.st_mode))
            {
                // dirname exists but is not a directory
                return false;
            }

            // Directory exists and is accessible. Permissions might still be bad.

            // Currently, writeCalibration() always returns true no matter what
            // (ros-pkg ticket #5010).
            return ::camera_calibration_parsers::writeCalibration(filename, cname, new_info);
        }

/** Callback for SetCameraInfo request.
 *
 * Always updates cam_info_ class variable, even if save fails.
 *
 * @param req SetCameraInfo request message
 * @param rsp SetCameraInfo response message
 * @return true if message handled
 */
        bool
        CameraInfoManager::setCameraInfoService(ros_impl::sensor_msgs::SetCameraInfo::Request &req,
                                                ros_impl::sensor_msgs::SetCameraInfo::Response &rsp)
        {
            // copies of class variables needed for saving calibration
            std::string url_copy;
            std::string cname;
            {
                std::lock_guard lock(mutex_);
                cam_info_ = req.camera_info;
                url_copy = url_;
                cname = camera_name_;
                loaded_cam_info_ = true;
            }

            rsp.success = saveCalibration(req.camera_info, url_copy, cname);
            if (!rsp.success)
                rsp.status_message = "Error storing camera calibration.";

            return true;
        }

/** Set a new camera name.
 *
 * @param cname new camera name to use for saving calibration data
 *
 * @return true if new name has valid syntax; valid names contain only
 *              alphabetic, numeric, or '_' characters.
 *
 * @post @c cam_name_ updated, if valid; since it may affect the URL,
 *       @c cam_info_ will be reloaded before being used again.
 */
        bool CameraInfoManager::setCameraName(const std::string &cname)
        {
            // the camera name may not be empty
            if (cname.empty())
                return false;

            // validate the camera name characters
            for (unsigned i = 0; i < cname.size(); ++i)
            {
                if (!isalnum(cname[i]) && cname[i] != '_')
                    return false;
            }

            // The name is valid, so update our private copy.  Since the new
            // name might cause the existing URL to resolve somewhere else,
            // force @c cam_info_ to be reloaded before being used again.
            {
                std::lock_guard lock(mutex_);
                camera_name_ = cname;
                loaded_cam_info_ = false;
            }

            return true;
        }

/** Set the camera info manually
 *
 * @param camera_info new camera calibration data
 *
 * @return true if new camera info is set
 *
 * @post @c cam_info_ updated, if valid;
 */
        bool CameraInfoManager::setCameraInfo(const ros_impl::sensor_msgs::CameraInfo &camera_info)
        {
            std::lock_guard lock(mutex_);

            cam_info_ = camera_info;
            loaded_cam_info_ = true;

            return true;
        }

/** Validate URL syntax.
 *
 * @param url Uniform Resource Locator to check
 *
 * @return true if URL syntax is supported by CameraInfoManager
 *              (although the resource need not actually exist)
 */
        bool CameraInfoManager::validateURL(const std::string &url)
        {
            std::string cname;                    // copy of camera name
            {
                std::lock_guard lock(mutex_);
                cname = camera_name_;
            } // release the lock

            url_type_t url_type = parseURL(resolveURL(url, cname));
            return (url_type < URL_invalid);
        }

        CameraInfoManager::~CameraInfoManager() {

        }

    } // namespace camera_info_manager
}