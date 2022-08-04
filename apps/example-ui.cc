#define DEPTHAI_HAVE_OPENCV_SUPPORT 1
#include <depthai/depthai.hpp>
#include <depthai/device/Device.hpp>
#include "cr/dai-tools/DeviceRunner.h"
#include "cr/dai-tools/NodeWalker.h"
#include "cr/dai-tools/Utils.h"

#include <opencv2/opencv.hpp>

bool invert_tx = true, flip = true, norgb = false;
bool output_error = false, output_amplitude = false;

void initUndistortRectifyMap(const float *lp,
                             int height, int width, float *map12) {
    float u0 = lp[2], v0 = lp[3];
    float fx = lp[0], fy = lp[1];

    float k1 = lp[4];
    float k2 = lp[5];
    float p1 = lp[6];
    float p2 = lp[7];
    float k3 = lp[8];
    float k4 = lp[9];
    float k5 = lp[10];
    float k6 = lp[11];

    float ir[] = {
            1.f / fx, 0, -u0 / fx,
            0, 1.f / fy, -v0 / fy,
            0, 0, 1
    };

    for (int i = 0; i < height; i++) {
        float _x = i * ir[1] + ir[2], _y = i * ir[4] + ir[5], _w = i * ir[7] + ir[8];

        for (int j = 0; j < width; j++, _x += ir[0], _y += ir[3], _w += ir[6]) {
            float w = 1.f / _w, x = _x * w, y = _y * w;
            float x2 = x * x, y2 = y * y;
            float r2 = x2 + y2, _2xy = 2 * x * y;
            float kr = (1 + ((k3 * r2 + k2) * r2 + k1) * r2) / (1 + ((k6 * r2 + k5) * r2 + k4) * r2);
            float u = fx * (x * kr + p1 * _2xy + p2 * (r2 + 2 * x2)) + u0;
            float v = fy * (y * kr + p1 * (r2 + 2 * y2) + p2 * _2xy) + v0;

            map12[(width * i + j) * 3 + 0] = u;
            map12[(width * i + j) * 3 + 1] = v;
        }
    }
}

void fill_pointcloud_map(const float *lp, float *pc_map, int width, int height) {
    float u0 = lp[2], v0 = lp[3];
    float fx = lp[0], fy = lp[1];

    initUndistortRectifyMap(lp, height, width, pc_map);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            size_t index = (width * y + x) * 3;
            float xs = pc_map[index + 0];
            float ys = pc_map[index + 1];
            float rx = (xs - u0) / fx;
            float ry = (ys - v0) / fy;

            pc_map[index + 0] = 1. / sqrt(rx * rx + ry * ry + 1);
            pc_map[index + 1] = rx;
            pc_map[index + 2] = ry;
        }
    }
}

float ext_scale = .01;

static inline void transform_point(const std::vector<std::vector<float>> &tx, const float *xyz, float *tx_xyz) {
    float X = xyz[0], Y = xyz[1], Z = xyz[2];
    tx_xyz[0] = tx[0][0] * X + tx[0][1] * Y + tx[0][2] * Z + tx[0][3] * ext_scale;
    tx_xyz[1] = tx[1][0] * X + tx[1][1] * Y + tx[1][2] * Z + tx[1][3] * ext_scale;
    tx_xyz[2] = tx[2][0] * X + tx[2][1] * Y + tx[2][2] * Z + tx[2][3] * ext_scale;
}

static inline void project_points(const float *lp, const float *xyz,
                                  float *uv) {
    const float fx = lp[0], fy = lp[1], cx = lp[2], cy = lp[3] ;

    float x = xyz[0], y = xyz[1], z = xyz[2];
    z = z ? 1. / z : 1;
    x *= z;
    y *= z;

    const float r2 = x * x + y * y;
    const float r4 = r2 * r2;
    const float r6 = r4 * r2;
    const float a1 = 2 * x * y;
    const float a2 = r2 + 2 * x * x;
    const float a3 = r2 + 2 * y * y;

    const float cdist = 1 + lp[4 + 0] * r2 + lp[4 + 1] * r4 + lp[4 + 4] * r6;
    const float icdist2 = 1.f / (1 + lp[4 + 5] * r2 + lp[4 + 6] * r4 + lp[4 + 7] * r6);
    const float xd0 = x * cdist * icdist2 + lp[4 + 2] * a1 + lp[4 + 3] * a2 + lp[4 + 8] * r2 + lp[4 + 9] * r4;
    const float yd0 = y * cdist * icdist2 + lp[4 + 2] * a3 + lp[4 + 3] * a1 + lp[4 + 10] * r2 + lp[4 + 11] * r4;

    const float vecTilt[3] = {xd0, yd0, 1};
    const float invProj = vecTilt[2] != 0 ? 1.f / vecTilt[2] : 1;
    const float xd = invProj * vecTilt[0];
    const float yd = invProj * vecTilt[1];

    uv[0] = xd * fx + cx;
    uv[1] = yd * fy + cy;
}

cv::Mat lastRGB;

struct DepthaAICamera {
    std::string name;
    float scale = -1;

    DepthaAICamera(const std::string &name, float scale = -1)
            : name(name), scale(scale) {}

    void publish(const cv::Mat &cvFrame, const cv::Mat *blend = 0) const {
        cv::namedWindow(name, cv::WINDOW_GUI_EXPANDED);
        if (cvFrame.channels() == 3) {
            lastRGB = cvFrame;
            cv::Mat img_color = cvFrame.clone();
            if(flip) {
                cv::rotate(img_color, img_color, cv::ROTATE_180);
            }
            cv::imshow(name, img_color);
        } else {
            cv::Mat as8u;
            if(scale > 0) {
                cv::convertScaleAbs(cvFrame, as8u, 1 / scale * 255., 0);
            } else {
                cv::normalize(cvFrame, as8u, 0, 255, cv::NORM_MINMAX, CV_8U);
            }
            cv::Mat img_color;
            applyColorMap(as8u, img_color, cv::COLORMAP_JET);

            cv::Mat mask;
            cv::inRange(cvFrame, cv::Scalar(0), cv::Scalar(0), mask);
            img_color.setTo(cv::Scalar(0, 0, 0), mask);

            if (blend && blend->data)
                addWeighted(img_color, .5, *blend, .5, 0.0, img_color);

            if(flip) {
                cv::rotate(img_color, img_color, cv::ROTATE_180);
            }
            cv::imshow(name, img_color);
        }
    }
};

std::vector<float> create_lp(const std::vector<std::vector<float>> &k, const std::vector<float> &d, float scale = 1) {
    std::vector<float> rtn = {
            k[0][0] * scale,
            k[1][1] * scale,
            k[0][2] * scale,
            k[1][2] * scale,
            d[0], d[1], d[2], d[3], d[4], d[5],
            d[6], d[7], d[8]
    };
    rtn.resize(12);
    return rtn;
}

static inline void xyz_from_depth(float depth, const float *zxy, float *xyz) {
    float z_scale = /*zxy[0]*/1, nx_over_z = zxy[1], ny_over_z = zxy[2];
    const float z = fmax(0, depth * z_scale);

    xyz[0] = nx_over_z * z;
    xyz[1] = ny_over_z * z;
    xyz[2] = z;
}

cv::Mat registerDepth(const cv::Mat &depth, const float *pc_map, const std::vector<std::vector<float>> &tx,
                      const std::vector<float> &depth_lp, const std::vector<float> &rgb_lp, int height, int width,
                      float scale) {
    cv::Mat_<uint16_t> registeredDepth = cv::Mat_<uint16_t>::zeros(height / scale, width / scale);
    auto scaled_rgb = rgb_lp;
    for (int i = 0; i < 4; i++) scaled_rgb[i] = rgb_lp[i] / scale;

    for (int i = 0; i < depth.cols; i++) {
        for (int j = 0; j < depth.rows; j++) {
            uint16_t d = depth.at<uint16_t>(j, i);
            if (d == 0) continue;
            float xyz[3];
            float uv[2];
            xyz_from_depth(d / 1000.f, pc_map + 3 * (i + j * depth.cols), xyz);
            transform_point(tx, xyz, xyz);
            project_points(scaled_rgb.data(), xyz, uv);
            if (uv[0] >= 0 && uv[0] < registeredDepth.cols)
                if (uv[1] >= 0 && uv[1] < registeredDepth.rows) {
                    registeredDepth(uv[1], uv[0]) = d;
                }
        }
    }
    return registeredDepth;
}

struct OpenCVRunner : public cr::dai_tools::AutoDeviceRunner, cr::dai_tools::NodeWalker {
    std::map<std::string, std::shared_ptr<DepthaAICamera>> cameras;
    int width = 0, height = 0;
    std::vector<std::vector<float>> tx;
    std::vector<float> depth_lp, rgb_lp;
    dai::CalibrationHandler calibration;
    dai::EepromData eepromData;
    dai::CameraBoardSocket rgb_socket = dai::CameraBoardSocket::AUTO, depth_socket = dai::CameraBoardSocket::AUTO;
    std::vector<float> pc_map;

    void OnSetupPipeline(std::shared_ptr<dai::Pipeline> pipeline) override {
        DeviceRunner::OnSetupPipeline(pipeline);
        calibration = this->device->readCalibration();
        eepromData = calibration.getEepromData();

        this->VisitAll(pipeline);

        if(rgb_socket != dai::CameraBoardSocket::AUTO && depth_socket != dai::CameraBoardSocket::AUTO) {
            tx = !invert_tx ?
                    calibration.getCameraExtrinsics(rgb_socket, depth_socket) :
                    calibration.getCameraExtrinsics(depth_socket, rgb_socket);
        }

        const std::set<std::string> outputs = {
                "rgb",
                "depth",
                "stereo_depth",
                "isp",
                "out",
                "amplitude"
        };

        for(auto& it : cr::dai_tools::GetXLinkOuts(*pipeline)) {
            auto xLinkOut = std::get<0>(it);
            auto outputName = std::get<1>(it);

            if(outputs.find(outputName) == outputs.end()) {
                std::cout << "Setting " << outputName << " to not output." << std::endl;
                xLinkOut->setMetadataOnly(true);
            } else {
                std::cout << "Setting " << outputName << " " << xLinkOut->getStreamName() << " to output." << std::endl;
                xLinkOut->setMetadataOnly(false);
            }
        }
    }

    virtual bool Visit(std::shared_ptr <dai::node::ColorCamera> cam) {
        rgb_socket = cam->getBoardSocket();
        auto rgb_intrinsics = calibration.getCameraIntrinsics(rgb_socket);
        auto rgb_distortion = calibration.getDistortionCoefficients(rgb_socket);

        rgb_lp = create_lp(rgb_intrinsics, rgb_distortion);

        return true;
    }


    virtual bool Visit(std::shared_ptr <dai::node::MonoCamera> cam) {
        return true;
    }

    virtual bool Visit(std::shared_ptr <dai::node::StereoDepth> cam) {
        auto alignSocket = cam->properties.depthAlignCamera;
        if(alignSocket == dai::CameraBoardSocket::AUTO) {
            if(cam->initialConfig.get().algorithmControl.depthAlign == dai::RawStereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_RIGHT) {
                alignSocket = dai::CameraBoardSocket::RIGHT;
            } else {
                alignSocket = dai::CameraBoardSocket::LEFT;
            }
        }
        int height = 0, width = 0;
        height = cam->properties.outHeight.value_or(height);
        width = cam->properties.outWidth.value_or(width);

        depth_socket = alignSocket;
        auto depth_intrinsics = calibration.getCameraIntrinsics(depth_socket);
        std::vector<float> depth_distortion; depth_distortion.resize(15);

        depth_lp = create_lp(depth_intrinsics, depth_distortion, 0.5);

        cameras["registered_depth"] = std::make_shared<DepthaAICamera>("registered_depth", 3000);

        auto depth_data = eepromData.cameraData[depth_socket];
        pc_map.resize(3 * depth_data.width * depth_data.height);
        fill_pointcloud_map(depth_lp.data(), pc_map.data(), depth_data.width * .5, depth_data.height * .5);

        return true;
    }

#ifdef HAS_CR_FORK
    virtual bool Visit(std::shared_ptr <dai::node::ToF> cam) {
        depth_socket = dai::CameraBoardSocket::RGB;
        auto depth_intrinsics = calibration.getCameraIntrinsics(depth_socket);
        auto depth_distortion = calibration.getDistortionCoefficients(depth_socket);

        depth_lp = create_lp(depth_intrinsics, depth_distortion);

        cameras["registered_depth"] = std::make_shared<DepthaAICamera>("registered_depth", 3000);

        auto depth_data = eepromData.cameraData[depth_socket];
        pc_map.resize(3 * depth_data.width * depth_data.height);
        fill_pointcloud_map(depth_lp.data(), pc_map.data(), depth_data.width, depth_data.height);

        return true;
    }
#endif

    void OnStreamData(const std::string &event, std::shared_ptr<dai::ImgFrame> img) override {
        DeviceRunner::OnStreamData(event, img);

        auto ch = cv::waitKey(1);
        if (ch == 'q' || ch == 'Q') device->close();

        if(img->getData().empty())
            return;

        cv::Mat cvFrame;
        std::string encoding;
        {
            cvFrame = img->getFrame();

            if (img->getType() == dai::ImgFrame::Type::NV12) {
                cvFrame = img->getCvFrame();
            } else if (img->getType() == dai::ImgFrame::Type::YUV420p) {
                cvFrame = img->getCvFrame();
            }
        }

        bool isDepth = event.find("depth") != std::string::npos;
        if(cameras[event] == 0) {
            if(isDepth) {
                cameras[event] = std::make_shared<DepthaAICamera>(event, 3000);
            } else {
                cameras[event] = std::make_shared<DepthaAICamera>(event);
            }
        }

        if (cameras[event] && cvFrame.rows > 1 && cvFrame.cols > 1)
            cameras[event]->publish(cvFrame);
        if (event.find("rgb") != std::string::npos) {
            width = cvFrame.cols;
            height = cvFrame.rows;
        }
        if (width != 0 && isDepth && cvFrame.rows > 1 && cvFrame.cols > 1 && !tx.empty()) {
            auto registered_depth = registerDepth(cvFrame, pc_map.data(), tx, depth_lp, rgb_lp, height, width, 4);
            cv::Mat downsample;
            cv::resize(lastRGB, downsample, cv::Size(registered_depth.cols, registered_depth.rows));
            cameras["registered_depth"]->publish(registered_depth, &downsample);
        }
    }
};

int main(int argc, char **argv) {

    OpenCVRunner runner;
    runner.Run();
    return 0;
}