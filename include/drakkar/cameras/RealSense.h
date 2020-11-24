//---------------------------------------------------------------------------------------------------------------------
//  DRAKKAR extracted from mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to
//  deal in the Software without restriction, including without limitation the
//  rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
//  sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//  IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#ifndef DRAKKAR_CAMERAS_REALSENSE_H_
#define DRAKKAR_CAMERAS_REALSENSE_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <librealsense2/hpp/rs_internal.hpp>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>

#define STREAM RS2_STREAM_COLOR  // rs2_stream is a types of data provided by RealSense device //
#define FORMAT RS2_FORMAT_BGR8   // rs2_format identifies how binary data is encoded within a frame //
#define WIDTH 640                // Defines the number of columns for each frame //
#define HEIGHT 480               // Defines the number of lines for each frame //
#define FPS 30                   // Defines the rate of frames per second //
#define STREAM_INDEX 0           // Defines the stream index, used for multiple streams of the same type //

namespace rs {
class context;
class device;
class depth_sensor;
struct extrinsics;
struct intrinsics;
}  // namespace rs

namespace drakkar {

class RealSense {
   public:
    RealSense();

    bool init();

    bool grab();

    bool rgb(cv::Mat &_color);

    bool depth(cv::Mat &_depth);

    bool cloud(pcl::PointCloud<pcl::PointXYZRGB> &_cloud);

    ~RealSense();

   private:
    std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords);

   private:
    rs2::device rsDevice;
    rs2::pipeline_profile pipe_profile;

    rs2_intrinsics rsColorIntrinsic, rsDepthIntrinsic;
    rs2_extrinsics rsDepthToColor, rsColorToDepth;

    rs2::colorizer color_map;

    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::frameset frames;

    rs2::frame color_frame;
    rs2::frame depth_frame;
    rs2::align *rsAlign;

    cv::Mat depthIntrinsic, colorIntrinsic, extrinsicColorToDepth;
    cv::Mat color_, depth_;

    float rsDepthScale;

    int downsampleStep = 1;
    bool densecloud = true;
    bool realsense_calc = false;

    bool computed_ = false;
    mutable std::mutex dataLock_;
    ;
};
}  // namespace drakkar

#endif