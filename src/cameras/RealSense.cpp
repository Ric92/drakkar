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
#include <drakkar/cameras/RealSense.h>

namespace drakkar {

RealSense::RealSense() {
}

bool RealSense::init() {
    cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, FORMAT, FPS);
    cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS);

    pipe_profile = pipe.start(cfg);

    rs2::align align_to_color(RS2_STREAM_COLOR);

    // get intrinsic extrinsic and projections from realsense2 profile
    auto depth_stream = pipe_profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream = pipe_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    rsDepthToColor = depth_stream.get_extrinsics_to(color_stream);
    rsColorToDepth = color_stream.get_extrinsics_to(depth_stream);
    rsDepthIntrinsic = depth_stream.get_intrinsics();
    rsColorIntrinsic = color_stream.get_intrinsics();

    // Projection matrix Depth
    depthIntrinsic = cv::Mat::eye(3, 3, CV_32F);
    depthIntrinsic.at<float>(0, 0) = rsDepthIntrinsic.fx;
    depthIntrinsic.at<float>(1, 1) = rsDepthIntrinsic.fy;
    depthIntrinsic.at<float>(0, 2) = rsDepthIntrinsic.ppx;
    depthIntrinsic.at<float>(1, 2) = rsDepthIntrinsic.ppy;

    // Projection matrix Color
    colorIntrinsic = cv::Mat::eye(3, 3, CV_32F);
    colorIntrinsic.at<float>(0, 0) = rsColorIntrinsic.fx;
    colorIntrinsic.at<float>(1, 1) = rsColorIntrinsic.fy;
    colorIntrinsic.at<float>(0, 2) = rsColorIntrinsic.ppx;
    colorIntrinsic.at<float>(1, 2) = rsColorIntrinsic.ppy;

    extrinsicColorToDepth = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat(3, 3, CV_32F, &rsColorToDepth.rotation[0]).copyTo(extrinsicColorToDepth(cv::Rect(0, 0, 3, 3)));
    extrinsicColorToDepth(cv::Rect(0, 0, 3, 3)) = extrinsicColorToDepth(cv::Rect(0, 0, 3, 3)).t();
    cv::Mat(3, 1, CV_32F, &rsColorToDepth.translation[0]).copyTo(extrinsicColorToDepth(cv::Rect(3, 0, 1, 3)));
    auto sensor = pipe_profile.get_device().first<rs2::depth_sensor>();
    rsDepthScale = sensor.get_depth_scale();

    rsAlign = new rs2::align(RS2_STREAM_COLOR);

    frames = pipe.wait_for_frames();

}

bool RealSense::grab() {
    frames = pipe.wait_for_frames();
    frames = rsAlign->process(frames);

    color_frame = frames.get_color_frame();
    depth_frame = frames.get_depth_frame();
    color_ = cv::Mat(cv::Size(rsColorIntrinsic.width, rsColorIntrinsic.height), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
    depth_ = cv::Mat(cv::Size(rsDepthIntrinsic.width, rsDepthIntrinsic.height), CV_16U, (uchar *)depth_frame.get_data(), cv::Mat::AUTO_STEP);
    computed_ = true;
    return true;
}

bool RealSense::rgb(cv::Mat &_color) {
    if (computed_) {
        std::lock_guard<std::mutex> lock(dataLock_);
        color_.copyTo(_color);
        return true;
    } else {
        return false;
    }
}

bool RealSense::depth(cv::Mat &_depth) {
    if (computed_) {
        std::lock_guard<std::mutex> lock(dataLock_);
        depth_.copyTo(_depth);
        return true;
    } else {
        return false;
    }
}

bool RealSense::cloud(pcl::PointCloud<pcl::PointXYZRGB> &_cloud) {
    if (realsense_calc) {
        rs2::pointcloud pc;
        // Generate the pointcloud and texture mappings
        auto points = pc.calculate(depth_frame);
        pc.map_to(color_frame);

        auto text_coords = points.get_texture_coordinates();
        auto vertices = points.get_vertices();

        auto sp = points.get_profile().as<rs2::video_stream_profile>();
        // Config of PCL Cloud object
        _cloud.width = static_cast<uint32_t>(sp.width());
        _cloud.height = static_cast<uint32_t>(sp.height());
        _cloud.is_dense = false;
        _cloud.points.resize(points.size());

        for (int i = 0; i < points.size(); i++) {
            pcl::PointXYZRGB point;

            point.x = vertices[i].x;
            point.y = vertices[i].y;
            point.z = vertices[i].z;

            std::tuple<uint8_t, uint8_t, uint8_t> current_color;
            current_color = get_texcolor(color_frame, text_coords[i]);

            point.r = std::get<2>(current_color);
            point.g = std::get<1>(current_color);
            point.b = std::get<0>(current_color);
            _cloud.push_back(point);
        }
    } else {
        for (int dy = 0; dy < depth_.rows; dy = dy + downsampleStep) {
            for (int dx = 0; dx < depth_.cols; dx = dx + downsampleStep) {
                // Retrieve the 16-bit depth value and map it into a depth in meters
                uint16_t depth_value = depth_.at<uint16_t>(dy, dx);
                float depth_in_meters = depth_value * rsDepthScale;
                pcl::PointXYZRGB point;
                // Skip over pixels with a depth value of zero, which is  used to indicate no data
                if (depth_value == 0) {
                    if (densecloud) {
                        point.x = NAN;
                        point.y = NAN;
                        point.z = NAN;
                    }
                    else
                        continue;
                } else {
                    // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                    cv::Point2f depth_pixel(dx, dy);
                    float x = (depth_pixel.x - rsDepthIntrinsic.ppx) / rsDepthIntrinsic.fx;
                    float y = (depth_pixel.y - rsDepthIntrinsic.ppy) / rsDepthIntrinsic.fy;
                    cv::Point3f depth_point(x * depth_in_meters, y * depth_in_meters, depth_in_meters);
                    point.x = depth_point.x;
                    point.y = depth_point.y;
                    point.z = depth_point.z;

                    auto rgb = color_.at<cv::Vec3b>(dy, dx);
                    point.r = rgb[2];
                    point.g = rgb[1];
                    point.b = rgb[0];

                }
                _cloud.push_back(point);
            }
        }
        if (densecloud) {
            _cloud.is_dense = false;    // 666
            _cloud.width = color_.cols / downsampleStep;
            _cloud.height = color_.rows / downsampleStep;
            _cloud.resize(_cloud.width * _cloud.height);
        }
    }
}

// Get RGB values based on normals - texcoords, normals value [u v]
std::tuple<uint8_t, uint8_t, uint8_t> RealSense::get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords) {
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels
    
    // Normals to Texture Coordinates conversion
    int x_value = std::min(std::max(int(texcoords.u * width  + .5f), 0), width - 1);
    int y_value = std::min(std::max(int(texcoords.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());
    
    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

RealSense::~RealSense() {
}
}  // namespace drakkar