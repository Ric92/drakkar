//---------------------------------------------------------------------------------------------------------------------
//  DRAKKAR
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 Ricardo Lopez Lopez (a.k.a. ric92) ricloplop@gmail.com
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

#include <drakkar/Entity.h>
#include <drakkar/cameras/RealSense.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <iostream>
#include <librealsense2/hpp/rs_internal.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <thread>

#define STREAM RS2_STREAM_COLOR  // rs2_stream is a types of data provided by RealSense device //
#define FORMAT RS2_FORMAT_BGR8   // rs2_format identifies how binary data is encoded within a frame //
#define WIDTH 640                // Defines the number of columns for each frame //
#define HEIGHT 480               // Defines the number of lines for each frame //
#define FPS 30                   // Defines the rate of frames per second //
#define STREAM_INDEX 0           // Defines the stream index, used for multiple streams of the same type //

namespace rs {
class context;
class device;

struct extrinsics;
struct intrinsics;
}  // namespace rs

int main(int argc, char const *argv[]) {
    // initialize pcl visualization
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    viewer.setBackgroundColor(100, 100, 100);
    viewer.addCoordinateSystem(0.05, "base");
    viewer.addCoordinateSystem(0.02, "current_pose");
    viewer.setCameraPosition(1.59696, 0.285761, -3.40482, -0.084178, -0.989503, -0.117468);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr body(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scaled_body(new pcl::PointCloud<pcl::PointXYZRGB>);
    bool compareModel = false;
    drakkar::Entity model;

    if (argc > 1) {
        pcl::io::loadPCDFile(argv[1], *body);
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform(0, 0) = transform(0, 0) * 0.001;
        transform(1, 1) = transform(1, 1) * 0.001;
        transform(2, 2) = transform(2, 2) * 0.001;
        pcl::transformPointCloud(*body, *scaled_body, transform);
        viewer.addPointCloud<pcl::PointXYZRGB>(scaled_body, std::string("Elephant"));
        float timeout = 1;
        float currentTime = 0;
        auto start = std::chrono::high_resolution_clock::now();

        while (currentTime < timeout) {
            auto finish = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = finish - start;
            currentTime = elapsed.count();
            viewer.spinOnce(1, true);
        }

        pcl::PointCloud<pcl::PointXYZRGB>::iterator del = body->points.begin();
        for (int i = 0; i < body->points.size(); i++) {
            if (!pcl::isFinite<pcl::PointXYZRGB>(body->points[i])) {
                //      PCL_WARN("normals[%d] is not finite\n", i);
                std::cout << "Model not finite" << std::endl;
                body->points.erase(del + i);
            }
        }
        model.model(body);
        compareModel = true;

    } else {
        std::cout << "Error usage ./matching_example model_directory" << std::endl;
        return 0;
    }

    drakkar::RealSense camera;
    camera.init();

    cv::namedWindow("RGB", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("DEPTH", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("ROI", CV_WINDOW_AUTOSIZE);
    // cv::startWindowThread();
    compareModel = true;
    bool initialized = false;
    cv::Rect2d r;
    while (true) {
        camera.grab();
        cv::Mat color, depth;
        camera.rgb(color);
        camera.depth(depth);

        cv::imshow("RGB", color);
        cv::imshow("DEPTH", depth);

        // compute dense cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        auto start = std::chrono::high_resolution_clock::now();
        camera.cloud(*cloud);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        std::cout << "PointCloud latency: " << elapsed.count() << " s \n";

        if(compareModel ){
            if(!initialized){
                r = cv::selectROI("ROI",color);
                initialized = true;
            }
            // get model cloud from scene
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            for (int dy = r.y; dy < r.y + r.height; dy++) {
                for (int dx = r.x; dx < r.x + r.width; dx++) {
                    pcl::PointXYZRGB p = cloud->at(dx,dy);
                    if(!boost::math::isnan(p.x) && !boost::math::isnan(p.y) && !boost::math::isnan(p.z) 
                        &&!boost::math::isnan(-p.x) && !boost::math::isnan(-p.y) && !boost::math::isnan(-p.z)){
                            if(pcl::isFinite<pcl::PointXYZRGB>(p))
                                modelCloud->push_back(p);
                        
                    }
                }
            }
            
            // similarity between the scene and the model
            model.computeSimilarity(modelCloud);
            auto pose = model.pose();

            // visualize the scene
            std::cout << "Transformation: " << pose << std::endl;
            viewer.updatePointCloudPose(std::string("Elephant"), Eigen::Affine3f(pose));

            viewer.removePointCloud(std::string("modelCloud"));
            viewer.addPointCloud<pcl::PointXYZRGB>(modelCloud, std::string("modelCloud"));
            viewer.updatePointCloudPose(std::string("modelCloud"), Eigen::Affine3f::Identity());
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, std::string("modelCloud"));

        }

        // viewer.removePointCloud(std::string("Entity"));
        // viewer.addPointCloud<pcl::PointXYZRGB>(modelCloud, std::string("Entity"));
        // viewer.updatePointCloudPose(std::string("Entity"), Eigen::Affine3f::Identity());
        // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, std::string("Entity"));
        viewer.spinOnce(1, true);
        cv::waitKey(1);
    }
    return 0;
}
