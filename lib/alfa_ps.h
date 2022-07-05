#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <fstream>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_spherical.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/common/float_image_utils.h>
#include <unistd.h>
#include <pcl/io/png_io.h>
#include <chrono>
#include <time.h>
#include "alfa_node.h"
// 64 -> 77 | 32 -> 97 | 32 -> 594
#define NOF 76                                      //Number of frames in the rosbag
#define PPF 122000                                   //Number of points per frame

struct sensorParameters
{
    float angular_resolution_horizontal;
    float angular_resolution_vertical;
    float max_angle_width;
    float max_angle_height;
    int max_sensor_distance;
};

class AlfaPsCompressor : public  AlfaNode
{
    public:
        AlfaPsCompressor(string node_name,string node_type,vector<alfa_msg::ConfigMessage>* default_configurations);
        void process_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr  input_cloud);
        alfa_msg::AlfaConfigure::Response process_config(alfa_msg::AlfaConfigure::Request &req);
        void setSensorParameters();
        alfa_msg::AlfaMetrics output_metrics;
        void calculate_metrics(int cloud_size, string png_path, float duration_ri, float duration_png, int counter);
        void avg_metrics();

    private:
        sensorParameters sensor_parameters;
        pcl::RangeImage range_image;
    
        string file_name;
        Eigen::Affine3f sensor_pose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
        pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
        float noise_level= 0.00f;
        float min_range = 0.0f;
        int border_size = 0.0;

        double avg_exec_time_ri;
        double avg_exec_time_png;
        double avg_size_original;
        double avg_size_png;
        double total_points;
        double fps;
        double pps;                             //points per sec
};

