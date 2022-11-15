#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <fstream>
#include <iostream>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_spherical.h>
#include <boost/thread/thread.hpp>
//#include <pcl/io/png_io.h>
//#include <pcl/visualization/common/float_image_utils.h>
#include <unistd.h>
#include <chrono>
#include <time.h>
#include "alfa_node.h"
#include <opencv2/highgui.hpp>

// 64 -> 76 | 32 -> 96 | 32 -> 593
// #define NOF 76                                         //Number of frames in the rosbag
typedef long long int u64;

struct sensorParameters
{
    int sensor_tag;
    float angular_resolution_horizontal;
    float angular_resolution_horizontal_rads;
    float angular_resolution_vertical;
    float angular_resolution_vertical_rads;
    float min_vertical_angle;
    float max_angle_width;
    float max_angle_height;
    int max_sensor_distance;
    int n_columns;
};

class AlfaPsCompressor : public  AlfaNode
{
    public:
        AlfaPsCompressor(string node_name,string node_type,vector<alfa_msg::ConfigMessage>* default_configurations);
        void process_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr  input_cloud);
        alfa_msg::AlfaConfigure::Response process_config(alfa_msg::AlfaConfigure::Request &req);
        unsigned char* getVisualImage (const float* float_image, int width, int height, float min_value, float max_value, bool gray_scale);
        void getColorForFloat (float value, unsigned char& r, unsigned char& g, unsigned char& b);
        void setSensorParameters();
        alfa_msg::AlfaMetrics output_metrics;
        void calculate_metrics(int cloud_size, string png_path, float duration_ri, float duration_png, int counter);
        void avg_metrics();

        void publish_hardware();

    private:
        sensorParameters sensor_parameters;
        pcl::RangeImage range_image;
        cv::Mat image;
    
        string file_name;
        string file_name_hw;
        Eigen::Affine3f sensor_pose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
        pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
        float noise_level= 0.00f;
        float min_range = 0.0f;
        int border_size = 0.0; 

        int compression_lvl;
        vector<int> compression_params;

        double avg_exec_time_ri;
        double avg_exec_time_png;
        double avg_size_original;
        double avg_size_png;
        double total_points;

        bool hw;
        u64 *ddr_pointer;
        u_int32_t *hw32_vptr;

        double points_per_second;
        double frames_per_second;

        int NOF;
};

