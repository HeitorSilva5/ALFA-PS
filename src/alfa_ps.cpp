#include "alfa_ps.h"

AlfaPsCompressor::AlfaPsCompressor(string node_name,string node_type,vector<alfa_msg::ConfigMessage>* default_configurations):AlfaNode (node_name,node_type,default_configurations)
 {
    std::cout << "ALFA-Ps started" << std::endl;

    setSensorParameters();

    output_metrics.message_tag = "Ps-Compression performance";
    
    avg_exec_time_ri=0;
    avg_exec_time_png=0;
    avg_size_original=0;
    avg_size_png=0;
    total_points=0;
    fps=0;
    pps=0;
 }

void AlfaPsCompressor::setSensorParameters()
{
    std::cout << "Setting sensor parameters" << std::endl;

    sensor_parameters.angular_resolution_horizontal = (float) ( 0.2f * (M_PI/180.0f));
    sensor_parameters.angular_resolution_vertical = (float) ( 0.41875* (M_PI/180.0f));
    sensor_parameters.max_angle_width = (float) (360.0f * (M_PI/180.0f));
    sensor_parameters.max_angle_height = (float) (90.0f * (M_PI/180.0f));
    sensor_parameters.max_sensor_distance = 120;
}

void AlfaPsCompressor::process_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)
{   
    output_metrics.metrics.clear();

    static int counter=0;

    file_name="clouds/CompressedClouds/PNGS/rosbag_64_" + std::to_string(counter) + ".png";

    std::cout << counter+1 << " : " << input_cloud->size() << endl;

    auto start_ri = std::chrono::high_resolution_clock::now();
    range_image.createFromPointCloud(*input_cloud, sensor_parameters.angular_resolution_horizontal, sensor_parameters.angular_resolution_vertical, sensor_parameters.max_angle_width, sensor_parameters.max_angle_height,
                                     sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    auto stop_ri = std::chrono::high_resolution_clock::now();                                 
    auto duration_ri = std::chrono::duration_cast<std::chrono::milliseconds>(stop_ri - start_ri);

    //std::cout << range_image << "\n";

    float* ranges = range_image.getRangesArray();

    unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage (ranges, range_image.width, range_image.height, 0, sensor_parameters.max_sensor_distance, true);

    auto start_png = std::chrono::high_resolution_clock::now();
    pcl::io::saveRgbPNGFile(file_name, rgb_image, range_image.width, range_image.height);
    auto stop_png = std::chrono::high_resolution_clock::now();
    auto duration_png = std::chrono::duration_cast<std::chrono::milliseconds>(stop_png - start_png);

    calculate_metrics(input_cloud->size(), file_name, duration_ri.count(), duration_png.count(), counter+1);
    publish_metrics(output_metrics);

    if(counter==(NOF-1)){
        avg_metrics();
        counter=0;
    }
    free(ranges);
    counter++;
}

alfa_msg::AlfaConfigure::Response AlfaPsCompressor::process_config(alfa_msg::AlfaConfigure::Request &req)
{
    cout << "Updating Sensor Parameters" << endl;
    if(req.configurations.size()==5)
    {
        sensor_parameters.angular_resolution_horizontal = (float) (req.configurations[0].config * (M_PI/180.0f));
        sensor_parameters.angular_resolution_vertical = (float) (req.configurations[1].config * (M_PI/180.0f));
        sensor_parameters.max_angle_width = (float) (req.configurations[2].config * (M_PI/180.0f));
        sensor_parameters.max_angle_height = (float) (req.configurations[3].config * (M_PI/180.0f));
        sensor_parameters.max_sensor_distance = req.configurations[4].config;
        default_configurations[0][0].config = req.configurations[0].config;
        default_configurations[0][1].config = req.configurations[1].config;
        default_configurations[0][2].config = req.configurations[2].config;
        default_configurations[0][3].config = req.configurations[3].config;
        default_configurations[0][4].config = req.configurations[4].config;
    }
    alfa_msg::AlfaConfigure::Response response;
    response.return_status = 1;
    return response;
}

void AlfaPsCompressor::calculate_metrics(int cloud_size, string png_path, float duration_ri, float duration_png, int counter)
{

    std::ifstream file;
    float size_png = 0;
    float current_fps;
    float current_pps;

    file.open(png_path, std::ios::in | std::ios::binary | std::ios::ate );

    file.seekg(0, std::ios::end);
    size_png = file.tellg();

    float size_original = (static_cast<float> (cloud_size) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f)*1000;

    avg_exec_time_ri+=duration_ri;
    avg_exec_time_png+=duration_png;
    avg_size_original+=size_original/1000;
    avg_size_png+=size_png/1000;
    total_points+=cloud_size;
    current_fps=1000/(duration_png + duration_ri);
    current_pps=PPF*current_fps;

    // alfa metrics
    alfa_msg::MetricMessage new_message;

    new_message.metric = cloud_size;
    new_message.units = "Points ";
    new_message.metric_name = "Nº of Points in Point Cloud";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = size_original/1000;
    new_message.units = "kBs";
    new_message.metric_name = "Original Point Cloud Size";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = size_png/1000;
    new_message.units = "kBs";
    new_message.metric_name = "Compressed Size";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = size_original/size_png;
    new_message.units = "";
    new_message.metric_name = "Compression Ratio";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = duration_ri;
    new_message.units = "ms";
    new_message.metric_name = "Range Image processing time";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = duration_png;
    new_message.units = "ms";
    new_message.metric_name = "PNG processing time";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = duration_png + duration_ri;
    new_message.units = "ms";
    new_message.metric_name = "Total processing time";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = current_fps;
    new_message.units = "";
    new_message.metric_name = "Current FPS";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = current_pps;
    new_message.units = "";
    new_message.metric_name = "Points per second";
    output_metrics.metrics.push_back(new_message);

    if(counter==1 || counter==15 || counter==30 || counter==45 || counter==60 || counter==75 || counter==90){
        ROS_INFO("Range image processing time: %f\n", duration_ri);
        ROS_INFO("PNG processing time: %f\n", duration_png);
        ROS_INFO("Total processing time: %f\n", duration_png + duration_ri);
        ROS_INFO("Current FPS: %f\n", current_fps);
        ROS_INFO("Current PPS: %f\n", current_pps);
        ROS_INFO("Point Cloud size: %f\n", size_original/1000);
        ROS_INFO("Compressed size: %f\n", size_png/1000);
        ROS_INFO("Compression ratio: %f\n", size_original/size_png);
    }

}


void AlfaPsCompressor::avg_metrics()
{

    fps=1000/((avg_exec_time_png+avg_exec_time_ri)/NOF);
    pps=fps*PPF;
    ROS_INFO("---------------------------Ps-Compression finished---------------------------\n");
    ROS_INFO("Range image average processing time: %f\n", avg_exec_time_ri/NOF);
    ROS_INFO("PNG average processing time: %f\n", avg_exec_time_png/NOF);
    ROS_INFO("Total average processing time: %f\n", (avg_exec_time_png+avg_exec_time_ri)/NOF);
    ROS_INFO("Average FPS: %f\n", fps);
    ROS_INFO("Average PPS: %f\n", pps);
    ROS_INFO("Oringal point cloud size:\n");
    ROS_INFO("-------------------------PPF: %f points\n", total_points/NOF);
    ROS_INFO("-------------------------Total: %fkB\n", avg_size_original);
    ROS_INFO("-------------------------Average (per frame): %fkBs\n", avg_size_original/NOF);
    ROS_INFO("PNGs size:\n");
    ROS_INFO("-------------------------Total: %fkB\n", avg_size_png);
    ROS_INFO("-------------------------Average (per frame): %fkBs\n", avg_size_png/NOF);
    ROS_INFO("Average Compression Ratio: %f\n", avg_size_original/avg_size_png);

}