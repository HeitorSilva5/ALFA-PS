#include "alfa_ps.h"

AlfaPsCompressor::AlfaPsCompressor(string node_name,string node_type,vector<alfa_msg::ConfigMessage>* default_configurations):AlfaNode (node_name,node_type,default_configurations)
 {
    std::cout << "ALFA-Ps started" << std::endl;

    unsigned int region_size = 0x10000;
    off_t axi_pbase = 0xA0000000;
    int fd;
    unsigned int ddr_size = 0x200000;
    off_t ddr_ptr_base = 0x0F000000; // physical base address
    //Map the physical address into user space getting a virtual address for it
    hw=0;

    if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) != -1) {
      ddr_pointer = (u64 *)mmap(NULL, ddr_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, ddr_ptr_base);
      hw32_vptr = (u_int32_t *)mmap(NULL, region_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, axi_pbase);
      hw=1;
    }
    else
    ROS_INFO("NAO ENTROU NO NMAP :(");


    setSensorParameters();
    

    output_metrics.message_tag = "Ps-Compression performance";
    
    avg_exec_time_ri = 0;
    avg_exec_time_png = 0;
    avg_size_original = 0;
    avg_size_png = 0;
    total_points = 0;
    points_per_second = 0;
    frames_per_second = 0;
    
    NOF=76;

    compression_lvl=1;
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(compression_lvl);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);

    if(hw)
    {
      vector<int32_t> configs;
      configs.push_back(0);
      configs.push_back(0);
      configs.push_back(0);
      //configs.push_back(sensor_parameters.angular_resolution_horizontal*100);                                           //d_azimuth
      configs.push_back(0);
      //configs.push_back(sensor_parameters.angular_resolution_vertical*100);                                             //d_elevation               //hdl_64 -> 46.6
      configs.push_back(3);
      //configs.push_back((sensor_parameters.min_vertical_angle)*100);                                                    //min_vert_angle
      configs.push_back(2);
      //configs.push_back(sensor_parameters.sensor_tag);                                                                  //n_lines
      configs.push_back(2);
      //configs.push_back(sensor_parameters.n_columns);                                                                   //n_columns
      configs.push_back(0);
      if(sensor_parameters.max_sensor_distance==100)                                                                    //LUT input selector
        configs.push_back(0);                                                                                           
      else if(sensor_parameters.max_sensor_distance==80)
        configs.push_back(1); 
      write_hardware_registers(configs, hw32_vptr, 3);
    }

 }

void AlfaPsCompressor::setSensorParameters()
{
    std::cout << "Setting sensor parameters" << std::endl;

    sensor_parameters.sensor_tag = 64;
    sensor_parameters.angular_resolution_horizontal = 0.2f;
    sensor_parameters.angular_resolution_horizontal_rads = (float) ( sensor_parameters.angular_resolution_horizontal * (M_PI/180.0f));
    sensor_parameters.angular_resolution_vertical = (float) ( 0.46666f );          //hdl64 -> 0.46666
    sensor_parameters.angular_resolution_vertical_rads = (float) ( sensor_parameters.angular_resolution_vertical * (M_PI/180.0f));
    sensor_parameters.min_vertical_angle = -24.8;                                                 //hdl64 -> -24.8
    sensor_parameters.max_angle_width = (float) (360.0f * (M_PI/180.0f));
    sensor_parameters.max_angle_height = (float) (90.0f * (M_PI/180.0f));
    sensor_parameters.max_sensor_distance = 80;
    sensor_parameters.n_columns = 1800;
}

void AlfaPsCompressor::process_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)
{   
    output_metrics.metrics.clear();
    static int counter=0;

    if(hw)
    {
      //store point cloud
      auto start_store_hw = std::chrono::high_resolution_clock::now();
      store_pointcloud_hardware(input_cloud,ddr_pointer);
      auto stop_store_hw = std::chrono::high_resolution_clock::now();
      usleep(10);       //VERIFICAR!!!

      //create range image
      auto start_RI_hw = std::chrono::high_resolution_clock::now();
      vector<int32_t> configs;
      configs.push_back(1);
      configs.push_back(input_cloud->size());
      write_hardware_registers(configs, hw32_vptr);
      int hardware_finish = 0;
      int value = 0;
      while(!hardware_finish){
        vector<uint32_t> hardware_result = read_hardware_registers(hw32_vptr, 3);
        value = hardware_result[2];
        if(value==1)
          hardware_finish = 1;
        else
          usleep(1);
      }
      auto stop_RI_hw = std::chrono::high_resolution_clock::now();

      // read range image
      auto start_read_hw = std::chrono::high_resolution_clock::now();
      unsigned char * rgb_image_hw = read_hardware_pointcloud(ddr_pointer, sensor_parameters.n_columns*sensor_parameters.sensor_tag*2);
      auto stop_read_hw = std::chrono::high_resolution_clock::now();

      //PNG
      file_name_hw="./clouds/CompressedClouds/PNGS/rosbag_" + std::to_string(sensor_parameters.sensor_tag*2) + "_" + std::to_string(counter) + "_hw.png";
      auto start_png_hw = std::chrono::high_resolution_clock::now();
      image = cv::Mat(sensor_parameters.sensor_tag*2, sensor_parameters.n_columns, CV_8UC3, static_cast<void*> (rgb_image_hw));
      cv::imwrite(file_name_hw, image, compression_params);
      auto stop_png_hw = std::chrono::high_resolution_clock::now();

      auto duration_store_hw = std::chrono::duration_cast<std::chrono::milliseconds>(stop_store_hw - start_store_hw);
      auto duration_RI_hw = std::chrono::duration_cast<std::chrono::microseconds>(stop_RI_hw - start_RI_hw);
      auto duration_read_hw = std::chrono::duration_cast<std::chrono::microseconds>(stop_read_hw - start_read_hw);
      auto duration_png_hw = std::chrono::duration_cast<std::chrono::milliseconds>(stop_png_hw - start_png_hw);
      cout << "STORE TIME:" << duration_store_hw.count() << "ms" << endl;
      cout << "RANGE IMAGE TOOK:" << duration_RI_hw.count() << "us" << endl;
      cout << "READ TIME:" << duration_read_hw.count() << "us" << endl;
      cout << "PNG TIME:" << duration_png_hw.count() << "ms" << endl;
    }

    // static float max_range=0;
    // for (auto point :*input_cloud) {
    //   float range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    //   if(range>max_range)
    //     max_range = range;
    // }
    // std::cout << counter << ": Max Range" << max_range << endl;

    // float max_elevation=0, min_elevation=0, max_azimuth=0, min_azimuth = 20;
    // int cnt_above=0, cnt_below=0;
    // static float top_azimuth = 0, bot_azimuth=20;
    // for (auto point :*input_cloud) {
    //   float elevation = (float) ((std::atan2(point.z, std::hypot(point.x, point.y)))* (180.0f/M_PI)) *100;
    //   const auto a = std::atan2(point.y, point.x);
    //   float azimuth = (float) ((point.y >= 0 ? a : a + M_PI * 2) * (180.0f/M_PI)) *100;
    //   if(azimuth>max_azimuth)
    //     max_azimuth=azimuth;
    //   else if(azimuth<min_azimuth && azimuth >= 0)
    //     min_azimuth=azimuth;
    //   if(elevation>max_elevation)
    //     max_elevation=elevation;
    //   else if(elevation<min_elevation)
    //     min_elevation=elevation;
    //   if(azimuth > 35980)
    //     cnt_above++;
    //   else if(azimuth < 10 && azimuth >= 0)
    //     cnt_below++;
    //   if(azimuth>top_azimuth)
    //     top_azimuth=azimuth;
    //   else if(azimuth<bot_azimuth && azimuth >= 0)
    //     bot_azimuth=azimuth;
    // }

    // float range, range_n;
    // for(int cnt =0 ; cnt < 20 ; cnt ++){
    //   auto point = input_cloud->points[cnt];
    //   range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    //   range_n = range * 2.125;
    //   uint16_t a16_range = lrint(range_n);
    //   std::cout << cnt << " range: " << range << " | normalized rangef: " << range_n << " | normalized range: " << a16_range << endl;
    // }


    // std::cout << counter + 1 << "=> Azimuth max: " << max_azimuth << " | Azimuth min: " << min_azimuth << endl;
    // std::cout << "TOP AZIMUTH: " << top_azimuth << " | BOTTOM AZIMUTH: " << bot_azimuth << endl;
    // std::cout << "Above 35980: " << cnt_above << " | Below 20: " << cnt_below << endl;

    file_name="./clouds/CompressedClouds/PNGS/rosbag_" + std::to_string(sensor_parameters.sensor_tag) + "_" + std::to_string(counter) + ".png";
    // string file_name1 = "clouds/pointclouds/savedClouds/64/new_ascii_" + std::to_string(counter) + ".pcd";

    std::cout << counter+1 << " - " << input_cloud->size() << endl;

    auto start_ri = std::chrono::high_resolution_clock::now();
    range_image.createFromPointCloud(*input_cloud, sensor_parameters.angular_resolution_horizontal_rads, sensor_parameters.angular_resolution_vertical_rads, sensor_parameters.max_angle_width, sensor_parameters.max_angle_height,
                                     sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    auto stop_ri = std::chrono::high_resolution_clock::now();                                 
    auto duration_ri = std::chrono::duration_cast<std::chrono::milliseconds>(stop_ri - start_ri);

    //std::cout << range_image << "\n";

    float* ranges = range_image.getRangesArray();

    unsigned char* rgb_image = getVisualImage(ranges, range_image.width, range_image.height, 0, sensor_parameters.max_sensor_distance, true);

    // std::cout << range_image.width << " | " << range_image.height << endl;

    auto start_png = std::chrono::high_resolution_clock::now();
    //pcl::io::saveRgbPNGFile(file_name, rgb_image, range_image.width, range_image.height);
    image = cv::Mat(range_image.height, range_image.width, CV_8UC3, static_cast<void*> (rgb_image));
    //cv::imwrite(file_name, image, compression_params);
    auto stop_png = std::chrono::high_resolution_clock::now();
    auto duration_png = std::chrono::duration_cast<std::chrono::milliseconds>(stop_png - start_png);

    calculate_metrics(input_cloud->size(), file_name, duration_ri.count(), duration_png.count(), counter+1);
    publish_metrics(output_metrics);

    free(ranges);
    free(rgb_image);

    // pcl::io::savePCDFileASCII(file_name1, *input_cloud);

    if(counter == (NOF-1)){
        avg_metrics();
        counter = 0;
        avg_exec_time_ri = 0;
        avg_exec_time_png = 0;
        avg_size_original = 0;
        avg_size_png = 0;
        total_points = 0;
        points_per_second = 0;
        frames_per_second = 0; 
        return;
    }

    counter++;
}

unsigned char* AlfaPsCompressor::getVisualImage (const float* float_image, int width, int height, float min_value, float max_value, bool gray_scale) 
{
  //MEASURE_FUNCTION_TIME;
  
  //std::cout << "Image is of size "<<width<<"x"<<height<<"\n";
  int size = width*height;
  int arraySize = 3 * size;
  unsigned char* data = new unsigned char[arraySize];
  unsigned char* dataPtr = data;
  
  bool recalculateMinValue = std::isinf (min_value),
       recalculateMaxValue = std::isinf (max_value);
  if (recalculateMinValue) 
    min_value = std::numeric_limits<float>::infinity ();
  if (recalculateMaxValue) 
    max_value = -std::numeric_limits<float>::infinity ();
  
  if (recalculateMinValue || recalculateMaxValue) 
  {
    for (int i=0; i<size; ++i) 
    {
      float value = float_image[i];
      if (!std::isfinite(value)) continue;
      if (recalculateMinValue) 
        min_value = (std::min)(min_value, value);
      if (recalculateMaxValue) 
        max_value = (std::max)(max_value, value);
    }
  }
  //std::cout << "min_value is "<<min_value<<" and max_value is "<<max_value<<".\n";
  float factor = 1.0f / (max_value-min_value), offset = -min_value;
  
  for (int i=0; i<size; ++i) 
  {
    unsigned char& r=*(dataPtr++), & g=*(dataPtr++), & b=*(dataPtr++);
    float value = float_image[i];
    
    if (!std::isfinite(value)) 
    {
      //getColorForFloat(value, r, g, b);
      r = g = b = 0;
      continue;
    }
    
    // Normalize value to [0, 1]
    value = std::max (0.0f, std::min (1.0f, factor * (value + offset)));
    
    // Get a color from the value in [0, 1]
    if (gray_scale) 
    {
      r = g = b = static_cast<unsigned char> (pcl_lrint (value * 255));
    }
    else 
    {
      getColorForFloat(value, r, g, b);
    }
    //std::cout << "Setting pixel "<<i<<" to "<<(int)r<<", "<<(int)g<<", "<<(int)b<<".\n";
  }
  
  return data;
}

void AlfaPsCompressor::getColorForFloat (float value, unsigned char& r, unsigned char& g, unsigned char& b) 
{
  if (std::isinf (value)) 
  {
    if (value > 0.0f) 
    {
      r = 150;  g = 150;  b = 200;  // INFINITY
      return;
    }
    r = 150;  g = 200;  b = 150;  // -INFINITY
    return;
  }
  if (!std::isfinite (value)) 
  {
    r = 200;  g = 150;  b = 150;  // -INFINITY
    return;
  }
  
  r = g = b = 0;
  value *= 10;
  if (value <= 1.0) 
  {  // black -> purple
    b = static_cast<unsigned char> (pcl_lrint(value*200));
    r = static_cast<unsigned char> (pcl_lrint(value*120));
  }
  else if (value <= 2.0) 
  {  // purple -> blue
    b = static_cast<unsigned char> (200 + pcl_lrint((value-1.0)*55));
    r = static_cast<unsigned char> (120 - pcl_lrint((value-1.0)*120));
  }
  else if (value <= 3.0) 
  {  // blue -> turquoise
    b = static_cast<unsigned char> (255 - pcl_lrint((value-2.0)*55));
    g = static_cast<unsigned char> (pcl_lrint((value-2.0)*200));
  }
  else if (value <= 4.0) 
  {  // turquoise -> green
    b = static_cast<unsigned char> (200 - pcl_lrint((value-3.0)*200));
    g = static_cast<unsigned char> (200 + pcl_lrint((value-3.0)*55));
  }
  else if (value <= 5.0) 
  {  // green -> greyish green
    g = static_cast<unsigned char> (255 - pcl_lrint((value-4.0)*100));
    r = static_cast<unsigned char> (pcl_lrint((value-4.0)*120));
  }
  else if (value <= 6.0) 
  { // greyish green -> red
    r = static_cast<unsigned char> (100 + pcl_lrint((value-5.0)*155));
    g = static_cast<unsigned char> (120 - pcl_lrint((value-5.0)*120));
    b = static_cast<unsigned char> (120 - pcl_lrint((value-5.0)*120));
  }
  else if (value <= 7.0) 
  {  // red -> yellow
    r = 255;
    g = static_cast<unsigned char> (pcl_lrint((value-6.0)*255));
  }
  else 
  {  // yellow -> white
    r = 255;
    g = 255;
    b = static_cast<unsigned char> (pcl_lrint((value-7.0)*255.0/3.0));
  }
}

alfa_msg::AlfaConfigure::Response AlfaPsCompressor::process_config(alfa_msg::AlfaConfigure::Request &req)
{
    cout << "Updating configuration Parameters" << endl;
    if(req.configurations.size()==10)
    {
        sensor_parameters.sensor_tag = req.configurations[0].config;
        sensor_parameters.angular_resolution_horizontal = (float) (req.configurations[1].config);
        sensor_parameters.angular_resolution_horizontal_rads = (float) (sensor_parameters.angular_resolution_horizontal * (M_PI/180.0f));
        sensor_parameters.angular_resolution_vertical = (float) (req.configurations[2].config);
        sensor_parameters.angular_resolution_vertical_rads = (float) (sensor_parameters.angular_resolution_vertical * (M_PI/180.0f));
        sensor_parameters.max_angle_width = (float) (req.configurations[3].config * (M_PI/180.0f));
        sensor_parameters.max_angle_height = (float) (req.configurations[4].config * (M_PI/180.0f));
        sensor_parameters.max_sensor_distance = req.configurations[5].config;
        sensor_parameters.min_vertical_angle = req.configurations[6].config;
        sensor_parameters.n_columns = req.configurations[7].config;
        NOF = req.configurations[8].config;
        compression_params.clear();
        if(req.configurations[9].config==10)
        {
          compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
          compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
        }
        else if(req.configurations[9].config==11)
        {
          compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
          compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
        }
        else
        {
          compression_lvl = req.configurations[9].config;
          compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
          compression_params.push_back(compression_lvl);
        }
        default_configurations[0][0].config = req.configurations[0].config;
        default_configurations[0][1].config = req.configurations[1].config;
        default_configurations[0][2].config = req.configurations[2].config;
        default_configurations[0][3].config = req.configurations[3].config;
        default_configurations[0][4].config = req.configurations[4].config;
        default_configurations[0][5].config = req.configurations[5].config;
        default_configurations[0][6].config = req.configurations[6].config;
        default_configurations[0][7].config = req.configurations[7].config;
        default_configurations[0][8].config = req.configurations[8].config;
        default_configurations[0][9].config = req.configurations[9].config;

        if(hw)
        {
          vector<int32_t> configs;
          configs.push_back(0);
          configs.push_back(0);
          configs.push_back(0);
          configs.push_back(sensor_parameters.angular_resolution_horizontal*100);                                           //d_azimuth
          configs.push_back(sensor_parameters.angular_resolution_vertical*100);                                             //d_elevation               //hdl_64 -> 46.6
          configs.push_back((sensor_parameters.min_vertical_angle)*100);                                                    //min_vert_angle
          configs.push_back(sensor_parameters.sensor_tag);                                                                  //n_lines
          configs.push_back(sensor_parameters.n_columns);                                                                   //n_columns
          if(sensor_parameters.max_sensor_distance==100)                                                                    //LUT input selector
            configs.push_back(0);                                                                                           
          else if(sensor_parameters.max_sensor_distance==120)
            configs.push_back(1);                                                       
          write_hardware_registers(configs, hw32_vptr, 3);
        }

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

    avg_exec_time_ri += duration_ri;
    avg_exec_time_png += duration_png;
    avg_size_original += size_original/1000;
    avg_size_png += size_png/1000;
    total_points += cloud_size;

    current_pps = (1000*cloud_size)/(duration_png + duration_ri);
    current_fps = 1000/(duration_png + duration_ri);

    points_per_second += current_pps;
    frames_per_second += current_fps;

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
    new_message.metric_name = "Total compression time";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = current_fps;
    new_message.units = "";
    new_message.metric_name = "Current FPS";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = current_pps;
    new_message.units = "";
    new_message.metric_name = "Points per second";
    output_metrics.metrics.push_back(new_message);

    // if(counter==1 || counter==15 || counter==30 || counter==45 || counter==60 || counter==75 || counter==90){
    //     ROS_INFO("Frame number %d\n", counter);
    //     ROS_INFO("Range image processing time: %f\n", duration_ri);
    //     ROS_INFO("PNG processing time: %f\n", duration_png);
    //     ROS_INFO("Total processing time: %f\n", duration_png + duration_ri);
    //     ROS_INFO("Current FPS: %f\n", current_fps);
    //     ROS_INFO("Current PPS: %f\n", current_pps);
    //     ROS_INFO("Point Cloud size: %f\n", size_original/1000);
    //     ROS_INFO("Compressed size: %f\n", size_png/1000);
    //     ROS_INFO("Compression ratio: %f\n", size_original/size_png);
    // }

}


void AlfaPsCompressor::avg_metrics()
{

    ROS_INFO("---------------------------Ps-Compression finished---------------------------\n");
    ROS_INFO("Range image average processing time: %f\n", avg_exec_time_ri/NOF);
    ROS_INFO("PNG average processing time: %f\n", avg_exec_time_png/NOF);
    ROS_INFO("Total average processing time: %f\n", (avg_exec_time_png+avg_exec_time_ri)/NOF);
    ROS_INFO("Average FPS NEW: %f\n", frames_per_second/NOF);
    ROS_INFO("Average PPS: %f\n", points_per_second/NOF);
    ROS_INFO("Oringal point cloud size:\n");
    ROS_INFO("-------------------------PPF: %f points\n", total_points/NOF);
    ROS_INFO("-------------------------Total: %fkB\n", avg_size_original);
    ROS_INFO("-------------------------Average (per frame): %fkBs\n", avg_size_original/NOF);
    ROS_INFO("PNGs size:\n");
    ROS_INFO("-------------------------Total: %fkB\n", avg_size_png);
    ROS_INFO("-------------------------Average (per frame): %fkBs\n", avg_size_png/NOF);
    ROS_INFO("Average Compression Ratio: %f\n", avg_size_original/avg_size_png);

}