#include "ros/ros.h"
#include "alfa_ps.h"

#define NODE_NAME "alfa_ps"

#define NODE_TYPE "Storage data compression"


int main(int argc, char **argv)
{

     ros::init (argc, argv, "alfa_node");
       if (!ros::master::check()) {
           cout <<"Failed to inicialize ros"<<endl;
       }
       alfa_msg::ConfigMessage parameter1,parameter2,parameter3,parameter4,parameter5,parameter6;

       parameter1.config = 0.2;
       parameter1.config_name = "Horizontal Angular Resolution";

       parameter2.config = 0.41875;
       parameter2.config_name = "Vertical Angular Resolution";

       parameter3.config = 360;
       parameter3.config_name = "Maximum Width Angle";

       parameter4.config = 90;
       parameter4.config_name = "Maximum Height Angle";

       parameter5.config = 120;
       parameter5.config_name = "Sensor's maximum range";

      vector<alfa_msg::ConfigMessage> default_configurations;
      default_configurations.push_back(parameter1);
      default_configurations.push_back(parameter2);
      default_configurations.push_back(parameter3);
      default_configurations.push_back(parameter4);
      default_configurations.push_back(parameter5);

    AlfaPsCompressor new_node(NODE_NAME,NODE_TYPE,&default_configurations);
    while(ros::ok())
    {

    }
}
