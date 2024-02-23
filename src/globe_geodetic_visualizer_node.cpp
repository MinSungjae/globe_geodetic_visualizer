#include <globe_geodetic_visualizer/globe_geodetic_visualizer.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "globe_geodetic_visualizer_node");
    ros::NodeHandle nh;

    ros::Rate rate(10);

    double scale_division = 1000000.0;
    double altitude_scale = 1000.0;

    GlobeGeodeticVisualizer visualizer(&nh, scale_division, altitude_scale);

    while(ros::ok())
    {
        if(!visualizer.visualizeMarker())
            ROS_WARN("/ego not yet received...");
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}