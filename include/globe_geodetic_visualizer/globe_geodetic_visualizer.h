#include <ros/ros.h>
#include <ros/package.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <globe_geodetic_visualizer/GeoTraj.h>

#define PI              3.14159265357989
#define D2R(DEG)        DEG*PI/180.0
#define R2D(RAD)        RAD*180.0/PI
#define F2M(FEET)       FEET*3.28084
#define M2F(METER)      METER*0.3048
#define GLOBE_RADIUS    63781000.0
#define TRAJ_WIDTH      0.01

typedef double meter, feet;
typedef double rad, deg;

class GlobeGeodeticVisualizer
{
private:
    ros::NodeHandle* nh_;
    ros::Publisher globe_pub_, ego_pub_, waypoint_pub_, markers_pub_, land_pub_, traj_pub_, lookah_pub_;
    ros::Publisher world_pub_, axis_pub_;
    ros::Subscriber ego_sub_, waypoint_sub_, traj_sub_, lookah_sub_;

    tf2_ros::TransformBroadcaster ego_br_;
    geometry_msgs::TransformStamped ego_transform;

    std::string package_path_, meshes_path_;

    visualization_msgs::Marker globe_marker, ego_marker, waypoint_marker, land_marker, traj_marker, lookah_marker;
    geographic_msgs::GeoPoseStamped ego;
    geographic_msgs::GeoPoint lookah, waypoint;
    globe_geodetic_visualizer::GeoTraj traj;
    double scale_division_;
    double ego_scale_;
    double alt_scale_;

    tf2::Quaternion ego_marker_correction;

    visualization_msgs::MarkerArray visualizations;

    geometry_msgs::Point lla2ECEF(geographic_msgs::GeoPoint lla);

    void createGlobeMarker();
    //void createLandMarker();
    void createEgoMarker();
    void createWaypointMarker();
    void createTrajMarker();
    void createEgoTransform();
    void createLookAHMarker();

    void ego_geographic_cb(const geographic_msgs::GeoPoseStamped& msg);
    void waypoint_geographic_cb(const geographic_msgs::GeoPoint& msg);
    void traj_geographic_cb(const globe_geodetic_visualizer::GeoTraj& msg);
    void lookah_geographic_cb(const geographic_msgs::GeoPoint& msg);
    geometry_msgs::PoseStamped geographic2geometryECEF(geographic_msgs::GeoPoseStamped geographic);
    geometry_msgs::Pose geographic2geometryECEF(geographic_msgs::GeoPose geographic);
    geometry_msgs::Point geographic2geometryECEF(geographic_msgs::GeoPoint geographic);

public:
    GlobeGeodeticVisualizer(ros::NodeHandle* nh, meter viz_division, double alt_scale);

    bool visualizeMarker();
    
    void addMarkers(visualization_msgs::Marker marker);
};
