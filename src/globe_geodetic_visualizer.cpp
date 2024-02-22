#include <globe_geodetic_visualizer/globe_geodetic_visualizer.h>

GlobeGeodeticVisualizer::GlobeGeodeticVisualizer(ros::NodeHandle* nh, double viz_division, double alt_scale) : nh_(nh), scale_division_(viz_division), alt_scale_(alt_scale)
{
    globe_pub_ = nh_->advertise<visualization_msgs::Marker>("globe_marker", 0);
    ego_pub_ = nh_->advertise<visualization_msgs::Marker>("ego_marker", 0);
    markers_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("markers", 0);

    ego_sub_ = nh_->subscribe("/ego", 1, &GlobeGeodeticVisualizer::ego_cb, this);

    // package_path_ = ros::package::getPath("globe_geodetic_visualizer");
    // meshes_path_ = package_path_ + "/meshes/";

    createGlobeMarker();
    createEgoMarker();
}

void GlobeGeodeticVisualizer::ego_cb(const geographic_msgs::GeoPoseStamped& msg)
{
    ego = msg;
}

geometry_msgs::Point GlobeGeodeticVisualizer::lla2ECEF(geographic_msgs::GeoPoint lla)
{   
    geometry_msgs::Point ecef;

    double clat = cos(D2R(lla.latitude));
    double slat = sin(D2R(lla.latitude));
    double clon = cos(D2R(lla.longitude));
    double slon = sin(D2R(lla.longitude));

    meter alt_meter = F2M(lla.altitude);

    ecef.x = (GLOBE_RADIUS + alt_meter*alt_scale_)/scale_division_ * clat * clon;
    ecef.y = (GLOBE_RADIUS + alt_meter*alt_scale_)/scale_division_ * clat * slon;
    ecef.z = (GLOBE_RADIUS + alt_meter*alt_scale_)/scale_division_ * slat;
    
    return ecef;
}

geometry_msgs::PoseStamped GlobeGeodeticVisualizer::geographic2geometryECEF(geographic_msgs::GeoPoseStamped geographic)
{
    geometry_msgs::PoseStamped geometry;

    geometry.header = geographic.header;
    geometry.pose.position = lla2ECEF(geographic.pose.position);
    geometry.pose.orientation = geographic.pose.orientation;
    
    return geometry;
}

void GlobeGeodeticVisualizer::createGlobeMarker()
{
    globe_marker.header.frame_id = "world";
    globe_marker.header.stamp = ros::Time();
    globe_marker.ns = "globe";
    globe_marker.id = 0;
    globe_marker.type = visualization_msgs::Marker::SPHERE;
    // globe_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    // globe.mesh_resource = "package://globe_geodetic_visualizer/meshes/globe.dae";
    globe_marker.action = visualization_msgs::Marker::ADD;
    globe_marker.pose.position.x = 0;
    globe_marker.pose.position.y = 0;
    globe_marker.pose.position.z = 0;
    globe_marker.pose.orientation.x = 0.0;
    globe_marker.pose.orientation.y = 0.0;
    globe_marker.pose.orientation.z = 0.0;
    globe_marker.pose.orientation.w = 1.0;
    globe_marker.scale.x = GLOBE_RADIUS * 2 / scale_division_; // Diameter in X
    globe_marker.scale.y = GLOBE_RADIUS * 2 / scale_division_; // Diameter in Y
    globe_marker.scale.z = GLOBE_RADIUS * 2 / scale_division_; // Diameter in Z
    globe_marker.color.r = 0.0;
    globe_marker.color.g = 0.5;
    globe_marker.color.b = 1.0; // Blue globe_marker
    globe_marker.color.a = 1.0; // Don't forget to set the alpha!
    globe_marker.lifetime = ros::Duration();
}

void GlobeGeodeticVisualizer::createEgoMarker()
{
    ego_marker.header.frame_id = "world";
    ego_marker.header.stamp = ros::Time();
    ego_marker.ns = "ego_marker";
    ego_marker.id = 1;
    ego_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    ego_marker.mesh_resource = "package://globe_geodetic_visualizer/meshes/Cessna_172.stl";
    ego_scale_ = 10000.0/scale_division_;
    ego_marker.action = visualization_msgs::Marker::ADD;
    ego_marker.pose.position.x = 0;
    ego_marker.pose.position.y = 0;
    ego_marker.pose.position.z = 0;
    ego_marker.pose.orientation.x = 0.0;
    ego_marker.pose.orientation.y = 0.0;
    ego_marker.pose.orientation.z = 0.0;
    ego_marker.pose.orientation.w = 1.0;
    ego_marker.scale.x = ego_scale_; // Diameter in X
    ego_marker.scale.y = ego_scale_; // Diameter in Y
    ego_marker.scale.z = ego_scale_; // Diameter in Z
    ego_marker.color.r = 1.0;
    ego_marker.color.g = 1.0;
    ego_marker.color.b = 1.0; // White ego_marker
    ego_marker.color.a = 1.0; // Don't forget to set the alpha!
    ego_marker.lifetime = ros::Duration();
}

void GlobeGeodeticVisualizer::addMarkers(visualization_msgs::Marker marker)
{
    visualizations.markers.push_back(marker);
}

void GlobeGeodeticVisualizer::visualizeMarker()
{
    globe_pub_.publish(globe_marker);

    geometry_msgs::PoseStamped ego_ecef = geographic2geometryECEF(ego);
    ego_marker.header.stamp = ego_ecef.header.stamp;
    ego_marker.pose = ego_ecef.pose;
    ego_pub_.publish(ego_marker);

    markers_pub_.publish(visualizations);
}