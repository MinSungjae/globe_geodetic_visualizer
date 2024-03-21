#include <globe_geodetic_visualizer/globe_geodetic_visualizer.h>

GlobeGeodeticVisualizer::GlobeGeodeticVisualizer(ros::NodeHandle* nh, double viz_division, double alt_scale) : nh_(nh), scale_division_(viz_division), alt_scale_(alt_scale)
{
    globe_pub_ = nh_->advertise<visualization_msgs::Marker>("globe_marker", 1);
    land_pub_ = nh_->advertise<visualization_msgs::Marker>("land_marker", 1);
    ego_pub_ = nh_->advertise<visualization_msgs::Marker>("ego_marker", 1);
    lookah_pub_ = nh_->advertise<visualization_msgs::Marker>("lookah_marker", 1);
    waypoint_pub_ = nh_->advertise<visualization_msgs::Marker>("waypiont", 1);
    traj_pub_ = nh_->advertise<visualization_msgs::Marker>("traj_marker", 1);
    markers_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("markers", 1);
    axis_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("test_axis", 1);
    world_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("world_axis", 1);

    ego_sub_ = nh_->subscribe("/ego", 1, &GlobeGeodeticVisualizer::ego_geographic_cb, this);
    waypoint_sub_ = nh_->subscribe("/waypoint", 1, &GlobeGeodeticVisualizer::waypoint_geographic_cb, this);
    traj_sub_ = nh_->subscribe("/traj", 1, &GlobeGeodeticVisualizer::traj_geographic_cb, this);
    lookah_sub_ = nh_->subscribe("/lookah", 1, &GlobeGeodeticVisualizer::lookah_geographic_cb, this);

    // package_path_ = ros::package::getPath("globe_geodetic_visualizer");
    // meshes_path_ = package_path_ + "/meshes/";

    createGlobeMarker();
    //createLandMarker();
    createEgoMarker();
    createWaypointMarker();
    createTrajMarker();
    createLookAHMarker();
    
    createEgoTransform();
}

void GlobeGeodeticVisualizer::ego_geographic_cb(const geographic_msgs::GeoPoseStamped& msg)
{
    ego = msg;
    // Delete after ego publisher
    ego.header.stamp = ros::Time::now();
}

void GlobeGeodeticVisualizer::waypoint_geographic_cb(const geographic_msgs::GeoPoint& msg)
{
    waypoint = msg;
}

void GlobeGeodeticVisualizer::traj_geographic_cb(const globe_geodetic_visualizer::GeoTraj& msg)
{
    traj = msg;
}

void GlobeGeodeticVisualizer::lookah_geographic_cb(const geographic_msgs::GeoPoint& msg)
{
    lookah = msg;
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

    geometry.pose = geographic2geometryECEF(geographic.pose);

    return geometry;
}

geometry_msgs::Pose GlobeGeodeticVisualizer::geographic2geometryECEF(geographic_msgs::GeoPose geographic)
{
    geometry_msgs::Pose geometry;

    geometry.position = lla2ECEF(geographic.position);

    geometry.orientation = geographic.orientation;
    tf2::Transform geometry_tf, geocentric;
    tf2::fromMsg(geometry, geometry_tf);

    tf2::Quaternion ecef2ned;
    ecef2ned.setEulerZYX(0.0, D2R(-90.0), 0.0);
    
    tf2::Quaternion geocentric_quat;
    geocentric_quat.setEulerZYX(D2R(geographic.position.longitude), -D2R(geographic.position.latitude), 0.0);
    geometry_tf.setRotation(geocentric_quat*ecef2ned*geometry_tf.getRotation()*ecef2ned.inverse());

    geometry.orientation = tf2::toMsg(geometry_tf.getRotation());
    return geometry;
}

geometry_msgs::Point GlobeGeodeticVisualizer::geographic2geometryECEF(geographic_msgs::GeoPoint geographic)
{
    geometry_msgs::Point geometry;

    geometry = lla2ECEF(geographic);

    return geometry;
}

void GlobeGeodeticVisualizer::createGlobeMarker()
{
    globe_marker.header.frame_id = "world";
    globe_marker.header.stamp = ros::Time();
    globe_marker.ns = "globe";
    globe_marker.id = 0;
    globe_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    globe_marker.mesh_resource = "package://globe_geodetic_visualizer/meshes/Globe.obj";
    globe_marker.mesh_use_embedded_materials = true;
    globe_marker.action = visualization_msgs::Marker::ADD;
    globe_marker.pose.position.x = 0;
    globe_marker.pose.position.y = 0;
    globe_marker.pose.position.z = 0;
    tf2::Quaternion land_correction;
    land_correction.setEulerZYX(D2R(180.0), D2R(0.0), D2R(90.0));
    globe_marker.pose.orientation = tf2::toMsg(land_correction);
    globe_marker.scale.x = 0.00012725;
    globe_marker.scale.y = 0.00012725;
    globe_marker.scale.z = 0.00012725;
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
    ego_scale_ = 1500.0/scale_division_;
    ego_marker.action = visualization_msgs::Marker::ADD;
    ego_marker.pose.position.x = 0;
    ego_marker.pose.position.y = 0;
    ego_marker.pose.position.z = 0;
    ego_marker_correction.setEulerZYX(D2R(0.0), D2R(90.0), D2R(0.0));
    ego_marker.pose.orientation = tf2::toMsg(ego_marker_correction);
    ego_marker.scale.x = ego_scale_; // Diameter in X
    ego_marker.scale.y = ego_scale_; // Diameter in Y
    ego_marker.scale.z = ego_scale_; // Diameter in Z
    ego_marker.color.r = 1.0;
    ego_marker.color.g = 1.0;
    ego_marker.color.b = 1.0; // White ego_marker
    ego_marker.color.a = 1.0; // Don't forget to set the alpha!
    ego_marker.lifetime = ros::Duration();
}

void GlobeGeodeticVisualizer::createWaypointMarker()
{
    waypoint_marker.header.frame_id = "world";
    waypoint_marker.header.stamp = ros::Time();
    waypoint_marker.ns = "waypoint_marker";
    waypoint_marker.id = 1;
    waypoint_marker.type = visualization_msgs::Marker::SPHERE;
    waypoint_marker.action = visualization_msgs::Marker::ADD;
    waypoint_marker.pose.orientation.w = 1.0;
    waypoint_marker.scale.x = 0.05; // Diameter in X
    waypoint_marker.scale.y = 0.05; // Diameter in Y
    waypoint_marker.scale.z = 0.05; // Diameter in Z
    waypoint_marker.color.r = 1.0;
    waypoint_marker.color.g = 0.0;
    waypoint_marker.color.b = 0.0; // White waypoint_marker
    waypoint_marker.color.a = 1.0; // Don't forget to set the alpha!
    waypoint_marker.lifetime = ros::Duration();
}

void GlobeGeodeticVisualizer::createTrajMarker()
{
    traj_marker.header.frame_id = "world";
    traj_marker.ns = "traj";
    traj_marker.id = 1;
    traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
    traj_marker.action = visualization_msgs::Marker::ADD;
    traj_marker.scale.x = TRAJ_WIDTH;
    traj_marker.lifetime = ros::Duration();

    // geographic_msgs::GeoPoint point;
    // point.latitude = 36.0;
    // point.longitude = 127.0;
    // point.altitude = 61.0;
    // traj.points.push_back(point);

    // point.latitude = 36.5;
    // point.longitude = 127.0;
    // point.altitude = 1000.0;
    // traj.points.push_back(point);

    // point.latitude = 46.0;
    // point.longitude = 127.0;
    // point.altitude = 2500.0;
    // traj.points.push_back(point);

    // point.latitude = 51.0;
    // point.longitude = 129.0;
    // point.altitude = 3500.0;
    // traj.points.push_back(point);
}

void GlobeGeodeticVisualizer::createEgoTransform()
{
    ego_transform.header.frame_id = "world";
    ego_transform.child_frame_id = "ego";
}

void GlobeGeodeticVisualizer::createLookAHMarker()
{
    lookah_marker.header.frame_id = "world";
    lookah_marker.header.stamp = ros::Time();
    lookah_marker.ns = "lookah_marker";
    lookah_marker.id = 1;
    lookah_marker.type = visualization_msgs::Marker::ARROW;
    lookah_marker.action = visualization_msgs::Marker::ADD;
    lookah_marker.pose.orientation.w = 1.0;
    lookah_marker.scale.x = 0.01; // Diameter in X
    lookah_marker.scale.y = 0.02; // Diameter in Y
    lookah_marker.scale.z = 0.0; // Diameter in Z
    lookah_marker.color.r = 1.0;
    lookah_marker.color.g = 0.0;
    lookah_marker.color.b = 1.0; // White lookah_marker
    lookah_marker.color.a = 1.0; // Don't forget to set the alpha!
    lookah_marker.lifetime = ros::Duration();
}

/** NOTE!!! Here, marker.pose.position is Geodetic coordinates.
 * @param marker.pose.position. {X: Latitude, Y: Longitude, Z: Altitude}.
*/
void GlobeGeodeticVisualizer::addMarkers(visualization_msgs::Marker marker)
{
    geographic_msgs::GeoPoseStamped geographic_pose;
    geographic_pose.pose.position.latitude = marker.pose.position.x;
    geographic_pose.pose.position.longitude = marker.pose.position.y;
    geographic_pose.pose.position.altitude = marker.pose.position.z;
    geographic_pose.pose.orientation = marker.pose.orientation;

    geometry_msgs::PoseStamped marker_ecef = geographic2geometryECEF(geographic_pose);
    marker.pose = marker_ecef.pose;

    visualizations.markers.push_back(marker);
}

bool GlobeGeodeticVisualizer::visualizeMarker()
{
    if(ego.header.stamp < ros::Time::now() - ros::Duration(1.0))
        return false;
    geometry_msgs::PoseStamped world;
    world.header.frame_id = "world";
    world.pose.orientation.w = 1.0;
    world_pub_.publish(world);
    globe_pub_.publish(globe_marker);
    land_pub_.publish(land_marker);

    // Draw ego
    geometry_msgs::PoseStamped ego_ecef = geographic2geometryECEF(ego);
    ego_marker.header.stamp = ego_ecef.header.stamp;
    ego_marker.pose.position = ego_ecef.pose.position;
    tf2::Quaternion ego_quat;
    tf2::fromMsg(ego_ecef.pose.orientation, ego_quat);
    ego_marker.pose.orientation = tf2::toMsg(ego_quat*ego_marker_correction);
    ego_pub_.publish(ego_marker);

    tf2::Quaternion ecef2ned;
    ecef2ned.setEulerZYX(0.0, D2R(-90.0), 0.0);
    geometry_msgs::PoseStamped axis;
    axis.header.frame_id = "world";
    axis.pose = ego_ecef.pose;
    axis.pose.orientation = tf2::toMsg(ego_quat*ecef2ned);
    axis_pub_.publish(axis);

    ego_transform.header.stamp = ego.header.stamp;
    ego_transform.transform.translation.x = ego_ecef.pose.position.x;
    ego_transform.transform.translation.y = ego_ecef.pose.position.y;
    ego_transform.transform.translation.z = ego_ecef.pose.position.z;
    ego_transform.transform.rotation = ego_ecef.pose.orientation;
    ego_br_.sendTransform(ego_transform);

    //* Draw waypoint
    geometry_msgs::Point waypoint_point = geographic2geometryECEF(waypoint);
    waypoint_marker.pose.position = waypoint_point;
    waypoint_pub_.publish(waypoint_marker);

    //* Draw waypoints trajectory
    traj_marker.points.clear();
    traj_marker.colors.clear();
    for(size_t idx = 0; idx < traj.points.size(); idx++)
    {
        geometry_msgs::Point traj_point_ecef = geographic2geometryECEF(traj.points.at(idx));
        traj_marker.points.push_back(traj_point_ecef);
        std_msgs::ColorRGBA color;
        color.a = color.r = color.g = color.b = 1.0;
        traj_marker.colors.push_back(color);
    }
    traj_pub_.publish(traj_marker);

    //* Draw lookahead
    lookah_marker.points.clear();
    lookah_marker.points.push_back(ego_marker.pose.position);
    geometry_msgs::Point lookah_point = geographic2geometryECEF(lookah);
    lookah_marker.points.push_back(lookah_point);
    lookah_pub_.publish(lookah_marker);
    
    markers_pub_.publish(visualizations);
    return true;
}