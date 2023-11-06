#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

//             Let
// x_pickup, y_pickup = pick up loaction
// x_dropoff, y_dropoff = drop off Location

// Define pick-up 
float x_pickup = 3.0;
float y_pickup = 3.0;

//Define drop-off coordinates
float x_dropoff = -3.0;
float y_dropoff = 3.0;

bool isPicked = false;
bool isDropped = false;

float margin = 0.5;

// Define a callback function to track robot's position
void OdometryCallback (const nav_msgs::Odometry::ConstPtr& msg) 
{
    // get Robot Position fron odometry
    float robot_x = msg->pose.pose.position.x;
    float robot_y = msg->pose.pose.position.y;
    
    float distanceToTarget;

    // define traget location as string which could be either pick off or drop off
    std::string targetLocation;

    
    if (!isPicked && !isDropped)
    {
        // get the distance from robot's position to pick up location using distance between vectors
        distanceToTarget = sqrt(pow((x_pickup - robot_x), 2) + pow((y_pickup - robot_y), 2));
        targetLocation = "pick-up";
    }
    else if (isPicked && !isDropped)
    {
        // get the distance from robot's position to drop off location using distance between vectors
        distanceToTarget = sqrt(pow((x_dropoff - robot_x), 2) + pow((y_dropoff - robot_y), 2));
        targetLocation = "drop-off";
    }

    ROS_INFO("Distance to the %s location = %f", targetLocation.c_str(), distanceToTarget);

    if (distanceToTarget <= margin)
    {
        ROS_INFO("Arrived at the %s location", targetLocation.c_str());

        if (!isPicked)
        {
            isPicked = true;
        }
        else
        {
            isDropped = true;
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_mover");
    ros::NodeHandle node;
    ros::Rate rate(1);
    ros::Publisher markerPublisher = node.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Subscribe to robot's odometry data with frequency of 1000Hz
    ros::Subscriber odometrySubscriber = node.subscribe("/odom", 1000, OdometryCallback);

    // Define the initial shape of the marker as a CYLINDER
    uint32_t markerShape = visualization_msgs::Marker::CYLINDER;

    // Initialize the marker
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Define the namespace and ID for the marker
    marker.ns = "object_shapes";
    marker.id = 0;

    // Set the marker type as a Cylinder
    marker.type = markerShape;

    // Set the marker action to ADD
    marker.action = visualization_msgs::Marker::ADD;

    // Set the marker's color to be blue and alpha to be non-zero to make marker visible
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    // Set the scale of the marker ie specify the size of the marker.
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.5;


    // Set the marker's initial pose
    marker.pose.position.x = x_pickup;
    marker.pose.position.y = y_pickup;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.lifetime = ros::Duration();

    while (ros::ok())
    {
        // Publish the marker
        while (markerPublisher.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }

        if (isPicked)
        {
            marker.action = visualization_msgs::Marker::DELETE; //this delete markers
            ROS_INFO("The object has been picked up");
            ros::Duration(2.0).sleep();
        }

        if (isDropped)
        {
            marker.pose.position.x = x_dropoff;
            marker.pose.position.y = y_dropoff;
            marker.action = visualization_msgs::Marker::ADD; // this adds markers
            ROS_INFO("The object has been successfully dropped off");
            ros::Duration(2.0).sleep();
        }

        markerPublisher.publish(marker);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

