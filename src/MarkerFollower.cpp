/*!
 *  \brief     Unibg ardrone project (robotics lab)
 *  \details   This class is used to receive tf between the marker and the ardone frontcam,
 *              to calculate twist and to send it to ardrone.
 *  \author    Yamuna Maccarana
 *  \author    Luca Calomeni
 *  \author    Manuel Facchinetti
 *  \date      2015
 *  \copyright University of Study of Bergamo: all rights reserved.
 */

#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
// Tf
#include <tf/transform_listener.h>
// Ardrone control
#include <std_msgs/Empty.h>
#include <boost/thread.hpp>

using namespace boost;
using namespace boost::this_thread;

// Global parameters------------------------------------------------------------------------------
int state = 0;  // Three states are provided:   0 : ready to takeoff
                //                              1 : follow marker
                //                              2 : land

// If user presses anything: go to next phase-----------------------------------------------------
void checkCin(){
    char follow_marker;
    std::cin >> follow_marker;
    state = 1; // state 1: follow marker
    char land;
    std::cin >> land;
    state = 2; // state 2: land
}

// main--------------------------------------------------------------------------------------------
int main(int argc, char **argv){
    ros::init(argc, argv, "MarkerFollower");

    ROS_INFO("MarkerFollower : Started");

    // Define the node handler for subscription
    tf::TransformListener listener;
    ros::NodeHandle n;

    // Loop rate
    ros::Rate rate(100.0);

    // Thread: if user presses anything: go to next phase
    boost::thread t(&checkCin);

    // Twist msgs initialization
    geometry_msgs::Twist twist_old; // Twist msg to control loss of marker
    geometry_msgs::Twist twist;
    twist.linear.x=0.0;
    twist.linear.y=0.0;
    twist.linear.z=0.0;
    twist.angular.x=0.0;
    twist.angular.y=0.0;
    twist.angular.z=0.0;

    while (ros::ok()){
        // Lookup transform-----------------------------------------------------------------------------------------
        tf::StampedTransform transform;
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

        try{
            listener.lookupTransform("/ardrone_base_frontcam", "marker",
                                     ros::Time(0), transform);
            printf("Received TF: x: %f, y: %f, z: %f. \n", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

            // Transformation logic----------------------------------------------------------------------------------
            // Forward-backward: quick enough: factor = 0.1
            twist.linear.x=(transform.getOrigin().z()-1)*0.1;
            // Left-right: quick enough: factor = 0.2
            twist.linear.y=-transform.getOrigin().x()*0.2;
            // Up-down: really slow: factor = 2.0
            twist.linear.z=-transform.getOrigin().y()*2.0; // see documentation for further details
        }
        catch (tf::TransformException &ex) { // If no tf are received: break operations untill next loop
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // Lost marker control---------------------------------------------------------------------------------------
        if(twist.linear.x==twist_old.linear.x && twist.linear.y==twist_old.linear.y && twist.linear.z==twist_old.linear.z){
            twist.linear.x=0.0;
            twist.linear.y=0.0;
            twist.linear.z=0.0;
            twist.angular.x=0.0;
            twist.angular.y=0.0;
            twist.angular.z=0.0;
        }
        else{
            twist_old = twist;
        }

        // States----------------------------------------------------------------------------------------------------
        // State 0: takeoff
        if(state == 0){
            ros::Publisher pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
            double time_start=(double)ros::Time::now().toSec();

            while ((double)ros::Time::now().toSec()< time_start+3.0){ // Send command for three seconds
                pub.publish(std_msgs::Empty()); // Launches the drone
                ros::spinOnce();
            }
        }

        // Phase 1: follow marker
        if(state == 1){
            ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            double time_start=(double)ros::Time::now().toSec();

            while ((double)ros::Time::now().toSec()< time_start+0.1){ // Send command for 0.1 second
                pub.publish(twist);
                ros::spinOnce();
            }
        }

        // Phase 2: land
        if(state == 2){
            ros::Publisher pub = n.advertise<std_msgs::Empty>("/ardrone/land", 1);
            double time_start=(double)ros::Time::now().toSec();

            while ((double)ros::Time::now().toSec()< time_start+3.0){ // Send command for three seconds
                pub.publish(std_msgs::Empty()); // Land the drone
                ros::spinOnce();
                rate.sleep();
            }

            state = 4;
        }

        if(state == 4){
            return 0;
        }

        rate.sleep();
    }
    return 0;
}
