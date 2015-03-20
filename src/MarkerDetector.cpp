/*!
 *  \brief     Unibg ardrone project (robotics lab)
 *  \details   This class is used to receive images from ardrone frontcam,
 *              to convert them, to detect the marker and its position relate to camera
 *              and to send tf between the marker and the sardrone frontcam.
 *  \author    Yamuna Maccarana
 *  \author    Luca Calomeni
 *  \author    Manuel Facchinetti
 *  \date      2015
 *  \copyright University of Study of Bergamo: all rights reserved.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <iostream>
// Includes for aruco library
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>
// Includes for image conversion
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
// Includes for tf
#include <tf/transform_broadcaster.h>

using namespace cv;
using namespace aruco;

// Global parameters------------------------------------------------------------------------------
CameraParameters TheCameraParameters;
bool firstImageReceived;
cv_bridge::CvImagePtr cv_ptr;
cv::Mat InImage;
Marker MyMarker;
float TheMarkerSize = 0.017;
static const std::string OPENCV_WINDOW = "Image window";
int MarkerNumber = 36;

// Callback----------------------------------------------------------------------------------------
void imageConverterCallback(const sensor_msgs::ImageConstPtr& msg){
    //ROS_INFO("Frontcam : image received");

    firstImageReceived = true;
    // Image conversion through cv_bridge
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    InImage = cv_ptr->image;
}

// main-------------------------------------------------------------------------------------------
int main(int argc, char **argv){
    ros::init(argc, argv, "MarkerDetector");
    firstImageReceived = false;

    // It's not necessary to define frequency:
    // whenever a frame is published, the node receives it
    // and call imageConverterCallback

    // Define the node handler for subscription
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/ardrone/front/image_raw", 1000, imageConverterCallback);

    ROS_INFO("MarkerDetector : Started");
    ROS_INFO("Waiting for camera connection...");

    // Camera parameters capture
    TheCameraParameters.readFromXMLFile("camera.yml");

    while(ros::ok()){
        if(firstImageReceived){
            // Marker detection-------------------------------------------------------------------
            vector<Marker> Markers;
            bool found = false;

            try{
                MarkerDetector MDetector;

                // Marker detection
                MDetector.detect(InImage, Markers, TheCameraParameters, TheMarkerSize);
                // Boundaries drawing for each marker in image
                for(unsigned int i=0;i<Markers.size();i++){
                    // UnibgMarkerDetector selects only markers with id = 36
                    if(Markers[i].id == MarkerNumber){
                        found = true;
                        MyMarker = Markers[i];
                        MyMarker.draw(InImage,Scalar(0,0,255),2);
                    }
                }

                // Image show with markers info
                cv::imshow(OPENCV_WINDOW,InImage);
                cv::waitKey(1);//wait for key to be pressed to close
            } catch (std::exception &ex){
                cout<<"Exception :"<<ex.what()<<endl;
            }

            // Detected marker elaboration-----------------------------------------------------------------------
            float x_tf, y_tf, z_tf;
            float roll, yaw, pitch;

            if(found){
                x_tf = MyMarker.Tvec.at<Vec3f>(0,0)[0] * 10; // in meters
                y_tf = MyMarker.Tvec.at<Vec3f>(0,0)[1] * 10;
                z_tf = MyMarker.Tvec.at<Vec3f>(0,0)[2] * 10;
                //printf("%4.2f %4.2f %4.2f\n",x_t,y_t,z_t);

                // You need to apply cv::Rodrigues() in order to obatain angles wrt to camera coords
                cv::Mat rot_mat(3,3,cv::DataType<float>::type);
                cv::Rodrigues(MyMarker.Rvec,rot_mat);

                pitch = -atan2(rot_mat.at<float>(2,0), rot_mat.at<float>(2,1));
                yaw = acos(rot_mat.at<float>(2,2));
                roll = -atan2(rot_mat.at<float>(0,2), rot_mat.at<float>(1,2));

                // Transform definition and publishing-----------------------------------------------------------
                // Txyz and Rxyz survey
                static tf::TransformBroadcaster br;
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(x_tf,y_tf,z_tf));
                transform.setRotation(tf::createQuaternionFromRPY(yaw,pitch,roll));

                // Tf publishing
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/ardrone_base_frontcam", "marker"));
            }else{
                printf("Searching for markers...\n");
            }

            // Marker rotation should be initially zero (for convenience)
/*          float p_off = CV_PI;
            float r_off = CV_PI/2;
            float y_off = CV_PI/2; */

            if (found){
                //printf( "Angles (deg) wrt Flight Dynamics: roll:%5.2f pitch:%5.2f yaw:%5.2f \n", (roll-r_off)*(180.0/CV_PI), (pitch-p_off)*(180.0/CV_PI), (yaw-y_off)*(180.0/CV_PI));
                printf( "Marker distance in metres:  x:%5.2f   y:%5.2f z:%5.2f \n", x_tf*10, y_tf*10, z_tf*10);
            }
        }

        ros::spinOnce();
    }

    ros::spin();
    return 0;
}
