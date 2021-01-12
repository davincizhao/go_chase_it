#include <iostream>
#include <algorithm>
#include <vector>
using namespace std;


#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Chasering ball: call service to drive robot");

    // Request drive robot to target
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;


    // Call the DriveToTarget service and pass the requested wheel angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service safe_move");

}


// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    int postion_index = -1;
    int ball_column_index = -1;



    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera


    // Loop through each pixel in the image , size= img.height*img.step
    for(int index = 0; index < img.height * img.step; index += 3) 
    {


	    //The reason to check i, i+1 and i+2 is because the image data is arranged as R, G, B values in the three consecutive array positions
            if (img.data[index] == white_pixel 
               && img.data[index + 1] == white_pixel 
	       && img.data[index + 2] == white_pixel){
                ROS_INFO_STREAM("yes,yes there is white ball");
                postion_index = index; // save white ball to position index;

		break;					
		} 
    	
    }
    ball_column_index = postion_index % img.width;  // white ball pixel's column index
    ROS_INFO("ball_column=%d,postion_index=%d",ball_column_index,postion_index);
    if (postion_index == -1) // there is no white ball, stop
    {
	drive_robot(0.0, 0.0); 
	ROS_INFO_STREAM("there is no white ball, stop stop!");	
    }
    else{ // get the ball postion.

	    if (ball_column_index >= 0 && ball_column_index < img.width /3){
	    	drive_robot(0.0, 0.05); 
		ROS_INFO_STREAM("Turn to Left");
	    }
	    if (ball_column_index >= img.width /3 && ball_column_index < 2 * img.width /3){
	    	drive_robot(0.05, 0.0); 
		ROS_INFO_STREAM("Drive forward");
	    }
	    if (ball_column_index >= 2 * img.width /3){
	    	drive_robot(0.0, -0.05 ); 
		ROS_INFO_STREAM("Turn to right");
		//drive_robot(0.0, 0.0);
	    }
    }



}




int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
