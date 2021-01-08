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
    float x = 0.0;
    float z = 0.0;
    int ball_pixel_num = 0;


    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    for(int row = 300; row < img.height; row ++) // Because top half image is sky or wall, so from row 300
    {
         for(int column = 0; column < img.width / 3; column ++) //left side search
	 {
            if (img.data[row * img.width + column] == white_pixel) {
                
                ball_pixel_num++ ;
		if (ball_pixel_num > 80){// if there is greater than 80 pixels in white
        		drive_robot(0.0, -0.1); 
			ROS_INFO_STREAM("Turn to Left");
			break;					
		} 
    	    }
	 }

         for(int column = img.width / 3; column <  (2 * img.width / 3); column ++) //middle side search
	 {
            if (img.data[row * img.width + column] == white_pixel) {
                
                ball_pixel_num++ ;
		if (ball_pixel_num > 80){// if there is greater than 80 pixels in white
        		drive_robot(0.2, 0.0);  // This request drives my_robot robot forward
			ROS_INFO_STREAM("Drive forward");
			break;					
		} 
    	    }
	 }

         for(int column = (2 * img.width / 3); column <  img.width; column ++) //right side search
	 {
            if (img.data[row * img.width + column] == white_pixel) {
                
                ball_pixel_num++ ;
		if (ball_pixel_num > 80){// if there is greater than 80 pixels in white
        		drive_robot(0.0, 0.1); 
			ROS_INFO_STREAM("Turn to Right");
			break;					
		} 
    	    }
	 }




    }
    if (ball_pixel_num == 0) {
        x = 0.0;
        z = 0.0;
	ROS_INFO_STREAM("No ball in front, stop");
	drive_robot(x, z);
    }

  
}

/***
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    int left_counter = 0;
    int front_counter = 0;
    int right_counter = 0;
	
    // TODO: 
    // Loop through each pixel in the image and check if there's a bright white one
    for (int i = 0; i < img.height * img.step; i += 3) {
        int position_index = i % (img.width * 3) / 3;
	
        if (img.data[i] == white_pixel && img.data[i + 1] == white_pixel && img.data[i + 2] == white_pixel) {
            if(position_index <= 265) {
		left_counter += 1;                
            }
            if(position_index > 265 && position_index <= 533) {
		front_counter += 1;               
            }
            if(position_index > 533) {
		right_counter += 1;                
            }
	}
    }
		
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    vector<int> position_counter{left_counter, front_counter, right_counter};
    int where_to_move = *max_element(position_counter.begin(), position_counter.end());

    // Depending on the white ball position, call the drive_bot function and pass velocities to it.
    // Request a stop when there's no white ball seen by the camera.
    if (where_to_move == 0){
	ROS_INFO_STREAM("Searching the ball!");
	drive_robot(0.0, 0.1);
        drive_robot(0.0, 0.0); // This request brings my_robot to a complete stop
    }
    else if (where_to_move == left_counter) {
	drive_robot(0.0, 0.1);  
	ROS_INFO_STREAM("Turn to Right");
    }
    else if (where_to_move == front_counter) {
        drive_robot(0.2, 0.0);  // This request drives my_robot robot forward
	ROS_INFO_STREAM("Drive forward");
    }
    else if (where_to_move == right_counter) {
        drive_robot(0.0, -0.1); 
	ROS_INFO_STREAM("Turn to Left");
    }
}
***/


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
