#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

ros::ServiceClient client;

void drive_robot(float lin_x, float ang_z) {
    
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    
    if (!client.call(srv)){
        ROS_ERROR("Failed to call service ball_chaser");
    }
    
}

void process_image_callback(const sensor_msgs::Image img) {

    int white_pixel = 255;
    int height_image = img.height;
    int step_image = img.step;
    int array_size = height_image * step_image; // Total number of elements in the data
    int white_ball_index = -1; // Initialze the white pixel index as -1 to check if detected
    
    
    
    for (int i = 0; i < array_size; i++) { // Iterating through image matrix
        if (img.data[i] == white_pixel) { // Checking if it matches the white pixel
            white_ball_index = i; // Store the index of the white pixel
            break; // Break the for loop
        }
    }
    if (white_ball_index == -1) {
        // White ball is not detected. Stopping the robot.
        drive_robot(0.0, 0.0);
    } else if (white_ball_index % step_image < step_image / 3) {
        // White ball on the left side of the image
        drive_robot(0.1, -0.2); // Turning right while moving slowly forward
    } else if (white_ball_index % step_image < 2 * step_image / 3) {
        // White ball on the center of the image
        drive_robot(0.3, 0.0); // Move straight forward
    } else {
        // White ball on the right side of the image
        drive_robot(0.1, 0.2); // Turning left while moving slowly forward
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

