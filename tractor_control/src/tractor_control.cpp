#include "ros/ros.h"
#include "math.h"
#include "ros/time.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"

float tractor_velocity_kmh = 0.0 ;
float tractor_steering_angle_radian = 0.0 ;

// Caller function  
void receivedVelocity(const std_msgs::Float32::ConstPtr &msg) 
{
    ROS_INFO("velocity received: %f", msg->data);
    
    tractor_velocity_kmh = msg->data ;
}

void receivedSteering(const std_msgs::Float32::ConstPtr &msg) 
{
    ROS_INFO("steering received: %f", msg->data);
    
    float data = msg->data ;
    
    //to radian
    tractor_steering_angle_radian = data * (M_PI/180.0) ;
}

int main(int argv, char **argc) 
{
    // Initialize node
    ros::init(argv, argc, "CPP_Joint_State_Controller");
    // Node handler
    ros::NodeHandle nodeHandler;
    // Publisher
    ros::Publisher publisherObject = nodeHandler.advertise<sensor_msgs::JointState>("joint_states", 10);
    
    
    ros::Subscriber sub_vel = nodeHandler.subscribe("/tractor/control/velocity_kmh", 1000, receivedVelocity);
    ros::Subscriber sub_steer = nodeHandler.subscribe("/tractor/control/steering_angle_degree", 1000, receivedSteering);
    
    ros::Rate rateController = ros::Rate(50);		//100Hz

    // Variables
    float angle = 0.0;              // Angle
    bool increment = true;
    sensor_msgs::JointState msg;    // Message to be published
    msg.header.frame_id = "";           // Empty frame ID
    msg.name.resize(6);                 // A 1 unit size vector
    msg.position.resize(6);             // A 1 unit size vector
    msg.name[0] = "left_steering_hinge_wheel";
    msg.name[1] = "front_left_wheel";
    msg.name[2] = "right_steering_hinge_wheel";
    msg.name[3] = "front_right_wheel";
    msg.name[4] = "rear_left_wheel";
    msg.name[5] = "rear_right_wheel";
    msg.position[0] = 0.0 ;
    msg.position[1] = 0.0 ;
    msg.position[2] = 0.0 ;
    msg.position[3] = 0.0 ;
    msg.position[4] = 0.0 ;
    msg.position[5] = 0.0 ;

    /*
    앞바퀴 반지름 : 586.745mm
    뒷바퀴 반지름 : 741.78mm
    */
    //front wheel radius
    const double front_wheel_radius = 586.745 ;
    const double front_wheel_circle_mm = 2.0 * M_PI * front_wheel_radius ;
    //rear wheel radius
    const double rear_whee_radius = 741.78 ;
    const double rear_wheel_circle_mm = 2.0 * M_PI * rear_whee_radius ;
    
    // Main iterative code
    while(ros::ok()) 
    {	
    	//kmh -> mm/hz
    	double tractor_velocity_mm_hz = (tractor_velocity_kmh * 1000000.0) / (3600.0 * 50.0) ;

	double radian_rotate_front_wheel = 2.0*M_PI * (tractor_velocity_mm_hz / front_wheel_circle_mm) ;
	double radian_rotate_rear_wheel = 2.0*M_PI * (tractor_velocity_mm_hz / rear_wheel_circle_mm) ;
	
	//front rotate circle 
	
	
        // Update header
        msg.header.stamp = ros::Time::now();    // Assign time
        
        msg.position[0] = tractor_steering_angle_radian;
        msg.position[1] += radian_rotate_front_wheel;
        msg.position[2] = tractor_steering_angle_radian;
        msg.position[3] += radian_rotate_front_wheel;
        msg.position[4] += radian_rotate_rear_wheel;
        msg.position[5] += radian_rotate_rear_wheel;
        
        publisherObject.publish(msg);
        msg.header.seq ++;                      // Next sequence
        
        // Control rate
        rateController.sleep();
        
        ros::spinOnce();
    }
    
    // End code
    return 0;
}
