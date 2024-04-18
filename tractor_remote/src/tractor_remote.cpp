#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#define MAX_STEERING_ANGLE 	45
#define MAX_VELOCITY_KMH	100

int getch()
{
    int c;
    struct termios oldattr, newattr;

    tcgetattr(STDIN_FILENO, &oldattr);           // 현재 터미널 설정 읽음
    newattr = oldattr;
    newattr.c_lflag &= ~(ICANON | ECHO);         // CANONICAL과 ECHO 끔
    newattr.c_cc[VMIN] = 1;                      // 최소 입력 문자 수를 1로 설정
    newattr.c_cc[VTIME] = 0;                     // 최소 읽기 대기 시간을 0으로 설정
    tcsetattr(STDIN_FILENO, TCSANOW, &newattr);  // 터미널에 설정 입력
    c = getchar();                               // 키보드 입력 읽음
    tcsetattr(STDIN_FILENO, TCSANOW, &oldattr);  // 원래의 설정으로 복구
    return c;
}

int main(int argv, char **argc) 
{
    // Initialize node
    ros::init(argv, argc, "Tractor_Remote_Controller");
    
    // Node handler
    ros::NodeHandle nodeHandler;

    // Publisher
    ros::Publisher pub_vel = nodeHandler.advertise<std_msgs::Float32>("/tractor/control/velocity_kmh", 1000);
    ros::Publisher pub_steer = nodeHandler.advertise<std_msgs::Float32>("/tractor/control/steering_angle_degree", 1000);
    
    ros::Rate rateController = ros::Rate(20);

    // Main iterative code
    float vel = 0.0 ;
    float angle = 0 ;
    	
    	
    while(ros::ok()) 
    {
    	
    	char key = getch();
    	
    	if( key == 'a' || key == 'A' )
    	{
    		angle-- ;
    		if( angle < -MAX_STEERING_ANGLE ) angle = -MAX_STEERING_ANGLE ;    		
    	}
    	else if( key == 'd' || key == 'D' )
    	{
	    	angle++ ;
	    	if( angle > MAX_STEERING_ANGLE ) angle = MAX_STEERING_ANGLE ;
    	}
    	else if( key == 'w' || key == 'W' )
    	{
    		vel++ ;
    		if( vel > MAX_VELOCITY_KMH ) vel = MAX_VELOCITY_KMH ;    		
    	}
    	else if( key == 's' || key == 'S' )
    	{
	    	vel-- ;
	    	if( vel < -MAX_VELOCITY_KMH ) vel = -MAX_VELOCITY_KMH ;
    	}
    	
    	//velocity
    	std_msgs::Float32 pubMsg;
        pubMsg.data = vel;  // Assign the number to the message to be published
        pub_vel.publish(pubMsg);
        
        pubMsg.data = angle;  // Assign the number to the message to be published
        pub_steer.publish(pubMsg);
        
        ros::spinOnce();
        
        ROS_INFO("tractor remote value : %f, %f", vel, angle);
        
        // Control rate
        rateController.sleep();
    }
    // End code
    return 0;
}
