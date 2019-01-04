#include <mybot_stop/mybot_stop.h>
#include "geometry_msgs/Twist.h"

Stopper::Stopper(){

    keep_moving = true;
    command_pub = node.advertise<geometry_msgs::Twist>("cmd_vel",30);
    laser_sub = node.subscribe("/mybot/laser/scan",1, &Stopper::scanCallBack,this);

}

void Stopper::moveForward(){

    geometry_msgs::Twist msg;
    msg.linear.x = FORWARD_SPEED;
    command_pub.publish(msg);

}

void Stopper::stopMoving(){
	geometry_msgs::Twist msg;
	msg.linear.x = 0;
	command_pub.publish(msg);



}


void Stopper::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan){

    bool is_obstacle_infront = false;
    //find the closest range between the defined min and max angles
    int min_index = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan-> angle_increment);
    int max_index = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan-> angle_increment);

    float closest_range = scan->ranges[min_index];
    for (int curr_index = min_index +1 ; curr_index <= max_index ; curr_index++){
    	if (scan->ranges[curr_index] < closest_range ){
    		closest_range = scan->ranges[curr_index];
    	}
    }

    /*if (scan->ranges[curr_index] < MIN_DIST_FROM_OBSTACLE ){
                is_obstacle_infront = true;
                break;

            }
            else{
            	is_obstacle_infront = false;
            	keep_moving = true;
            }
         */

    if (closest_range < MIN_DIST_FROM_OBSTACLE){
    	ROS_INFO("stop");
    	keep_moving = false;
    	is_obstacle_infront = true;

    }
    else {
    	ROS_INFO("keep_moving");
    	keep_moving = true;
    	is_obstacle_infront = false;
    }



   /* if (is_obstacle_infront){
        ROS_INFO("stop");
        keep_moving =false;
    }
    */

}

void Stopper::startMoving(){
    ros::Rate rate(30);
    ROS_INFO ("start moving");

    while (ros::ok()){
    	//updateStatus();
    	while (keep_moving){
    		moveForward();
    	    ros::spinOnce();
    	    rate.sleep();
    	}

    	        //stop moving

    	stopMoving();
    	ros::spinOnce();




    }

   //keep movig until the robot get close to an obstacle
   /* while (ros::ok && keep_moving ){
        moveForward();
        ros::spinOnce();
        rate.sleep();

    }

    //stop moving
    if(!keep_moving)
    	stopMoving();

*/
}


