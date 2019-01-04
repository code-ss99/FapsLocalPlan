
#include "faps_local_planner/faps_local_planner.h"

// pluginlib macros (defines, ...)
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(faps_local_planner, FapsPlannerROS, faps_local_planner::FapsPlannerROS, nav_core::BaseLocalPlanner)

namespace faps_local_planner{

	FapsPlannerROS::FapsPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

	FapsPlannerROS::FapsPlannerROS(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
         : costmap_ros_(NULL), tf_(NULL), initialized_(false)
         {
		// initialize planner
		initialize(name, tf, costmap_ros);
         }

	FapsPlannerROS::~FapsPlannerROS() {}

	void FapsPlannerROS::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
	{

		// check if the plugin is already initialized
		if(!initialized_)
		{
		// copy adress of costmap and Transform Listener (handed over from move_base)
		costmap_ros_ = costmap_ros;
		tf_ = tf;


		// subscribe to topics (to get odometry information, we need to get a handle to the topic in the global namespace)
		ros::NodeHandle nh;
		amcl_sub = nh.subscribe("amcl_pose", 100, &FapsPlannerROS::amclCallBack, this);

		//subscribe to scan to receive laser scan(topic have to fix)
		laser_sub = nh.subscribe("/scan",100,&FapsPlannerROS::scanCallBack,this);

		//output PoseStamped from global plan
        pose_pub =  nh.advertise<geometry_msgs::PoseArray>("/display_pose", 1, true);
        global_pose_.header.stamp = ros::Time::now();
        global_pose_.header.frame_id = "map";
        //output line strips to display circle around robot
        marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);
        circle_strip_.header.frame_id = "map";
        circle_strip_.header.stamp = ros::Time::now();

        //Parameter for dynamic reconfigure
        dsrv_ = new dynamic_reconfigure::Server<FAPSPlannerConfig>(nh);
        dynamic_reconfigure::Server<FAPSPlannerConfig>::CallbackType cb = boost::bind(&FapsPlannerROS::reconfigureCB, this, _1, _2);
        //dsrv_->setCallback(cb);
        dsrv_->setCallback(cb);


        //get the parameter
        double testrate=0.0;
        //controller_rate_=20.0;
        //nh_.setParam("/move_base/controller_frequency", controller_rate_);
        controller_rate_=0;
        while (testrate<0.0001) {

            ROS_INFO("reading param values: ");
            //this is the frequency at which the local planner will be updated
            nh.getParam("/move_base/controller_frequency", controller_rate_);
            ROS_INFO("controller update rate set to %f", controller_rate_);
            //here are specified dynamic limits of the robot:
            //SHOULD use namespace of this module (FapsPlanner), but here I simply
            //re-use the values specified for the default nav_stack base_local_planner,
            //as specified in base_local_planner_params.yaml in the launchfile:
            nh.getParam("/move_base/TrajectoryPlannerROS/acc_lim_theta", acc_lim_theta_);
            nh.getParam("/move_base/TrajectoryPlannerROS/acc_lim_x", acc_lim_x_);
            nh.getParam("/move_base/TrajectoryPlannerROS/max_vel_theta", max_vel_theta_);
            nh.getParam("/move_base/TrajectoryPlannerROS/max_vel_x", max_vel_x_);
            nh.getParam("/move_base/TrajectoryPlannerROS/min_vel_theta", min_vel_theta_);
            nh.getParam("/move_base/TrajectoryPlannerROS/min_vel_x", min_vel_x_);
            //nh.param("tolerance_timeout", tolerance_timeout_, 20);
            ROS_INFO("acc_max = %f; alpha_max = %f",acc_lim_x_,acc_lim_theta_);
            ROS_INFO("max_vel = %f; max_omega = %f",max_vel_x_,max_vel_theta_);
            testrate=controller_rate_*acc_lim_theta_*acc_lim_x_*max_vel_theta_*max_vel_x_;
            //set the duration of sleep
            ros::Duration(10).sleep();
        }
        //max brake dist: dmax = 0.5*a*t^2
        //v = a*t
        // dmax = 0.5*a*(v/a)^2 = 0.5*vmax^2/amax
        min_theta_brake_dist_= 0.5*min_vel_theta_*min_vel_theta_/acc_lim_theta_;

        min_lin_brake_dist_ = 0.5*max_vel_x_*max_vel_x_/acc_lim_x_;

		// set initialized flag
		initialized_ = true;

		//Parameter for dynamic reconfigure

		// this is only here to make this process visible in the rxlogger right from the start
		ROS_DEBUG("faps Local Planner plugin initialized.");
		}
		else
		{
		ROS_WARN("This planner has already been initialized, doing nothing.");
		}

	}

    void FapsPlannerROS::reconfigureCB(FAPSPlannerConfig &config, uint32_t level)
    {
        if (config.restore_defaults){
            config = default_config_;
            config.restore_defaults = false;
        }
        	config_ = config;

		//ROS_INFO("Reconfigure Request: %f ", config_.position_accuracy);

    }

    void FapsPlannerROS::circleMarker()
    {
        circle_strip_.ns = "lines";
        circle_strip_.action = visualization_msgs::Marker::ADD;
        circle_strip_.pose.orientation.w = 1.0;
        circle_strip_.id = 0;
        circle_strip_.type = visualization_msgs::Marker::LINE_STRIP;
        circle_strip_.scale.x = 0.01;
        //line is blue
        circle_strip_.color.b = 1.0;
        circle_strip_.color.a = 1.0;
        circle_strip_.points.clear();
        for (int i = 0; i < 360; ++i){
            geometry_msgs::Point p;
            float x = now.x + Chk_Circle_Radius * cos(i * D2R);
            float y = now.y + Chk_Circle_Radius * sin(i * D2R);
            p.x = x;
            p.y = y;
            p.z = 1.0;
            circle_strip_.points.push_back(p);

        }

    }


	bool FapsPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
	{

		// check if plugin initialized
		if(!initialized_)
		{
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}

		//reset next counter
		count = 1;

		//set plan, length and next goal
		plan = orig_global_plan;
		length = (plan).size();
		//ROS_INFO("GOT A PLAN OF SIZE %lu", length);
		setNext();

		// set goal as not reached
		goal_reached_ = false;
		return true;

	}

	bool FapsPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
	{

		// check if plugin initialized
		if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
		}
		//get the current pose of the robot in the fixed frame
        tf::Stamped<tf::Pose> robot_pose;
        if(!costmap_ros_->getRobotPose(robot_pose)){
            ROS_ERROR("Can't get robot pose");
            setVelZ();
            return false;
        }
        //ros time
        ros::Time begin = ros::Time::now();

		if(length != 0){

            if(!checkObstacle()){
                ROS_INFO("no obstacle:keep moving");
                //ROS_INFO("i: %lu", is_obstacle_near_);
                countDiff();
                //make sure that robot can stop before aim
                if(distance < Chk_Circle_Radius){
                    if(count < (length-10)){
                        while (distance < Chk_Circle_Radius){
                            count ++;
                            setNext();
                            countDiff();
                        }
                        setNext();
                    }
                    else if(count >= (length-10) && count < (length-1)){
                        count = length - 1;
                        setNext();

                    }

                        /*if((length - 1 - count) < 11){
                            count = length - 1;
                        }else{
                            count += 10;
                        }
                        setNext();
                        */
                    else{

                        setVelZ();
                        goal_reached_ = true;

                    }
                }
                else{

                    if(fabs(nDiff.az) > 25*D2R){

                        setRot();
                        ROS_INFO("set Rot");

                    }else{

                        setVel();
                        ROS_INFO("set Vel");

                    }
                }

			}
			else {
                ROS_INFO("found obstacle:slow down the vechicle");
                count = count;
                setVelScal();

			}


		}

		// set retrieved commands to reference variable
		ROS_INFO("Retrieving velocity command: (%f, %f, %f)", cmd.linear.x, cmd.linear.y, cmd.angular.z);
		cmd_vel = cmd;

		return true;

	}

	bool FapsPlannerROS::isGoalReached()
	{
		// check if plugin initialized
		if(!initialized_)
		{
			ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
			return false;
		}

		// this info comes from compute velocity commands:
		return goal_reached_;

	}

	void FapsPlannerROS::amclCallBack(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg)
	{

		ROS_INFO("pose: [%d]", msg->header.seq);

		now.x = msg->pose.pose.position.x;
		now.y = msg->pose.pose.position.y;
		now.az = getYaw(*msg);

		ROS_INFO("x: %lu ,y: %lu,z: %lu", now.x,now.y,now.az);
		countDiff();

		circleMarker();
		marker_pub.publish(circle_strip_);

	}

    void FapsPlannerROS::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        bool is_obstacle_near_ = false;
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
            keep_moving_ = false;
            is_obstacle_near_ = true;

        }
        else {
            //ROS_INFO("keep_moving");
            keep_moving_ = true;
            is_obstacle_near_= false;
        }
           /* if (is_obstacle_infront){
        ROS_INFO("stop");
        keep_moving =false;
        }
        */
        laserData = *scan;
    }

    bool FapsPlannerROS::checkObstacle()
    {
            for(int i = 0; i<180; i++){
                if(laserData.ranges[i]<= MIN_DIST_FROM_OBSTACLE)
                    return true;
            }

            return false;
    }



	double FapsPlannerROS::getYaw(geometry_msgs::PoseWithCovarianceStamped msg)
	{

		double q[4];
		q[0]= msg.pose.pose.orientation.x;
		q[1]= msg.pose.pose.orientation.y;
		q[2]= msg.pose.pose.orientation.z;
		q[3]= msg.pose.pose.orientation.w;

		double t3 = +2.0 * (q[3] * q[2] + q[0] * q[1]);
		double t4 = +1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);

		return std::atan2(t3, t4);

	}

	void FapsPlannerROS::setVel()
	{

		// the output speed has been adjusted with a P regulator, that depends on how close we are to our current goal
		cmd.linear.x= distance;
		//cmd.linear.x = max_vel_x;
		// keeping a small angular speed so that the movement is smooth
		//vel_theta_ = (nDiff.az)* max_vel_theta_;
		cmd.angular.z= vel_theta_;
		//cmd.angular.z= config_.min_rotation_vel;
		//make sure that linear velocity are limited unter max v and above min v
		double linear_uplimit = sqrt(cmd.linear.x * cmd.linear.x + cmd.linear.y * cmd.linear.y) / max_vel_x_;
		double linear_downlimit = min_vel_x_ / sqrt(cmd.linear.x * cmd.linear.x + cmd.linear.y * cmd.linear.y);
		//limit the max v
		if (linear_uplimit > 1 ){
            cmd.linear.x /= linear_uplimit;
            cmd.linear.y /= linear_uplimit;
		}
		//limit the min v
		if (linear_downlimit > 1){
            cmd.linear.x *= linear_downlimit;
            cmd.linear.y *= linear_downlimit;
		}

	}

	void FapsPlannerROS::setRot()
	{



		// the angular speed has been adjusted with a P regulator, that depends on how close we are to pointing at our current goal
		if (fabs(nDiff.az) > 50*D2R){

			//cmd.angular.z=config_.max_rotation_vel;
			cmd.angular.z=vel_theta_;

			// linear speed is zero while the angle is too big
			cmd.linear.x= 0.0;
			cmd.linear.y= 0.0;

		}else{

			//cmd.angular.z=config_.max_rotation_vel*0.75;
			cmd.angular.z=0.5*vel_theta_;

			// keeping a small linear speed so that the movement is smooth and make sure speed is above min v
			if ( nDiff.az * max_vel_x_ < min_vel_x_ ){
                cmd.linear.x = min_vel_x_;
                cmd.linear.y = min_vel_x_;
			}else {
                cmd.linear.x = nDiff.az * max_vel_x_;
                cmd.linear.y = nDiff.az * max_vel_x_;

			}
        }

	}

	void FapsPlannerROS::setVelZ()
	{

		cmd.linear.x= 0;
		cmd.linear.y= 0;
		cmd.angular.z=0;
		ROS_INFO("set v zero");

	}


    void FapsPlannerROS::setVelScal()
	{

		ROS_INFO("setScal");
		double scaling_factor = 0.5;
		double slow_factor = 50;
		double ds = scaling_factor/slow_factor;
		for (int i = 0; i < slow_factor ; i++){
            cmd.linear.x = cmd.linear.x * scaling_factor;
            cmd.linear.y = cmd.linear.y * scaling_factor;
            //cmd.linear.z = cmd.linear.z * scaling_factor;
            cmd.angular.z= 0;
            if(!checkObstacle()){
                //no more collision
                break;
            }

            scaling_factor = scaling_factor - ds;

		}

	}

	void FapsPlannerROS::setNext()
	{

		next.x = plan[count].pose.position.x;
		next.y = plan[count].pose.position.y;
		global_pose_.poses.push_back(plan[count].pose);
		pose_pub.publish(global_pose_);

	}

	void FapsPlannerROS::countDiff()
	{

		double d;

		nDiff.x = (next.x - now.x);
		nDiff.y = (next.y - now.y);

		// if these two variables are null, the tangent doesn't exist
		// plus, the angle error is irrelevant because we have arrived at our destination

		if (nDiff.y == 0 & nDiff.x == 0){
			d = now.az;
		}else{
			d = std::atan2(nDiff.y, nDiff.x);
		}

		distance = std::sqrt(nDiff.x*nDiff.x +nDiff.y*nDiff.y);
		nDiff.az = d - now.az;

		// make sure that we chose the smallest angle, so that the robot takes the shortest turn
		if ( nDiff.az > 180*D2R ) { nDiff.az -= 360*D2R; }
		if ( nDiff.az < -180*D2R ) { nDiff.az += 360*D2R;  }
		vel_theta_ = (nDiff.az)* max_vel_theta_;
	}

}
