
#ifndef FAPS_LOCAL_PLANNER_ROS_H_
#define FAPS_LOCAL_PLANNER_ROS_H_

#include <ros/ros.h>

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

// local planner specific classes which provide some macros
#include <base_local_planner/goal_functions.h>
//#include <base_local_planner/trajectory_planner_ros.h>

// msgs
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

// transforms
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/cost_values.h>
#include <base_local_planner/costmap_model.h>

//dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <faps_local_planner/FAPSPlannerConfig.h>

// boost classes ?
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// other
#include <array>
#include <vector>

// definitions
#define PI 3.14159265
#define D2R 0.0174532925      // = 3.14159265/180

namespace faps_local_planner{

  /**
   * @class fapsPlannerROS
   * @brief Plugin to the ros base_local_planner. Implements a wrapper for the Elastic Band Method
   */

   struct pos {

	double x, y, az;

   };


  class FapsPlannerROS : public nav_core::BaseLocalPlanner{

    public:
      /**
       * @brief Default constructor for the ros wrapper
       */
      FapsPlannerROS();

      /**
       * @brief Constructs the ros wrapper
       * @param name The name to give this instance of the elastic band local planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      FapsPlannerROS(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~FapsPlannerROS();

      /**
       * @brief Initializes the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Set the plan that the controller is following; also reset faps-planner
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();



    private:

      // Velocity methods
      /**
      * @brief Set Vel: function that sets linear speed
      */
      void setVel();

      /**
      * @brief Set Rot: function that sets angular speed
      */
      void setRot();

      /**
      * @brief Set Vel Z: function that sets linear and angular speed to ZERO
      */
      void setVelZ();
      /**
      * @brief Set Vel Scal: function that sets linear and angular speed with scal
      */
      void setVelScal();


      // Methods
      /**
       * @brief Amcl Callback: function is called whenever a new amcl msg is published on that topic
       * @param Pointer to the received message
       */
      void amclCallBack(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg);

      /**
       * @brief Scan Callback: function is called whenever a new scan msg is published on that topic
       * @param Pointer to the received scan message
       */
      void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);

      /**
      * @brief getYaw: function calculates the Yaw angle from the position message that amcl sent
      * @param msg: passes the amcl position info to the function
      */
      double getYaw(geometry_msgs::PoseWithCovarianceStamped msg);

      /**
      * @brief Obstacle: checks if there is any obstacle in the near
      */
      bool checkObstacle();

      /**
      * @brief countDiff: calculates the different between the next and present frame
      */
      void countDiff();

      /**
      * @brief getNext: uses count to set the next goal frame
      */
      void setNext();
      /**
      *@brief Reconfigure config_
      */
      void reconfigureCB(FAPSPlannerConfig &config, uint32_t level);
      /**
      *@brief draw circle aroud robot
      */
      void circleMarker();

      // Topics & Services
      ros::Subscriber amcl_sub; ///<@brief subscribes to the amcl topicc
      ros::Subscriber laser_sub;
      ros::Publisher pose_pub;
      ros::Publisher marker_pub;

      //Pointer to external objects (do NOT delete object)
      costmap_2d::Costmap2DROS* costmap_ros_; ///<@brief pointer to costmap
      tf::TransformListener* tf_; ///<@brief pointer to Transform Listener

      //for dynamic reconfigure
      dynamic_reconfigure::Server<FAPSPlannerConfig> *dsrv_;
      //dynamic_reconfigure::Server<FAPSPlannerConfig> dsrv_;
      //start config
      faps_local_planner::FAPSPlannerConfig default_config_;
      //reconfigure config
      faps_local_planner::FAPSPlannerConfig config_;


      // Data
      pos now; // present frame
      pos next; // next frame
      pos nDiff; // difference between present and next frames
      double distance;
      double vel_theta_;
      int length; // number of frames in the global plan
      int count; // keeps track of the number for the next frame in the global plan
      std::vector<geometry_msgs::PoseStamped> plan; // contains the global plan
      geometry_msgs::PoseArray global_pose_;
      geometry_msgs::Twist cmd; // contains the velocity
      sensor_msgs::LaserScan laserData;

      // Flags
      bool goal_reached_;
      bool initialized_;
      bool keep_moving_;
      bool is_obstacle_near_;

      // v and a of x/theta
      double controller_rate_, controller_dt_;
      double acc_lim_theta_,acc_lim_x_,max_vel_theta_,max_vel_x_;
      double min_vel_theta_,min_vel_x_;
      double max_reorientation_omega_;
      double min_theta_brake_dist_,min_lin_brake_dist_;

      //marker to display
      visualization_msgs::Marker circle_strip_;

      //collision plan
      //base_local_planner::TrajectoryPlannerROS collision_planner_;
      //set scan angle between -30 to 30
      //-30/180*M_PI
      constexpr const static double MIN_SCAN_ANGLE = -0.52;
      //+30/180*M_PI
      constexpr const static double MAX_SCAN_ANGLE = +0.52;
      //safe distance (need to fix later)
      constexpr const static double MIN_DIST_FROM_OBSTACLE = 0.9;
      //jump circle (check the waypoint of global plan)
      constexpr const static double Chk_Circle_Radius = 0.3;





  };
};

#endif

