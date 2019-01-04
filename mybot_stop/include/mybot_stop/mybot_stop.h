#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class Stopper{

public:

    const static double FORWARD_SPEED = 0.5;
    //-30/180*M_PI
    const static double MIN_SCAN_ANGLE = -0.52;
    //+30/180*M_PI
    const static double MAX_SCAN_ANGLE = +0.52;
    const static float MIN_DIST_FROM_OBSTACLE = 0.3;

    Stopper();
    void startMoving();
    void stopMoving();


private:

    ros::NodeHandle node;
    ros::Publisher command_pub;
    ros::Subscriber laser_sub;
    bool keep_moving;
    void moveForward();
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);


};







