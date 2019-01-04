#include <mybot_stop/mybot_stop.h>

int main(int argc, char **argv){

    ros::init(argc,argv, "stopper");

    //create new stopper object

    Stopper stopper;
    stopper.startMoving();


    return 0;


}
