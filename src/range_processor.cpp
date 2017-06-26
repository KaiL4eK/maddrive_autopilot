#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sstream>

uint32_t seq_prev = 0;

using namespace std;
using namespace ros;

// #define LOSING_TEST

vector<Publisher> publishers;

void rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{
    sensor_msgs::Range new_msg = *msg;
    uint8_t rangefinder_number = 0;

    string frame_id = string(msg->header.frame_id);
    char last_frame_character = frame_id.back();

    rangefinder_number = last_frame_character - '0';

    // ROS_INFO("[%d], [%f]", rangefinder_number, msg->range );

    publishers.at(rangefinder_number-1).publish(new_msg);

#ifdef LOSING_TEST
    if ( seq_prev != msg->header.seq - 1 )
        cout << "Lost" << endl;

    seq_prev = msg->header.seq;
#endif
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "range_processor");
    NodeHandle n;
    Subscriber sub = n.subscribe("range", 10, rangeCallback);
    

    for (int i_handle = 0; i_handle < 8; i_handle++)
    {
        ostringstream ss;
        ss << "rangefinder" << (i_handle+1);
        publishers.push_back( n.advertise<sensor_msgs::Range>( ss.str(), 100 ) );
    }

    ros::spin();

    return 0;
}