#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <vector>

int main(int argc, char **argv) {
    ros::init(argc, argv, "multi_topic_publisher");
    ros::NodeHandle nh;

    int num_topics = atoi(argv[1]);
    double rate = atof(argv[2]);

    std::vector<ros::Publisher> publishers;
    for (int i = 0; i < num_topics; ++i) {
        std::stringstream topic_name;
        topic_name << "topic_" << i;
        publishers.push_back(nh.advertise<std_msgs::String>(topic_name.str(), 1000));
    }

    ros::Rate loop_rate(rate);

    while (ros::ok()) {
        for (int i = 0; i < num_topics; ++i) {
            std_msgs::String msg;
            std::stringstream ss;
            ss << "Hello from " << publishers[i].getTopic() << "!";
            msg.data = ss.str();

            publishers[i].publish(msg);
            // ROS_INFO("%s", msg.data.c_str());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}