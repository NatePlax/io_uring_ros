#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <array>
#include <sstream>
#include <chrono>
#include <atomic>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/String.h"

std::array<int, 1> output_fds;
std::atomic<long long> total_duration(0);
std::atomic<int> callback_count(0);

bool open_output_files() {
    std::string filename = "output.txt";
    output_fds[0] = open(filename.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (output_fds[0] < 0) {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return false;
    }
    return true;
}

void cleanup() {
    for (auto& fd : output_fds) {
        if (fd != -1) {
            close(fd);
        }
    }
}

void write_to_file(const std::string& data) {
    ssize_t written = write(output_fds[0], data.c_str(), data.size());
    if (written < 0) {
        ROS_ERROR("Failed to write to file");
    }
}

void messageCallback(const std_msgs::String::ConstPtr& msg) {
    auto start = std::chrono::high_resolution_clock::now();

    std::stringstream ss;
    ss << "Received message: " << msg->data << "\n";

    write_to_file(ss.str());

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::nano> duration = end - start;
    total_duration += (long)duration.count();
    callback_count++;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");

    if (!open_output_files()) {
        return 1;
    }

    ros::NodeHandle n;
    std::vector<ros::Subscriber> subscribers;
    int num_topics = 36;

    for (int i = 0; i < num_topics; ++i) {
        std::stringstream topic_name;
        topic_name << "/topic_" << i;
        subscribers.push_back(n.subscribe(topic_name.str(), 1000, messageCallback));
    }

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();

    for (auto& fd : output_fds) {
        if (fd != -1) {
            fsync(fd);
        }
    }



    if (callback_count > 0) {
        double average_duration = static_cast<double>(total_duration.load()) / callback_count.load();
        std::cout << "Average callback duration: " << average_duration << " ns" << std::endl;
    }

    cleanup();

    return 0;
}