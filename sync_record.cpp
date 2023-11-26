#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include <chrono>
#include <atomic>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

int output_fd = -1; // File descriptor for the output file
std::atomic<long long> total_duration(0); // Total duration in nano seconds
std::atomic<int> callback_count(0); // Number of callbacks

bool open_output_file(const char* filename) {
    output_fd = open(filename, O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (output_fd < 0) {
        ROS_ERROR("Failed to open file");
        return false;
    }
    return true;
}

void close_output_file() {
    if (output_fd != -1) {
        close(output_fd);
    }
}

void write_to_file(const std::string& data) {
    if (output_fd < 0) {
        ROS_ERROR("File descriptor is not valid");
        return;
    }

    ssize_t written = write(output_fd, data.c_str(), data.size());
    if (written < 0) {
        ROS_ERROR("Failed to write to file");
    }
}

void messageCallback(const geometry_msgs::PoseStamped& msg) {
    auto start = std::chrono::high_resolution_clock::now();

    std::stringstream ss;
    ss << "Header: " << msg.header.stamp << " Frame ID: " << msg.header.frame_id << "\n";
    ss << "Position: ("
       << msg.pose.position.x << ", "
       << msg.pose.position.y << ", "
       << msg.pose.position.z << ")\n";
    ss << "Orientation: ("
       << msg.pose.orientation.x << ", "
       << msg.pose.orientation.y << ", "
       << msg.pose.orientation.z << ", "
       << msg.pose.orientation.w << ")\n";

    write_to_file(ss.str());

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::nano> duration = end - start;
    total_duration += (long)duration.count();
    callback_count++;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    
    std::cout << "Recording..." << std::endl;

    if (!open_output_file("output.txt")) {
        return 1;
    }

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/vrpn_client_node/omega_truck/pose", 1000, messageCallback);

    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();
    ros::waitForShutdown();

    close_output_file();

    if (callback_count > 0) {
        std::cout << total_duration.load() << std::endl;
        std::cout << callback_count.load() << std::endl;
        double average_duration = static_cast<double>(total_duration.load()) / callback_count.load();
        std::cout << "Average callback duration: " << average_duration << " ns" << std::endl;
    }

    std::cout << "Exiting..." << std::endl;

    return 0;
}
