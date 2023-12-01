#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include <chrono>
#include <atomic>
#include <array>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

std::array<int, 48> output_fds;
std::atomic<long long> total_duration(0);
std::atomic<int> callback_count(0);

bool open_output_files() {
    for (size_t i = 0; i < output_fds.size(); ++i) {
        std::string filename = "output" + std::to_string(i + 1) + ".txt";
        output_fds[i] = open(filename.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
        if (output_fds[i] < 0) {
            ROS_ERROR("Failed to open file: %s", filename.c_str());
            return false;
        }
    }
    return true;
}

void close_output_files() {
    for (auto& fd : output_fds) {
        if (fd != -1) {
            close(fd);
        }
    }
}

void write_to_file(const std::string& data) {
    for (auto fd : output_fds) {
        if (fd < 0) {
            //ROS_ERROR("File descriptor is not valid");
            std::cout << "Invalid fd" << std::endl;
            continue;
        }

        ssize_t written = write(fd, data.c_str(), data.size());
        if (written < 0) {
            //ROS_ERROR("Failed to write to file");
            std::cout << "write failed" << std::endl;
        }
    }
}

void messageCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    auto start = std::chrono::high_resolution_clock::now();

    std::stringstream ss;
    ss << "Header: " << msg->header.stamp << " Frame ID: " << msg->header.frame_id << "\n";
    ss << "Position: (" << msg->pose.position.x << ", " << msg->pose.position.y << ", " << msg->pose.position.z << ")\n";
    ss << "Orientation: (" << msg->pose.orientation.x << ", " << msg->pose.orientation.y << ", " << msg->pose.orientation.z << ", " << msg->pose.orientation.w << ")\n";

    for (int i = 0; i < 1; i++) {
        write_to_file(ss.str());
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::nano> duration = end - start;
    total_duration += static_cast<long>(duration.count());
    callback_count++;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");

    std::cout << "Recording..." << std::endl;

    if (!open_output_files()) {
        return 1;
    }

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/vrpn_client_node/omega_truck/pose", 1000, messageCallback);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();

    
    for (auto& fd : output_fds) {
        if (fd != -1) {
            fsync(fd);
        }
    }


    close_output_files();

    if (callback_count > 0) {
        std::cout << total_duration.load() << std::endl;
        std::cout << callback_count.load() << std::endl;
        double average_duration = static_cast<double>(total_duration.load()) / callback_count.load();
        std::cout << "Average callback duration: " << average_duration << " ns" << std::endl;
    }

    std::cout << "Exiting..." << std::endl;

    return 0;
}
