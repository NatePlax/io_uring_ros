#include <fcntl.h>
#include <unistd.h>
#include <liburing.h>
#include <string>
#include <array>
#include <sstream>
#include <chrono>
#include <atomic>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

std::array<int, 48> output_fds;
io_uring ring;
std::atomic<long long> total_duration(0);
std::atomic<int> callback_count(0);

bool setup_io_uring() {
    if (io_uring_queue_init(256, &ring, 0) < 0) {
        ROS_ERROR("Failed to initialize io_uring");
        return false;
    }
    return true;
}

bool open_output_files() {
    for (int i = 0; i < output_fds.size(); ++i) {
        std::string filename = "output" + std::to_string(i + 1) + ".txt";
        output_fds[i] = open(filename.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
        if (output_fds[i] < 0) {
            ROS_ERROR("Failed to open file: %s", filename.c_str());
            return false;
        }
    }
    return true;
}

void cleanup_io_uring() {
    io_uring_queue_exit(&ring);
    for (auto& fd : output_fds) {
        if (fd != -1) {
            close(fd);
        }
    }
}

void write_to_file(const std::string& data) {
    char* write_data = new char[data.length()];
    memcpy(write_data, data.c_str(), data.length());

    struct iovec iov = {
        .iov_base = write_data,
        .iov_len = data.length()
    };

    for (auto fd : output_fds) {
        if (fd < 0) continue;

        struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
        if (!sqe) {
            ROS_ERROR("Failed to get submission queue entry");
            delete[] write_data;
            return;
        }

        io_uring_prep_writev(sqe, fd, &iov, 1, 0);
        sqe->user_data = (uintptr_t)write_data;
    }

    io_uring_submit(&ring);
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
    total_duration += (long)duration.count();
    callback_count++;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");

    if (!setup_io_uring()) {
        return 1;
    }

    if (!open_output_files()) {
        cleanup_io_uring();
        return 1;
    }

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/vrpn_client_node/omega_truck/pose", 1000, messageCallback);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();

    if (callback_count > 0) {
        double average_duration = static_cast<double>(total_duration.load()) / callback_count.load();
        std::cout << "Average callback duration: " << average_duration << " ns" << std::endl;
    }

    cleanup_io_uring();

    return 0;
}
