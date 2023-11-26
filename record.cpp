#include <fcntl.h>
#include <unistd.h>
#include <liburing.h>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>

io_uring ring;
int output_fd = -1;
std::atomic<long long> total_duration(0); // Total duration in nano seconds
std::atomic<int> callback_count(0); // Number of callbacks

bool setup_io_uring() {
    if (io_uring_queue_init(32, &ring, 0) < 0) {
        ROS_ERROR("Failed to initialize io_uring");
        return false;
    }
    return true;
}

void cleanup_io_uring() {
    io_uring_queue_exit(&ring);
    if (output_fd != -1) {
        close(output_fd);
    }
}

bool open_output_file(const char* filename) {
    output_fd = open(filename, O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (output_fd < 0) {
        ROS_ERROR("Failed to open file");
        return false;
    }
    return true;
}

void write_to_file(const std::string& data) {
    if (output_fd < 0) {
        ROS_ERROR("File descriptor is not valid");
        return;
    }

    char* write_data = new char[data.length()];
    memcpy(write_data, data.c_str(), data.length());

    struct iovec iov = {
        .iov_base = write_data,
        .iov_len = data.length()
    };

    struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
    if (!sqe) {
        ROS_ERROR("Failed to get submission queue entry");
        delete[] write_data;
        return;
    }

    io_uring_prep_writev(sqe, output_fd, &iov, 1, 0);

    sqe->user_data = (uintptr_t)write_data;

    io_uring_submit(&ring);
}

void messageCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    auto start = std::chrono::high_resolution_clock::now();
    std::stringstream ss;
    ss << "Header: " << msg->header.stamp << " Frame ID: " << msg->header.frame_id << "\n";
    ss << "Position: ("
       << msg->pose.position.x << ", "
       << msg->pose.position.y << ", "
       << msg->pose.position.z << ")\n";
    ss << "Orientation: ("
       << msg->pose.orientation.x << ", "
       << msg->pose.orientation.y << ", "
       << msg->pose.orientation.z << ", "
       << msg->pose.orientation.w << ")\n";

    write_to_file(ss.str());

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::nano> duration = end - start;
    total_duration += (long)duration.count();
    callback_count++;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");

    std::cout << "Recording..." << std::endl;

    if (!setup_io_uring()) {
        return 1;
    }

    if (!open_output_file("output.txt")) {
        cleanup_io_uring();
        return 1;
    }

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/vrpn_client_node/omega_truck/pose", 1000, messageCallback);

    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();
    ros::waitForShutdown();

    if (callback_count > 0) {
        double average_duration = static_cast<double>(total_duration.load()) / callback_count.load();
        std::cout << "Average callback duration: " << average_duration << " ns" << std::endl;
    }

    std::cout << "Exiting..." << std::endl;

    cleanup_io_uring();

    return 0;
}
