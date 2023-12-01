#include <iostream>
#include <thread>
#include <vector>
#include <cmath>
#include <atomic>
#include <csignal>
#include <chrono>
#include <cstdlib>

std::atomic<unsigned long long> sqrt_count(0);
std::atomic<bool> running(true); 

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    running = false;
}

void cpuIntensiveTask() {
    while (running) {
        std::sqrt(rand());
        ++sqrt_count;
    }
}

void runExternalProgram(const std::string &command) {
    std::system(command.c_str());
}

int main(int argc, char *argv[]) {
    signal(SIGINT, signalHandler);
    std::thread program1(runExternalProgram, "./" + std::string(argv[1]));
    std::thread program2(runExternalProgram, "rosbag play " + std::string(argv[2]));

    unsigned int num_cpus = std::thread::hardware_concurrency();
    std::vector<std::thread> threads;

    auto start_time = std::chrono::high_resolution_clock::now();

    std::cout << "Starting CPU throttling on " << num_cpus << " threads." << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;

    for (unsigned int i = 0; i < num_cpus; ++i) {
        threads.emplace_back(cpuIntensiveTask);
    }

    if (program1.joinable()) {
        program1.join();
        running = false;
        std::cout << "done running!" << std::endl;
    }
    if (program2.joinable()) {
        program2.join();
        running = false;
        std::cout << "done running 2" << std::endl;
    }

    for (auto &t : threads) {
        t.join();
    }


    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> running_time = end_time - start_time;

    double sqrt_per_second = sqrt_count.load() / running_time.count();
    std::cout << "Total square roots computed: " << sqrt_count.load() << std::endl;
    std::cout << "Average square roots per second: " << sqrt_per_second << std::endl;

    return 0;
}
