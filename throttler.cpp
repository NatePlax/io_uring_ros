#include <iostream>
#include <thread>
#include <vector>
#include <cmath>
#include <atomic>
#include <csignal>
#include <chrono>

std::atomic<unsigned long long> sqrt_count(0); // Global counter for sqrt operations
std::atomic<bool> running(true); // Global flag to control the running of threads

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    running = false; // Set the running flag to false
}

void cpuIntensiveTask() {
    while (running) {
        std::sqrt(rand()); // Perform a CPU-intensive operation
        ++sqrt_count; // Increment the counter
    }
}

int main() {
    signal(SIGINT, signalHandler); // Register signal handler for SIGINT

    unsigned int num_cpus = std::thread::hardware_concurrency();
    std::vector<std::thread> threads;

    auto start_time = std::chrono::high_resolution_clock::now(); // Start time

    std::cout << "Starting CPU throttling on " << num_cpus << " threads." << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;

    for (unsigned int i = 0; i < num_cpus; ++i) {
        threads.emplace_back(cpuIntensiveTask);
    }

    for (auto &t : threads) {
        t.join(); // Wait for all threads to finish
    }

    auto end_time = std::chrono::high_resolution_clock::now(); // End time
    std::chrono::duration<double> running_time = end_time - start_time; // Total running duration

    double sqrt_per_second = sqrt_count.load() / running_time.count();
    std::cout << "Total square roots computed: " << sqrt_count.load() << std::endl;
    std::cout << "Average square roots per second: " << sqrt_per_second << std::endl;

    return 0;
}
