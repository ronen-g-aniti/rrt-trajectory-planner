#include "../headers/rrt.h"
#include "../headers/obstacles.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

/**
 * @brief Saves the trajectory to a CSV file
 * @param path Trajectory to save
 * @param filename Name of the file to save the trajectory
 */
void saveTrajectory(const std::vector<std::vector<double>>& path, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // Write the header
    file << "x,y,z,vx,vy,vz,t\n";

    // Write the data
    for (const auto& state : path) {
        for (size_t i = 0; i < state.size(); ++i) {
            file << state[i];
            if (i < state.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
    }

    file.close();
    std::cout << "Trajectory saved to " << filename << std::endl;
}

/**
 * @brief Saves the states to a CSV file
 * @param states States to save
 * @param filename Name of the file to save the states
 */
void saveStates(const std::vector<std::vector<double>>& states, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // Write the header
    file << "x,y,z,vx,vy,vz,t\n";

    // Write the data
    for (const auto& state : states) {
        for (size_t i = 0; i < state.size(); ++i) {
            file << state[i];
            if (i < state.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
    }

    file.close();
    std::cout << "States saved to " << filename << std::endl;
}

/**
 * @brief Main function
 */
int main(int argc, char* argv[]) {
    if (argc < 7) {
        std::cerr << "Usage: " << argv[0] << " <startX> <startY> <startZ> <goalX> <goalY> <goalZ>" << std::endl;
        return 1;
    }

    // Parse command-line arguments
    double startX = std::stod(argv[1]);
    double startY = std::stod(argv[2]);
    double startZ = std::stod(argv[3]);
    double goalX = std::stod(argv[4]);
    double goalY = std::stod(argv[5]);
    double goalZ = std::stod(argv[6]);

    // Create start and goal position vectors
    std::vector<double> startPos = { startX, startY, startZ };
    std::vector<double> goalPos = { goalX, goalY, goalZ };

    // Define environment and obstacles
    ObstacleParser environment("data/obstacles.csv");

    // Create RRT object
    RRT rrt(environment, startPos, goalPos);

    // Run RRT
    std::vector<std::vector<double>> path = rrt.run();

    // Print the path
    std::cout << "Path:" << std::endl;
    for (const auto& state : path) {
        for (double val : state) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }

    // Save the trajectory to a CSV file
    saveTrajectory(path, "data/trajectory.csv");

    // Save the states to a CSV file
    saveStates(rrt.getStates(), "data/states.csv");

    return 0;
}
