#include "../headers/rrt.h"
#include "../headers/obstacles.h"
#include <iostream>
#include <fstream>
#include <vector>

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
int main() {
   
    // Define environment and obstacles
    ObstacleParser environment("data/obstacles.csv");

    // Define start and goal positions
    std::vector<double> startPos = { 0.0, 0.0, 0.0 };
    std::vector<double> goalPos = { 110.0, -10, 150.0 };

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