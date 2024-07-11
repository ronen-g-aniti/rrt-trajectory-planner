#include "../headers/obstacles.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>
#include <algorithm>


/**
 * @brief Represents an obstacle in 3D space
 */
Obstacle::Obstacle(float posX, float posY, float posZ, float halfSizeX, float halfSizeY, float halfSizeZ)
    : posX(posX), posY(posY), posZ(posZ), halfSizeX(halfSizeX), halfSizeY(halfSizeY), halfSizeZ(halfSizeZ) {}

/**
 * @brief Checks if a point is in collision with the obstacle
 * @param point Point to check for collision
 * @return True if the point is in collision with the obstacle, false otherwise
 */
bool Obstacle::isCollision(const std::vector<double>& point) const {
    return (point[0] >= getMinX() && point[0] <= getMaxX() &&
        point[1] >= getMinY() && point[1] <= getMaxY() &&
        point[2] >= getMinZ() && point[2] <= getMaxZ());
}

/**
 * @brief Returns the minimum X-coordinate of the obstacle
 * @return Minimum X-coordinate of the obstacle
 */
float Obstacle::getMinX() const { return posX - halfSizeX; }

/**
 * @brief Returns the maximum X-coordinate of the obstacle
 * @return Maximum X-coordinate of the obstacle
 */
float Obstacle::getMaxX() const { return posX + halfSizeX; }

/**
 * @brief Returns the minimum Y-coordinate of the obstacle
 * @return Minimum Y-coordinate of the obstacle
 */
float Obstacle::getMinY() const { return posY - halfSizeY; }

/**
 * @brief Returns the maximum Y-coordinate of the obstacle
 * @return Maximum Y-coordinate of the obstacle
 */
float Obstacle::getMaxY() const { return posY + halfSizeY; }

/**
 * @brief Returns the minimum Z-coordinate of the obstacle
 * @return Minimum Z-coordinate of the obstacle
 */
float Obstacle::getMinZ() const { return posZ - halfSizeZ; }

/**
 * @brief Returns the maximum Z-coordinate of the obstacle
 * @return Maximum Z-coordinate of the obstacle
 */
float Obstacle::getMaxZ() const { return posZ + halfSizeZ; }

/**
 * @brief Parses a CSV file containing obstacles
 * @param filename Name of the CSV file
 */
ObstacleParser::ObstacleParser(const std::string& filename) {
    parseCSV(filename);
    computeBounds();
}

/**
 * @brief Parses a CSV file containing obstacles
 * @param filename Name of the CSV file
 */
void ObstacleParser::parseCSV(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string token;
        std::vector<float> values;
        while (std::getline(ss, token, ',')) {
            values.push_back(std::stof(token));
        }
        obstacles.emplace_back(values[0], values[1], values[2], values[3], values[4], values[5]);
    }
}

/**
 * @brief Computes the bounds of the obstacles
 */
void ObstacleParser::computeBounds() {
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();
    float minZ = std::numeric_limits<float>::max();
    float maxZ = std::numeric_limits<float>::lowest();

    for (const auto& obstacle : obstacles) {
        minX = std::min(minX, obstacle.getMinX());
        maxX = std::max(maxX, obstacle.getMaxX());
        minY = std::min(minY, obstacle.getMinY());
        maxY = std::max(maxY, obstacle.getMaxY());
        minZ = std::min(minZ, obstacle.getMinZ());
        maxZ = std::max(maxZ, obstacle.getMaxZ());
    }

    bounds = { minX, maxX, minY, maxY, minZ, maxZ };
}

/**
 * @brief Returns the bounds of the obstacles
 * @return Bounds of the obstacles
 */
const std::vector<float>& ObstacleParser::getBounds() const {
    return bounds;
}

/**
 * @brief Returns the list of obstacles
 * @return List of obstacles
 */
std::vector<Obstacle> ObstacleParser::getObstacles() const {
    return obstacles;
}