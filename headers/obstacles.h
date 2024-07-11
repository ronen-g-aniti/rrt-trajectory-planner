#pragma once

#include <vector>
#include <string>

/**
 * @brief Represents an obstacle in 3D space
 */
class Obstacle {
public:

    /**
     * @brief Constructor
     * @param posX X-coordinate of the obstacle
     * @param posY Y-coordinate of the obstacle
     * @param posZ Z-coordinate of the obstacle
     * @param halfSizeX Half size of the obstacle in the X direction
     * @param halfSizeY Half size of the obstacle in the Y direction
     * @param halfSizeZ Half size of the obstacle in the Z direction
     */
    Obstacle(float posX, float posY, float posZ, float halfSizeX, float halfSizeY, float halfSizeZ);

    /**
     * @brief Checks if a point is in collision with the obstacle
     * @param point Point to check for collision
     * @return True if the point is in collision with the obstacle, false otherwise
     */
    bool isCollision(const std::vector<double>& point) const;
    
    /**
     * @brief Returns the minimum X-coordinate of the obstacle
     * @return Minimum X-coordinate of the obstacle
     */
    float getMinX() const;
    
    /**
     * @brief Returns the maximum X-coordinate of the obstacle
     * @return Maximum X-coordinate of the obstacle
     */
    float getMaxX() const;
    
    /**
     * @brief Returns the minimum Y-coordinate of the obstacle
     * @return Minimum Y-coordinate of the obstacle
     */
    float getMinY() const;
    
    /**
     * @brief Returns the maximum Y-coordinate of the obstacle
     * @return Maximum Y-coordinate of the obstacle
     */
    float getMaxY() const;
    
    /**
     * @brief Returns the minimum Z-coordinate of the obstacle
     * @return Minimum Z-coordinate of the obstacle
     */
    float getMinZ() const;
    
    /**
     * @brief Returns the maximum Z-coordinate of the obstacle
     * @return Maximum Z-coordinate of the obstacle
     */
    float getMaxZ() const;

private:
    float posX; ///< X-coordinate of the obstacle center
    float posY; ///< Y-coordinate of the obstacle center
    float posZ; ///< Z-coordinate of the obstacle center
    float halfSizeX; ///< Half size of the obstacle in the X direction
    float halfSizeY; ///< Half size of the obstacle in the Y direction
    float halfSizeZ; ///< Half size of the obstacle in the Z direction
};

/**
 * @brief Parses a CSV file containing obstacles
 */
class ObstacleParser {
public:

    /**
     * @brief Constructor
     * @param filename Name of the CSV file containing obstacles
     */
    ObstacleParser(const std::string& filename);

    /**
     * @brief Returns the list of obstacles
     * @return List of obstacles
     */
    std::vector<Obstacle> getObstacles() const;
    
    /**
     * @brief Returns the bounds of the obstacles
     * @return Bounds of the obstacles
     */
    const std::vector<float>& getBounds() const;

private:
    void parseCSV(const std::string& filename); ///< Parses the CSV file containing obstacles
    void computeBounds(); ///< Computes the bounds of the obstacles
    std::vector<Obstacle> obstacles; ///< List of obstacles
    std::vector<float> bounds; ///< Bounds of the obstacles
};