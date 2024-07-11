#pragma once

#include <vector>
#include <map>
#include <nanoflann.hpp>
#include "obstacles.h"
#include <cmath>

#ifndef M_PI 
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief Represents a 3D point cloud for KDTree
 */
struct PointCloud {

    /**
     * @brief Represents a 3D point
     */
	struct Point {
		double x, y, z;
	};

	std::vector<Point> points; ///< List of points in the point cloud 

    /**
     * @brief Returns the number of points in the point cloud
     * @return Number of points in the point cloud
     */
	inline size_t kdtree_get_point_count() const { return points.size(); }

    /** 
     * @brief Returns the coordinate of a point in the specified dimension
     * @param idx Index of the point
     * @param dim Dimension of the point (0, 1, 2)
     * @return Coordinate of the point in the specified dimension
     */
	inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
		if (dim == 0) return points[idx].x;
		else if (dim == 1) return points[idx].y;
		else return points[idx].z;
	}

    /**
     * @brief Dummy bounding box computation function.
     * @return Always returns false
     */
	template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
};

typedef nanoflann::KDTreeSingleIndexAdaptor<
	nanoflann::L2_Simple_Adaptor<double, PointCloud>,
	PointCloud,
	3
> myKDTree;


/**
 * @brief Represents a Rapidly-exploring Random Tree (RRT) algorithm
 */
class RRT {
public:
    
        /**
        * @brief Constructs an RRT object
        * @param environment Environment with obstacles
        * @param startPos Starting position of the robot
        * @param goalPos Goal position of the robot
        * @param goalBias Probability of selecting the goal as the sample point
        * @param maxSteeringAngleRate Maximum steering angle rate of the robot
        * @param timeStep Time step for integration
        * @param timeInterval Time interval for the robot to reach the goal
        * @param speed Speed of the robot
        * @param maxIterations Maximum number of iterations
        * @param goalTolerance Tolerance for the goal
        */
	RRT(ObstacleParser& environment, std::vector<double> startPos, std::vector<double> goalPos,
		double goalBias = 0.20, double maxSteeringAngleRate = M_PI / 24, double timeStep = 0.1,
		double timeInterval = 2.0, double speed = 2.0, int maxIterations = 10000, double goalTolerance = 1.0);


    /**
     * @brief Runs the RRT algorithm
     * @return List of states from the start to the goal
     */
	std::vector<std::vector<double>> run();

    /**
     * @brief Returns the states of the robot
     * @return List of states of the robot
     */
	std::vector<std::vector<double>> getStates() const { return states; }

private:
	ObstacleParser& environment; ///< Reference to the environment with obstacles
	std::vector<double> startPos; ///< Starting position of the robot
	std::vector<double> goalPos; ///< Goal position of the robot
	double goalBias; ///< Probability of selecting the goal as the sample point
	double maxSteerAngle; ///< Maximum steering angle of the tree expansion
	double timeStep; ///< Time step for Euler integration
	double speed; ///< Speed of the expansion process
	int maxIterations; ///< Maximum number of iterations before giving up
	double goalTolerance; ///< Tolerance for the goal
	double timeInterval; ///< Maximum time allocated to each branch expansion
	int integrationSteps; ///< Maximum nunmber of integration steps allocated to each branch expansion
	bool goalIsFound; ///< Flag to indicate if the goal is found
	std::vector<std::vector<double>> states; ///< List of explored states
	std::map<int, int> edges; ///< Edges of the tree
	PointCloud cloud; ///< Point cloud for KDTree
	myKDTree* kdTree; ///< KDTree for nearest neighbor search

    /**
     * @brief Samples a point in the environment
     * @return Sampled point
     */
	std::vector<double> sampleWithBias();

    /**
     * @brief Finds the nearest state in the tree to the query point
     * @param queryPos Query point
     * @return Nearest state in the tree
     */
	int findNearestState(const std::vector<double>& queryPos);

    /**
     * @brief Integrates the search node forward from the nearest state to the sample point
     * @param nearestState Nearest state in the tree
     * @param samplePoint Sample point
     * @return List of states from the nearest state to the sample point
     */
	std::vector<double> integrateForward(const std::vector<double>& nearestState, const std::vector<double>& samplePoint);

    /**
     * @brief Updates the state of the robot
     * @param nearestState Nearest state in the tree
     * @param samplePoint Sample point
     * @return Updated state of the robot
     */
	std::vector<double> updateState(const std::vector<double>& nearestState, const std::vector<double>& samplePoint);

    /**
     * @brief Checks if the search node is in collision with the environment
     * @param point Point to check for collision
     * @return True if the robot is in collision with the environment, false otherwise
     */
	bool isCollision(const std::vector<double>& point);

    /**
     * @brief Checks if the search node is in collision with the environment
     * @param point Point to check for collision
     * @return True if the robot is in collision with the environment, false otherwise
     */
	bool isInsideEnvironment(const std::vector<double>& point);

    /**
     * @brief Checks if the search node is in collision with the environment
     * @param point Point to check for collision
     * @return True if the robot is in collision with the environment, false otherwise
     */
	std::vector<std::vector<double>> constructPath(int newIndex);

};