#include "../headers/rrt.h"
#include <cmath>
#include <iostream>
#include <random>
#include <algorithm>
#include <numeric>

/**
 * @brief Constructs an RRT object
 * @param environment Obstacle parser object
 * @param startPos Start position of the drone
 * @param goalPos Goal position of the drone
 * @param goalBias Probability of selecting the goal position as the sample point
 * @param maxSteeringAngleRate Maximum steering angle rate at each time step
 * @param timeStep Time step for integration
 * @param timeInterval Time interval for integration
 * @param speed Speed of the expansion process
 * @param maxIterations Maximum number of iterations for the RRT algorithm
 * @param goalTolerance Tolerance for the goal position
 */
RRT::RRT(ObstacleParser& environment, std::vector<double> startPos, std::vector<double> goalPos,
	double goalBias, double maxSteeringAngleRate, double timeStep, double timeInterval,
	double speed, int maxIterations, double goalTolerance) :
	environment(environment), startPos(startPos), goalPos(goalPos), goalBias(goalBias),
	maxSteerAngle(maxSteeringAngleRate* timeStep), timeStep(timeStep), speed(speed),
	maxIterations(maxIterations), goalTolerance(goalTolerance), timeInterval(timeInterval),
	integrationSteps(static_cast<int>(timeInterval / timeStep)), goalIsFound(false) {

	// Calculate initial attitude vector from start position to goal position
	std::vector<double> initialAttitude(3);
	for (int i = 0; i < 3; ++i) {
		initialAttitude[i] = goalPos[i] - startPos[i];
	}
	double norm = std::sqrt(std::inner_product(initialAttitude.begin(), initialAttitude.end(), initialAttitude.begin(), 0.0));
	std::transform(initialAttitude.begin(), initialAttitude.end(), initialAttitude.begin(), [norm](double val) { return val / norm; });

	// Initial state: position = startPos, attitude = initialAttitude, time = 0
	std::vector<double> startState = { startPos[0], startPos[1], startPos[2], initialAttitude[0], initialAttitude[1], initialAttitude[2], 0.0 };
	states.push_back(startState);
	edges[0] = -1;
	cloud.points.push_back({ startState[0], startState[1], startState[2] });
	kdTree = new myKDTree(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
	kdTree->buildIndex();
}

/**
 * @brief Samples a point with bias towards the goal position
 * @return Sampled point
 */
std::vector<double> RRT::sampleWithBias() {
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_real_distribution<> dis(0.0, 1.0);
	static std::uniform_real_distribution<> disX(environment.getBounds()[0], environment.getBounds()[1]);
	static std::uniform_real_distribution<> disY(environment.getBounds()[2], environment.getBounds()[3]);
	static std::uniform_real_distribution<> disZ(environment.getBounds()[4], environment.getBounds()[5]);

	if (dis(gen) < goalBias) {
		return goalPos;
	}
	else {
		return { disX(gen), disY(gen), disZ(gen) };
	}
}

/**
 * @brief Finds the nearest state in the tree to the query point
 * @param queryPos Query point
 * @return Index of the nearest state
 */
int RRT::findNearestState(const std::vector<double>& queryPos) {
	const double queryPoint[3] = { queryPos[0], queryPos[1], queryPos[2] };
	size_t retIndex;
	double outDistSqr;
	nanoflann::KNNResultSet<double> resultSet(1);
	resultSet.init(&retIndex, &outDistSqr);
	kdTree->findNeighbors(resultSet, &queryPoint[0], nanoflann::SearchParameters());
	
	return static_cast<int>(retIndex);

	
}

/**
 * @brief Integrates forward from the nearest state to the sample point
 * @param nearestState Nearest state in the tree
 * @param samplePoint Sample point
 * @return New state after integration
 */
std::vector<double> RRT::integrateForward(const std::vector<double>& nearestState, const std::vector<double>& samplePoint) {
	std::cout << "Integrating forward from " << nearestState[0] << ", " << nearestState[1] << ", " << nearestState[2] << "\n";
	std::vector<double> currentState = nearestState;
	for (int i = 0; i < integrationSteps; i++) {
		std::vector<double> priorState = currentState;
		currentState = updateState(currentState, samplePoint);

		if (std::sqrt(std::pow(goalPos[0] - currentState[0], 2) +
			std::pow(goalPos[1] - currentState[1], 2) +
			std::pow(goalPos[2] - currentState[2], 2)) < goalTolerance) {
			goalIsFound = true;
			std::cout << "Goal found!" << std::endl;
			return currentState;
		}
		if (isCollision({ currentState[0], currentState[1], currentState[2] }) ||
			!isInsideEnvironment({ currentState[0], currentState[1], currentState[2] })) {

			return priorState;
		}
	}
	return currentState;
}

/**
 * @brief Updates the state of the robot
 * @param nearestState Nearest state in the tree
 * @param samplePoint Sample point
 * @return Updated state of the robot
 */
std::vector<double> RRT::updateState(const std::vector<double>& nearestState, const std::vector<double>& samplePoint) {
	
    // Extract the prior attitude, position, and time from the nearest state
    std::vector<double> priorAttitude(nearestState.begin() + 3, nearestState.begin() + 6);
	
    // Extract the prior position from the nearest state
    std::vector<double> priorPosition(nearestState.begin(), nearestState.begin() + 3);
	
    // Extract the prior time from the nearest state
    double priorTime = nearestState[6];

    // Calculate the unit vector from the prior position to the sample point
	std::vector<double> u1(3);
	for (int i = 0; i < 3; ++i) {
		u1[i] = samplePoint[i] - priorPosition[i];
	}

    // Normalize the unit vector
	double u1_norm = std::sqrt(std::inner_product(u1.begin(), u1.end(), u1.begin(), 0.0));
	std::transform(u1.begin(), u1.end(), u1.begin(), [u1_norm](double val) { return val / u1_norm; });

    // Calculate the rotation axis vector
	std::vector<double> axisVector(3);

    // Calculate the cross product of the prior attitude and the unit vector
	axisVector[0] = priorAttitude[1] * u1[2] - priorAttitude[2] * u1[1];
	axisVector[1] = priorAttitude[2] * u1[0] - priorAttitude[0] * u1[2];
	axisVector[2] = priorAttitude[0] * u1[1] - priorAttitude[1] * u1[0];

    // Normalize the rotation axis vector
	double axisVector_norm = std::sqrt(std::inner_product(axisVector.begin(), axisVector.end(), axisVector.begin(), 0.0));
	
    // If the axis vector is too small, set it to the zero vector
    if (axisVector_norm <= 0.01) {
		axisVector = { 0.0, 0.0, 0.0 };
	}
	else {

        // Normalize the axis vector
		std::transform(axisVector.begin(), axisVector.end(), axisVector.begin(), [axisVector_norm](double val) { return val / axisVector_norm; });
	}

    // Calculate the angle between the prior attitude and the unit vector pointing to the sample point
	double theta = std::acos(std::inner_product(priorAttitude.begin(), priorAttitude.end(), u1.begin(), 0.0));
	
    // Clamp the angle to the maximum steering angle
    double steerAngle = std::clamp(theta, -maxSteerAngle, maxSteerAngle);

    // Calculate the K matrix used in Rodrigues' rotation formula
	std::vector<std::vector<double>> K = {
		{0, -axisVector[2], axisVector[1]},
		{axisVector[2], 0, -axisVector[0]},
		{-axisVector[1], axisVector[0], 0}
	};

    // Initialize the rotation matrix as the identity matrix
	std::vector<std::vector<double>> rotationMatrix = {
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1}
	};

    // If the axis vector is not the zero vector, calculate the rotation matrix
	if (axisVector_norm > 0.01) {
		double sinTheta = std::sin(steerAngle);
		double cosTheta = std::cos(steerAngle);
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				rotationMatrix[i][j] += sinTheta * K[i][j];
				for (int k = 0; k < 3; ++k) {
					rotationMatrix[i][j] += (1 - cosTheta) * K[i][k] * K[k][j];
				}
			}
		}
	}

    // Calculate the new attitude by multiplying the rotation matrix with the prior attitude
 	std::vector<double> newAttitude(3);
    for (int i = 0; i < 3; ++i) {
		newAttitude[i] = 0;
		for (int j = 0; j < 3; ++j) {
			newAttitude[i] += rotationMatrix[i][j] * priorAttitude[j];
		}
	}

    // Normalize the new attitude
	std::vector<double> newPosition(3);
	for (int i = 0; i < 3; ++i) {
		newPosition[i] = priorPosition[i] + speed * timeStep * newAttitude[i];
	}

    // Calculate the new time
	double newTime = priorTime + timeStep;

    // Compute the new state (x, y, z, attitude_x, attitude_y, attitude_z, time)
	std::vector<double> newState = { newPosition[0], newPosition[1], newPosition[2], newAttitude[0], newAttitude[1], newAttitude[2], newTime };

    // Return the new state
	return newState;
}

/**
 * @brief Runs the RRT algorithm
 * @return List of states from the start to the goal if found, otherwise an empty list. 
 * The list of states is in the form of (x, y, z, attitude_x, attitude_y, attitude_z, time).
 */
std::vector<std::vector<double>> RRT::run() {
	for (int i = 0; i < maxIterations; i++) {
		std::vector<double> samplePoint = sampleWithBias();
		int nearestIndex = findNearestState(samplePoint);
		std::vector<double> nearestState = states[nearestIndex];

		std::vector<double> newState = integrateForward(nearestState, samplePoint);
		if (goalIsFound) {
			states.push_back(newState);
			int newIndex = states.size() - 1;
			edges[newIndex] = nearestIndex;
			return constructPath(newIndex);
		}

		states.push_back(newState);
		int newIndex = states.size() - 1;
		edges[newIndex] = nearestIndex;

		cloud.points.push_back({ newState[0], newState[1], newState[2] });
		kdTree->buildIndex();
	}
	std::cerr << "Path not found!" << std::endl;
	return {};
}

/**
 * @brief Constructs the path from the start to the goal
 * @param newIndex Index of the goal state
 * @return List of states from the start to the goal
 */
std::vector<std::vector<double>> RRT::constructPath(int newIndex) {
	std::vector<int> path;
	path.push_back(newIndex);
	int parent = edges[newIndex];
	while (parent != -1) {
		path.push_back(parent);
		parent = edges[parent];
	}
	std::reverse(path.begin(), path.end());

	std::vector<std::vector<double>> pathAsStates;
	for (int idx : path) {
		pathAsStates.push_back(states[idx]);
	}
	return pathAsStates;
}

/**
 * @brief Checks if the point is in collision with any of the obstacles
 * @param point Point to check for collision
 * @return True if the point is in collision, otherwise false
 */
bool RRT::isCollision(const std::vector<double>& point) {
	for (const auto& obstacle : environment.getObstacles()) {
		if (obstacle.isCollision(point)) {
			return true;
		}
	}
	return false;
}

/**
 * @brief Checks if the point is inside the environment
 * @param point Point to check for collision
 * @return True if the point is inside the environment, otherwise false
 */
bool RRT::isInsideEnvironment(const std::vector<double>& point) {
	const auto& bounds = environment.getBounds();
	return (point[0] >= bounds[0] && point[0] <= bounds[1] &&
		point[1] >= bounds[2] && point[1] <= bounds[3] &&
		point[2] >= bounds[4] && point[2] <= bounds[5]);
}