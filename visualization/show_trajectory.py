import matplotlib.pyplot as plt
import numpy as np
import csv
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def load_trajectory(filename):
    """
    Load the trajectory from a CSV file.

    Args:
        filename(str): The path to the CSV file.

    Returns:
        list of tuple: A list of tuples containing the x, y, z, vx, vy, vz, and t values.
    """
    trajectory = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row
        for row in reader:
            # Assuming the format is: x, y, z, vx, vy, vz, t
            x, y, z, vx, vy, vz, t = map(float, row)
            trajectory.append((x, y, z, vx, vy, vz, t))
    return trajectory

def load_states(filename):
    """
    Load the states from a CSV file.

    Args:
        filename(str): The path to the CSV file.

    Returns:
        list of tuple: A list of tuples containing the x, y, z, vx, vy, vz, and t values.
    """
    states = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row
        for row in reader:
            # Assuming the format is: x, y, z, vx, vy, vz, t
            x, y, z, vx, vy, vz, t = map(float, row)
            states.append((x, y, z, vx, vy, vz, t))
    return states

def load_obstacles(filename):
    """
    Load the obstacles from a CSV file.

    Args:
        filename(str): The path to the CSV file.

    Returns:
        list of tuple: A list of tuples containing the x, y, z, hx, hy, hz values.
    """
    obstacles = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row if present
        for row in reader:
            # Assuming the format is: x, y, z, hx, hy, hz
            x, y, z, hx, hy, hz = map(float, row)
            obstacles.append((x, y, z, hx, hy, hz))
    return obstacles

def plot_trajectory_and_states(trajectory, states, obstacles):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Extracting positions and velocities for the trajectory
    traj_x = [state[0] for state in trajectory]
    traj_y = [state[1] for state in trajectory]
    traj_z = [state[2] for state in trajectory]
    traj_u = [state[3] for state in trajectory]
    traj_v = [state[4] for state in trajectory]
    traj_w = [state[5] for state in trajectory]

    # Extracting positions and velocities for the states
    states_x = [state[0] for state in states]
    states_y = [state[1] for state in states]
    states_z = [state[2] for state in states]
    states_u = [state[3] for state in states]
    states_v = [state[4] for state in states]
    states_w = [state[5] for state in states]

    # Plotting the quiver plot for the search tree states
    ax.quiver(states_x, states_y, states_z, states_u, states_v, states_w, length=10, color='g', alpha=0.5, label='Search Tree')

    # Plotting the quiver plot for the trajectory
    ax.quiver(traj_x, traj_y, traj_z, traj_u, traj_v, traj_w, length=10, color='r', label='Trajectory')

    # Infer the bounding box from the states
    min_x, max_x = min(states_x), max(states_x)
    min_y, max_y = min(states_y), max(states_y)
    min_z, max_z = min(states_z), max(states_z)

    # Filtering and plotting the obstacles as 3D boxes
    for obs in obstacles:
        x, y, z, hx, hy, hz = obs
        if min_x <= x <= max_x and min_y <= y <= max_y and min_z <= z <= max_z:
            # Define the vertices of the box
            r = [
                [x - hx, y - hy, z - hz],
                [x + hx, y - hy, z - hz],
                [x + hx, y + hy, z - hz],
                [x - hx, y + hy, z - hz],
                [x - hx, y - hy, z + hz],
                [x + hx, y - hy, z + hz],
                [x + hx, y + hy, z + hz],
                [x - hx, y + hy, z + hz]
            ]
            # Create a list of sides' polygons
            verts = [
                [r[0], r[1], r[5], r[4]],
                [r[7], r[6], r[2], r[3]],
                [r[0], r[3], r[7], r[4]],
                [r[1], r[2], r[6], r[5]],
                [r[0], r[1], r[2], r[3]],
                [r[4], r[5], r[6], r[7]]
            ]
            ax.add_collection3d(Poly3DCollection(verts, facecolors='b', linewidths=1, edgecolors='r', alpha=.25))

    # Setting labels and limits
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    ax.set_zlim(min_z, max_z)

    plt.title('RRT Trajectory and Search Tree Visualization with Obstacles')
    plt.legend()
    plt.show()

if __name__ == "__main__":

    # Load the trajectory, states, and obstacles
    trajectory = load_trajectory('data/trajectory.csv')

    # Load the states 
    states = load_states('data/states.csv')
    
    # Load the obstacles
    obstacles = load_obstacles('data/obstacles.csv')
    
    # Plot the trajectory and states
    plot_trajectory_and_states(trajectory, states, obstacles)