import matplotlib.pyplot as plt
import numpy as np

import math


def read_poses(file_path: str) -> list:
    """Reads trajectory from text file and returns a list of poses."""
    with open(file_path, 'r') as file:
        lines = file.readlines()
    poses = []
    for line in lines:
        pose = [float(x) for x in line.split(" ")]
        poses.append(pose)
    return poses


def read_times(file_path: str) -> list:
    """Reads times from text file and returns a list of times."""
    with open(file_path, 'r') as file:
        lines = file.readlines()
    times = []
    for line in lines:
        time = float(line)
        times.append(time)
    return times


def plot_xyz_error(poses: list, ground_truth: list, times: list) -> None:
    if len(poses) != len(ground_truth):
        print("Error: poses and ground_truth have different length.")
        return

    error_x = np.absolute(
        np.array([element[3] for element in poses]) - np.array([element[3] for element in ground_truth]))
    error_y = np.absolute(
        np.array([element[7] for element in poses]) - np.array([element[7] for element in ground_truth]))
    error_z = np.absolute(
        np.array([element[11] for element in poses]) - np.array([element[11] for element in ground_truth]))

    fig, (ax1, ax2, ax3) = plt.subplots(3)
    fig.suptitle('XYZ Error')
    ax1.plot(np.array(times), error_x)
    ax2.plot(np.array(times), error_y)
    ax3.plot(np.array(times), error_z)

    ax1.set(xlabel='time', ylabel='error_x (m)')  # y
    ax2.set(xlabel='time', ylabel='error_y (m)')  # z
    ax3.set(xlabel='time', ylabel='error_z (m)')  # x

    plt.show()
    fig.savefig('xyz_error.png')


def plot_rpy_error(poses: list, ground_truth: list, times: list) -> None:
    error_roll = []
    error_pitch = []
    error_yaw = []

    for i in range(len(poses)):
        roll, pitch, yaw = rot2eul(poses[i])
        roll_gt, pitch_gt, yaw_gt = rot2eul(ground_truth[i])
        error_roll.append(abs(roll - roll_gt))
        error_pitch.append(abs(pitch - pitch_gt))
        error_yaw.append(abs(yaw - yaw_gt))

    fig, (ax1, ax2, ax3) = plt.subplots(3)
    fig.suptitle('RPY Error')
    ax1.plot(np.array(times), error_roll)
    ax2.plot(np.array(times), error_pitch)
    ax3.plot(np.array(times), error_yaw)

    ax1.set(xlabel='time', ylabel='error_roll (rad)')
    ax2.set(xlabel='time', ylabel='error_pitch (rad)')
    ax3.set(xlabel='time', ylabel='error_yaw (rad)')

    plt.show()
    fig.savefig('rpy_error.png')


def plot_3d_trajectory(poses: list, ground_truth: list) -> None:
    if len(poses) != len(ground_truth):
        print("Error: poses and ground_truth have different length.")
        return

    x = np.array([element[3] for element in poses])
    y = np.array([element[7] for element in poses])
    z = np.array([element[11] for element in poses])

    x_gt = np.array([element[3] for element in ground_truth])
    y_gt = np.array([element[7] for element in ground_truth])
    z_gt = np.array([element[11] for element in ground_truth])

    fig = plt.figure()

    if max(x) > max(y):
        fig.set_figwidth(max(x) / max(y) * 5)
        fig.set_figheight(1 * 5)
    else:
        fig.set_figwidth(1 * 5)
        fig.set_figheight(max(y) / max(x) * 5)

    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x, y, z, label='Estimated')
    ax.plot(x_gt, y_gt, z_gt, label='Ground Truth')
    ax.legend()

    ax.set(xlabel='x (m)', ylabel='y (m)', zlabel='z (m)')

    plt.show()
    fig.savefig('3d_trajectory.png')


def plot_2d_trajectory(poses: list, ground_truth: list) -> None:
    if len(poses) != len(ground_truth):
        print("Error: poses and ground_truth have different length.")
        return

    x = np.array([element[11] for element in poses])
    y = np.array([element[3] for element in poses])

    x_gt = np.array([element[11] for element in ground_truth])
    y_gt = np.array([element[3] for element in ground_truth])

    fig = plt.figure()

    if max(x) > max(y):
        fig.set_figwidth(max(x) / max(y) * 5)
        fig.set_figheight(1 * 5)
    else:
        fig.set_figwidth(1 * 5)
        fig.set_figheight(max(y) / max(x) * 5)

    ax = fig.add_subplot(111)
    ax.plot(x, y, label='Estimated')
    ax.plot(x_gt, y_gt, label='Ground Truth')
    ax.legend()

    ax.set(xlabel='x (m)', ylabel='y (m)')

    plt.show()
    fig.savefig('2d_trajectory.png')


def rot2eul(pose: list) -> np.array:
    rot = np.array([[pose[0], pose[1], pose[2]],
                    [pose[4], pose[5], pose[6]],
                    [pose[8], pose[9], pose[10]]])

    pitch = -np.arcsin(rot[2, 0])
    roll = np.arctan2(rot[2, 1] / np.cos(pitch), rot[2, 2] / np.cos(pitch))
    yaw = np.arctan2(rot[1, 0] / np.cos(pitch), rot[0, 0] / np.cos(pitch))
    return roll, pitch, yaw
