import utils

if __name__ == '__main__':

    poses = utils.read_poses("datas/poses.txt")
    ground_truth = utils.read_poses("datas/gt.txt")
    times = utils.read_times("datas/times.txt")

    utils.plot_xyz_error(poses, ground_truth, times)
    utils.plot_rpy_error(poses, ground_truth, times)
    utils.plot_3d_trajectory(poses, ground_truth)
    utils.plot_2d_trajectory(poses, ground_truth)
