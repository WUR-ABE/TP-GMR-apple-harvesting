import math as m
import numpy as np


def transform_matrix_3D(a, init):
    # Translation
    dX = - init[0]
    dY = - init[1]
    dZ = - init[2]

    # Rotation
    rC = np.cos(init[3])
    rS = np.sin(init[3])
    pC = np.cos(init[4])
    pS = np.sin(init[4])
    yC = np.cos(init[5])
    yS = np.sin(init[5])

    # Matrices and dot product
    Translate_matrix = np.array([[1, 0, 0, dX],
                                 [0, 1, 0, dY],
                                 [0, 0, 1, dZ],
                                 [0, 0, 0, 1]])
    Rotate_X_matrix = np.array([[1, 0, 0, 0],
                                [0, rC, -rS, 0],
                                [0, rS, rC, 0],
                                [0, 0, 0, 1]])
    Rotate_Y_matrix = np.array([[pC, 0, pS, 0],
                                [0, 1, 0, 0],
                                [-pS, 0, pC, 0],
                                [0, 0, 0, 1]])
    Rotate_Z_matrix = np.array([[yC, -yS, 0, 0],
                                [yS, yC, 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
    rot_3d_matrix = np.dot(Rotate_Z_matrix, np.dot(Rotate_Y_matrix, np.dot(Rotate_X_matrix, Translate_matrix)))

    # Apply translation and rotation
    pos = [a[0], a[1], a[2], 1]
    pos_trans = np.dot(rot_3d_matrix, np.transpose(pos))[0:3]
    ori_trans = a[3:] - init[3:]
    pose_trans = np.concatenate((pos_trans, ori_trans), axis=None)

    return pose_trans


def inv_for_lintrans(pose):
    # Rotation
    rC = np.cos(pose[3])
    rS = np.sin(pose[3])
    pC = np.cos(pose[4])
    pS = np.sin(pose[4])
    yC = np.cos(pose[5])
    yS = np.sin(pose[5])

    Rotate_X_matrix = np.array([[1, 0, 0, 0],
                                [0, rC, -rS, 0],
                                [0, rS, rC, 0],
                                [0, 0, 0, 1]])
    Rotate_Y_matrix = np.array([[pC, 0, pS, 0],
                                [0, 1, 0, 0],
                                [-pS, 0, pC, 0],
                                [0, 0, 0, 1]])
    Rotate_Z_matrix = np.array([[yC, -yS, 0, 0],
                                [yS, yC, 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
    rot_3d_matrix = np.dot(Rotate_Z_matrix, np.dot(Rotate_Y_matrix, Rotate_X_matrix))
    inv_rot_3d_matrix = np.linalg.inv(rot_3d_matrix)

    A = np.identity(n=6)
    A[0:3, 0:3] = inv_rot_3d_matrix[0:3, 0:3]

    # Translation
    b = pose

    return A, b


def pythagoras_3d(pos):
    dist = m.sqrt(pos[0]**2+pos[1]**2+pos[2]**2)
    return dist


def cost_function(demo, repro):
    costsum = 0
    for i in range(len(repro)):
        ## Cost function
        # Initial pose
        diff_init = sum(np.abs(repro[i][0, :] - demo[i][0]))

        # Final pose
        diff_end = sum(np.abs(repro[i][-1, :] - demo[i][-1]))

        # Trajectory length
        diff_repro = np.diff(repro[i], axis=0)
        path_length = sum(np.apply_along_axis(pythagoras_3d, 1, diff_repro))

        # Initial trajectory
        start_repro = np.apply_along_axis(transform_matrix_3D, 1, repro[i], demo[i][0])
        row_index = np.argmax(start_repro[:, 2] > 0.025)
        diff_traj = sum(np.abs(start_repro[row_index, [0, 1, 3, 4]]))

        # Sum costs
        costsum += diff_init
        costsum += diff_end
        costsum += path_length
        costsum += diff_traj

    cost = costsum/len(demo)

    return cost
