import numpy as np
import os
from itertools import islice


def readPoseFiles(fileneme):
    pose = {}
    with open(fileneme, "r") as f:
        while True:
            next_n_lines = list(islice(f, 5))
            if not next_n_lines:
                break
            # process next_n_lines
            temp = next_n_lines[0].split()
            temp1 = np.array(list(map(lambda x: float(x), next_n_lines[1].split())))
            temp2 = np.array(list(map(lambda x: float(x), next_n_lines[2].split())))
            temp3 = np.array(list(map(lambda x: float(x), next_n_lines[3].split())))
            T = np.vstack((temp1, temp2, temp3))
            pose[int(temp[0])] = T
    return pose


def readNameFile(filename):
    name = {}
    i = 0
    with open(filename, 'r') as f:
        for line in f.readlines():
            temp = line.split()
            name[i] = temp[0]
            i += 1
    return name


def transRo2Qu(pose: dict, name: dict):
    results = []
    results.append("# ground truth trajectory")
    results.append("# file: 'rgbd_dataset_freiburg1_room.bag'")
    results.append("# timestamp tx ty tz qx qy qz qw")
    for i in range(len(pose)):
        data = pose[i]
        # np.split(data, [3], axis=1)
        R, t = np.split(data, [3], axis=1)
        trR = R[0][0] + R[1][1] + R[2][2]
        w = np.sqrt(trR + 1) / 2
        x = (R[2][1] - R[1][2]) / (4 * w)
        y = (R[0][2] - R[2][0]) / (4 * w)
        z = (R[1][0] - R[0][1]) / (4 * w)
        results.append("%s %.5f %.5f %.5f %.5f %.5f %.5f %.5f" % (name[i], t[0], t[1], t[2], x, y, z, w))
