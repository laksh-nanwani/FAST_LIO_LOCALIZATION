import numpy as np
import open3d as o3d
from matplotlib.pyplot import imsave
import yaml
import os

if __name__ == "__main__":
    pcd_path = (
        "/home/laksh/wheelchair-camera-lidar/workspace/src/FAST_LIO_LOCALIZATION/PCD/lab_map_with_outside_corridor.pcd"
    )
    out_path = "/home/laksh/wheelchair-camera-lidar/workspace/src/FAST_LIO_LOCALIZATION/maps"
    z_range = [-1.0, 0.3]
    res = 0.05
    map_name = "lab_map_with_outside_corridor"

    pcd = o3d.io.read_point_cloud(pcd_path)
    # o3d.visualization.draw_geometries([pcd])
    pcd = np.array(pcd.points)

    # mask = np.where((pcd[:, 2] > z_range[0]) & (pcd[:, :2] < z_range[1]))
    # filtered_pcd = pcd[mask]

    x_min = np.min(pcd[:, 0])
    x_max = np.max(pcd[:, 0])
    y_min = np.min(pcd[:, 1])
    y_max = np.max(pcd[:, 1])
    # z_min = np.min(pcd[:, 2])
    # z_max = np.max(pcd[:, 2])
    # print(z_min, z_max, pcd.shape)
    # exit()

    w = int((x_max - x_min) / res) + 1
    h = int((y_max - y_min) / res) + 1

    grid = np.full((h, w), 0.5)

    for p in pcd:
        x_grid = int((p[0] - x_min) / res)
        y_grid = h - 1 - int((p[1] - y_min) / res)

        if p[2] < z_range[0]:
            grid[y_grid, x_grid] = 1

        elif p[2] >= z_range[0] and p[2] <= z_range[1]:
            grid[y_grid, x_grid] = 0

    grid = (grid * 255).astype(int)

    map_dict = {
        "image": f"{map_name}.png",
        "resolution": res,
        "origin": [float(x_min), float(y_min), 0.0],
        "occupied_thresh": 0.6,
        "free_thresh": 0.3,
        "negate": 0,
    }

    imsave(os.path.join(out_path, f"{map_name}.png"), grid, cmap="gray")

    with open(os.path.join(out_path, f"{map_name}.yaml"), "w") as file:
        yaml.dump(map_dict, file, default_flow_style=None)
