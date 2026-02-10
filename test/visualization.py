import os
import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

root_dir = os.path.dirname(os.path.abspath(__file__))


def extract_scene_data(json_path):
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    # Camera position in world coordinates: C = -R^T * t
    cam_centers = []
    for cam_id, cam_data in data['cameras'].items():
        E = np.array(cam_data['calib']['extrinsics']) # 3x4 matrix
        R = E[:, :3]
        t = E[:, 3]
        C = -R.T @ t
        cam_centers.append(C)
    
    # World Points
    world_points = np.array(list(data['worldPoints'].values()))
    
    return np.array(cam_centers), world_points


def main():
    noisy_file = os.path.join(root_dir, 'noisy-calibrations-and-world-points.json')
    optimized_file = os.path.join(root_dir, 'answer.json')

    # Load data
    c_noisy, p_noisy = extract_scene_data(noisy_file)
    c_opt, p_opt = extract_scene_data(optimized_file)

    # Setup Plot
    fig = plt.figure(figsize=(12, 12))
    ax = fig.add_subplot(111, projection='3d')

    # Plot World Points (Noisy = Red, Optimized = Blue)
    ax.scatter(p_noisy[:, 0], p_noisy[:, 1], p_noisy[:, 2], 
               c='red', marker='.', s=10, alpha=0.3, label='Points (Noisy)')
    ax.scatter(p_opt[:, 0], p_opt[:, 1], p_opt[:, 2], 
               c='blue', marker='.', s=10, alpha=0.5, label='Points (Optimized)')

    # Plot Camera Centers (Orange triangles = Noisy, Green triangles = Optimized)
    ax.scatter(c_noisy[:, 0], c_noisy[:, 1], c_noisy[:, 2], 
               c='orange', marker='^', s=100, label='Cameras (Noisy)')
    ax.scatter(c_opt[:, 0], c_opt[:, 1], c_opt[:, 2], 
               c='green', marker='^', s=100, label='Cameras (Optimized)')

    # Labels and Legend
    ax.set_xlabel('X (World)')
    ax.set_ylabel('Y (World)')
    ax.set_zlabel('Z (World)')
    ax.set_title('Bundle Adjustment Visualization: Cameras and Points')
    ax.legend(loc='upper right')

    ax.set_aspect('equal', adjustable='box')
    plt.show()


if __name__ == "__main__":
    main()