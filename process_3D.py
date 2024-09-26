import numpy as np
import os

from lib.visualization import visualize
from lib.pointcloud import process_raw, load_raw_scan
from lib.config import Config, get_scan_dict


def get_cartesian_list(filepaths):
    cartesian = []
    for file_path in filepaths:
        data = np.load(file_path)
        cartesian.append(data)
    return cartesian


if __name__ == "__main__":

    scan_id = "240824-1230"

    config = Config()
    config.init(scan_id=scan_id)

    # DISABLE TEXTURING AND FILTERING
    config.set(True, "ENABLE_3D")
    config.set(False, "ENABLE_PANO")
    config.set(True, "ENABLE_VERTEXCOLOUR")
    config.set(False, "ENABLE_FILTERING")


    # load saved data from pickle file || or by merging npy files 
    if os.path.exists(config.raw_path):
        print("ply file found")
        raw_scan = load_raw_scan(config.raw_path)

        print("keys:", raw_scan.keys()) 
        #print(f"{len(raw_scan['cartesian'])} planes (min: {min(raw_scan['z_angles'])}, max: {max(raw_scan['z_angles'])}, shape: {raw_scan['cartesian'][0].shape}")

    else:
        from lib.file_utils import angles_from_filenames
        from lib.pointcloud import save_raw_scan

        filepaths, z_angles = angles_from_filenames(f"{config.scan_id}/lidar", name="plane", ext="npy")
        print(f"{len(filepaths)} files found (min: {min(z_angles)}, max: {max(z_angles)}).")

        cartesian = get_cartesian_list(filepaths)
        raw_scan = get_scan_dict(z_angles, cartesian)
        save_raw_scan(config.raw_path, raw_scan)   


    # MAIN FUNCTION: convert 2D numpy arrays to 3D pointcloud
    print("processing 3D planes...")
    pcd = process_raw(config, raw_scan, save=True)
    print("processing 3D completed.")

    # VISUALIZATION
    if config.platform == 'Windows':
        visualize(pcd, view="front", unlit=True, point_size=1, fullscreen=True)
