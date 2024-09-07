import cv2
import open3d as o3d
import numpy as np

from lib.pointcloud import merge_2D_points, pcd_from_np, transform, angular_lookup, angular_from_cartesian, save_pointcloud_threaded, colormap_pcd
from lib.pointcloud import downsample, filter_outliers, filter_by_reference
from lib.file_utils import angles_from_filenames


def process_3D(config, export=True):
    print("Starting 3D processing...")

    # GET ANGLES FROM FILENAMES
    filepaths, angles = angles_from_filenames(config.lidar_dir, name="plane", ext="npy")
    print(f"{len(filepaths)} files found (min: {min(angles)}, max: {max(angles)}).")

    array_3D = merge_2D_points(filepaths, angles=angles, 
                               offset=(0, config.get("3D","Y_OFFSET"), 0), 
                               lidar_offset_angle=config.get("LIDAR","LIDAR_OFFSET_ANGLE"),
                               up_vector=(0,0,1), columns="XZI") 
    
    pcd = pcd_from_np(array_3D, estimate_normals=True, max_nn=50,
                      radius=config.get("3D","NORMAL_RADIUS"))  # radius for normal estimation in mm

    pcd = transform(pcd, translate=(0, 0, config.get("3D","Z_OFFSET")))

    print("Merge completed.")

    # scale pointcloud
    scene_scale = config.get("3D","SCALE")  # mm -> 0.001 m
    if scene_scale !=1:
        pcd = transform(pcd, scale=scene_scale)


    # ANGULAR LOOKUP ("TEXTURING")
    if config.get("ENABLE_VERTEXCOLOUR"):
        colors = angular_lookup(angular_from_cartesian(np.asarray(pcd.points)),  # angular_points
                                cv2.imread(config.pano_path),  # pano
                                scale=config.get("VERTEXCOLOUR","SCALE"), 
                                z_rotate=config.get("VERTEXCOLOUR","Z_ROTATE"))
        
        pcd.colors = o3d.utility.Vector3dVector(np.asarray(colors))
        if export:
            save_pointcloud_threaded(pcd, config.pcd_path, ply_ascii=config.get("3D","ASCII")) 

    else:  # colorize pointcloud by mapping intensities to colormap
        pcd = colormap_pcd(pcd, gamma=1, cmap="viridis")
        if export:
            save_pointcloud_threaded(pcd, config.pcd_path, ply_ascii=config.get("3D","ASCII")) 


    # FILTER OUTLIER POINTS
    if config.get("ENABLE_FILTERING"):
        low_pcd = downsample(pcd, voxel_size=config.get("FILTERING", "VOXEL_SIZE"))

        nb_points = config.get("FILTERING", "NB_POINTS")
        radius = config.get("FILTERING", "RADIUS")
        filtered_low_pcd = filter_outliers(low_pcd, nb_points=nb_points, radius=radius)

        pcd = filter_by_reference(pcd, filtered_low_pcd, radius=radius)
        if export:
            save_pointcloud_threaded(pcd, config.filtered_pcd_path, ply_ascii=config.get("3D","ASCII"))    

    return pcd


if __name__ == "__main__":
    from lib.visualization import visualize
    from lib.config import Config

    
    scans_root = None  # "C:/DOKUMENTE/PiDAR/scans"
    scan_id = "240901-0647"  # "240901-0654"

    config = Config(scans_root=scans_root)
    config.init(scan_id=scan_id)

    # DISABLE TEXTURING AND FILTERING
    config.set(True, "ENABLE_3D")
    config.set(False, "ENABLE_PANO")
    config.set(False, "ENABLE_VERTEXCOLOUR")
    config.set(False, "ENABLE_FILTERING")

    pcd = process_3D(config, export=True)

    if config.platform == 'Windows':
        visualize(pcd, view="front", unlit=True, point_size=1, fullscreen=True)
