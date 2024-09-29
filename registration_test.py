import numpy as np
import open3d as o3d

from lib.pointcloud import load_pointcloud, save_pointcloud, get_transform_vectors, transform
from lib.registration import global_registration, icp, color_icp
from lib.visualization import visualize


def run_registration():
    pass



if __name__ == "__main__":

    view={"zoom": 0.1, "front": (-1, 1, 0.5), "lookat": (0, 0, 0), "up": (0, 0, 1)}
    point_size = 1

    pcd1 = load_pointcloud("scans/240928-0912/240928-0912.ply")
    pcd2 = load_pointcloud("scans/240928-0922/240928-0922.ply")
    pcd3 = load_pointcloud("scans/240928-0944/240928-0944.ply")
    pcd4 = load_pointcloud("scans/240928-0948/240928-0948.ply")

    source = pcd1
    target = pcd2


    # ------------------------------------------------
    # GLOBAL REGISTRATION
    voxel_size = 0.05  # means 5cm for this dataset
    fast_gr = False
    p2l_icp = True


    reg_gr = global_registration(source, target, voxel_size, fast=fast_gr)
    visualize([source, target], transformation=reg_gr.transformation, point_colors="uniform", view=view, point_size=point_size)


    ransac_translation, ransac_euler = get_transform_vectors(reg_gr.transformation)
    print(f"[RANSAC] translate:\t{ransac_translation})")
    print(f"[RANSAC] rotate:\t{ransac_euler})")


    # # evaluate with voxel size of ICP
    # evaluation = o3d.pipelines.registration.evaluate_registration(source, target, voxel_size*0.4, reg_gr.transformation)
    # print("GR:", evaluation)


    # # ------------------------------------------------
    # # ICP REGISTRATION
    # reg_icp = icp(source, target, reg_gr.transformation, voxel_size, p2l=p2l_icp)
    # print("ICP:", reg_icp)
    # visualize([source, target], transformation=reg_icp.transformation, point_colors="uniform", view=view, point_size=point_size)

    
    # # EXPORT TRANSFORMED POINT CLOUDS
    # transformed_source = transform(source, transformation=reg_icp.transformation)
    # save_pointcloud(transformed_source + target, "scans/registration_test.ply")


    # # ------------------------------------------------
    # # COLORED POINT CLOUD REGISTRATION

    # visualize([source, target], view=view, transformation=reg_gr.transformation, point_size=point_size)
    
    # reg_color = color_icp(source, target,reg_gr.transformation)
    # #print(result_icp)
    # visualize([source, target], transformation=reg_color.transformation, point_colors="uniform", view=view, point_size=point_size)
