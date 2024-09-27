from lib.config import Config
from lib.visualization import visualize
from lib.pointcloud import process_raw


if __name__ == "__main__":

    scan_id = "240824-1230"  # "240927-0040"

    config = Config()
    config.init(scan_id=scan_id)

    # DISABLE TEXTURING AND FILTERING
    config.set(True, "ENABLE_3D")
    config.set(False, "ENABLE_PANO")
    config.set(True, "ENABLE_VERTEXCOLOUR")
    config.set(False, "ENABLE_FILTERING")

    # PROCESS 3D
    pcd = process_raw(config, save=True)

    # VISUALIZATION
    if config.platform == 'Windows':
        visualize(pcd, view="front", unlit=True, point_size=1, fullscreen=True)
