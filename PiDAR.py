from time import sleep
import os

from process_3D import process_3D

from lib.config import Config
from lib.lidar_driver import Lidar
from lib.a4988_driver import A4988
from lib.rpicam_utils import take_HDR_photo, estimate_camera_parameters
from lib.pano_utils import hugin_stitch


config = Config()
config.init()


# initialize stepper
stepper = A4988(config.get("STEPPER", "pins", "DIR_PIN"), 
                config.get("STEPPER", "pins", "STEP_PIN"), 
                config.get("STEPPER", "pins", "MS_PINS"), 
                delay=config.get("STEPPER", "STEP_DELAY"),
                step_angle=config.get("STEPPER", "STEP_ANGLE"),
                microsteps=config.get("STEPPER", "MICROSTEPS"),
                gear_ratio=config.get("STEPPER", "GEAR_RATIO"))


# initialize lidar
if config.get("ENABLE_LIDAR"):
    lidar = Lidar(config, visualization=None)
    
    # callback function for lidar.read_loop()
    def move_steps_callback():
        stepper.move_steps(config.steps if config.SCAN_ANGLE > 0 else -config.steps)
        lidar.z_angle = stepper.get_current_angle()

    if not config.get("ENABLE_CAM"):
        # wait for lidar to lock rotational speed
        sleep(2)

# MAIN
try:
    # 360° SHOOTING PHOTOS
    if config.get("ENABLE_CAM"):
        # CALIBRATE CAMERA: extract exposure time and gain from exif data, iterate through Red/Blue Gains for custom AWB
        current_exposure_time, current_gain, current_awbgains = estimate_camera_parameters(config)
        # print("[RESULT] AE:", current_exposure_time, "| Gain:", current_gain, "| AWB R:", round(current_awbgains[0],3), "B:", round(current_awbgains[1],3))

        IMGCOUNT = config.get("PANO", "IMGCOUNT")
        for i in range(IMGCOUNT):

            # take HighRes image using fixed values
            current_angle = round(stepper.get_current_angle(), 2)
            filename = f"image_{current_angle}.jpg"
            
            imgpaths = take_HDR_photo(AEB           = config.get("CAM", "AEB"), 
                                      AEB_stops     = config.get("CAM", "AEB_STOPS"),
                                      path          = os.path.join(config.img_dir, filename), 
                                      exposure_time = current_exposure_time, 
                                      gain          = current_gain, 
                                      awbgains      = current_awbgains, 
                                      denoise       = config.get("CAM", "denoise"),
                                      sharpness     = config.get("CAM", "sharpness"),
                                      saturation    = config.get("CAM", "saturation"),
                                      save_raw      = config.get("CAM", "raw"), 
                                      blocking      = True)
            
            config.imglist.extend(imgpaths)
            stepper.move_to_angle((360/IMGCOUNT) * (i+1))
            sleep(1)
        
        stepper.move_to_angle(0)
        stepper.move_steps(1)
        sleep(1)


    # 180° SCAN
    if config.get("ENABLE_LIDAR"):
        lidar.read_loop(callback=move_steps_callback, 
                        max_packages=config.max_packages)
        
        stepper.move_to_angle(0)   # return to 0°
    
    # STITCHING PROCESS (NON-BLOCKING)
    project_path = hugin_stitch(config)
        

finally:
    print("PiDAR STOPPED")
    if config.get("ENABLE_LIDAR"):
        lidar.close()
    stepper.close()

    # 3D PROCESSING
    if config.get("ENABLE_3D"):
        pcd = process_3D(config)

    ## Relay Power off
    # GPIO.output(RELAY_PIN, 0)
    # GPIO.cleanup(RELAY_PIN)
