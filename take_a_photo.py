import time, os
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt


def save_each_photo_seperately(depth_list, color_list, time_list, overwrite=True):

    if overwrite:
        if not os.path.exists("color_frames"):
            os.mkdir("color_frames")
        if not os.path.exists("depth_frames"):
            os.mkdir("depth_frames")

        counter = 0
        for color, depth, time_inner in zip(color_list, depth_list, time_list):
            number = str(counter).zfill(5)
            plt.imsave("color_frames/" + number + "_" + str(time_inner) + "_color.png", color)
            plt.imsave("depth_frames/" + number + "_" + str(time_inner) + "_depth.png", depth)
            counter += 1

    else:
        time_stamp = str(int(time.time()))
        if not os.path.exists(time_stamp):
            os.mkdir(time_stamp)

        if not os.path.exists(time_stamp + "/color_frames"):
            os.mkdir(time_stamp + "/color_frames")
        if not os.path.exists(time_stamp + "/depth_frames"):
            os.mkdir(time_stamp + "/depth_frames")

        counter = 0
        for color, depth, time_inner in zip(color_list, depth_list, time_list):
            number = str(counter).zfill(5)
            plt.imsave(time_stamp + "/color_frames/" + number + "_" + str(time_inner) + "_color.png", color)
            plt.imsave(time_stamp + "/depth_frames/" + number + "_" + str(time_inner) + "_depth.png", depth)
            counter += 1



class AzureKinect:
    def __init__(self):
        self.config = o3d.io.AzureKinectSensorConfig()

        #o3d.io.write_azure_kinect_sensor_config('standard_config.txt', self.config)
        #o3d.io.write_azure_kinect_sensor_config('own_config_confirm.txt', o3d.io.read_azure_kinect_sensor_config("own_config.txt"))
        self.device = 0
        self.align_depth_to_color = 1

    def start(self):
        self.sensor = o3d.io.AzureKinectSensor(o3d.io.read_azure_kinect_sensor_config("config_test.txt"))
        if not self.sensor.connect(self.device):
            raise RuntimeError('Failed to connect to sensor')

    def frames(self):
        counter = 0
        while True:
            rgbd = self.sensor.capture_frame(self.align_depth_to_color)
            if rgbd is None:
                continue
            color, depth = np.asarray(rgbd.color).astype(np.uint8), np.asarray(rgbd.depth).astype(np.float32)


    def take_photo(self, add_pcd=False, just_single=True, number_of_photos=10):
        depth_list = []
        color_list = []
        pcd_list = []
        time_list = []
        counter = 0

        while True:
            rgbd = self.sensor.capture_frame(self.align_depth_to_color)
            time_stamp = time.time()

            if rgbd is None:
                continue
            color, depth = np.asarray(rgbd.color).astype(np.uint8), np.asarray(rgbd.depth).astype(np.float32) / 1000.0

            if add_pcd:
                intrinsic = o3d.camera.PinholeCameraIntrinsic(1280, 720, 601.1693115234375, 600.85931396484375, 637.83624267578125, 363.8018798828125)
                depth = o3d.geometry.Image(depth)
                img = o3d.geometry.Image(color)
                rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(img, depth, depth_scale=1000, convert_rgb_to_intensity=False)

                pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
            else:
                pcd = "empty"

            #append everything
            depth_list.append(depth)
            color_list.append(color)
            pcd_list.append(pcd)
            time_list.append(time_stamp)
            counter += 1

            if just_single:

                return depth_list, color_list, pcd_list, time_list
            else:
                if counter >= number_of_photos:
                    return depth_list, color_list, pcd_list, time_list

    def single_triade(self):
        depth_list, color_list, pcd_list, time_list = cam.take_photo(just_single=True, add_pcd=True)
        plt.imsave("color.png", color_list[0])
        plt.imsave("depth.png", depth_list[0])
        o3d.io.write_point_cloud("POINT_CLOUD.ply", pcd_list[0])





if __name__ == "__main__":
    cam = AzureKinect()
    cam.start()
    cam.single_triade()
    #depth_list, color_list, pcd_list, time_list = cam.take_photo(just_single=False, number_of_photos=2)
    #save_each_photo_seperately(depth_list, color_list, time_list, overwrite=False)
