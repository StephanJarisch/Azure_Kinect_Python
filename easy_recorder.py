import argparse
import datetime
import open3d as o3d


class RecorderWithCallback:

    def __init__(self, config, device, filename, align_depth_to_color):
        # Global flags
        self.flag_exit = False
        self.flag_record = False
        self.filename = filename

        self.align_depth_to_color = align_depth_to_color
        self.recorder = o3d.io.AzureKinectRecorder(config, device)
        if not self.recorder.init_sensor():
            raise RuntimeError('Failed to connect to sensor')

    def escape_callback(self, vis):
        self.flag_exit = True
        if self.recorder.is_record_created():
            print('Recording finished.')
        else:
            print('Nothing has been recorded.')
        return False

    def space_callback(self, vis):
        if self.flag_record:
            print('\n')
            print('XXXXXXXXXXXXXXXXXXXXXXXXX')
            print('\n')
            print('Recording PAUSED. '
                  'Press [Space] to continue. '
                  'Press [ESC] to save and exit.')
            print('\n')
            print('XXXXXXXXXXXXXXXXXXXXXXXXX')
            print('\n')
            self.flag_record = False

        elif not self.recorder.is_record_created():
            if self.recorder.open_record(self.filename):
                print('\n')
                print('OKOKOKOKOKOKOKOKOKOK')
                print('\n')
                print('Recording started. '
                      'Press [SPACE] to pause. '
                      'Press [ESC] to save and exit.')
                print('\n')
                print('OKOKOKOKOKOKOKOKOKOK')
                print('\n')
                self.flag_record = True

        else:
            print('Recording resumed, video may be discontinuous. '
                  'Press [SPACE] to pause. '
                  'Press [ESC] to save and exit.')
            self.flag_record = True

        return False

    def run(self):
        glfw_key_escape = 256
        glfw_key_space = 32
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.register_key_callback(glfw_key_escape, self.escape_callback)
        vis.register_key_callback(glfw_key_space, self.space_callback)

        vis.create_window('recorder', 1920, 540)
        print("Recorder initialized. Press [SPACE] to start. "
              "Press [ESC] to save and exit.")

        vis_geometry_added = False
        while not self.flag_exit:
            rgbd = self.recorder.record_frame(self.flag_record,
                                              self.align_depth_to_color)
            if rgbd is None:
                continue

            if not vis_geometry_added:
                vis.add_geometry(rgbd)
                vis_geometry_added = True

            vis.update_geometry(rgbd)
            vis.poll_events()
            vis.update_renderer()

        self.recorder.close_record()


if __name__ == '__main__':
    config = o3d.io.read_azure_kinect_sensor_config("config.txt")

    print("CINF", o3d.camera.PinholeCameraParameters().extrinsic)
    print("CINF", o3d.camera.PinholeCameraParameters().intrinsic.get_focal_length())

    filename = '{date:%Y-%m-%d-%H-%M-%S}.mkv'.format(date=datetime.datetime.now())
    print('Prepare writing to {}'.format(filename))

    device = 0
    if device < 0 or device > 255:
        print('Unsupported device id, fall back to 0')
        device = 0

    print(config)

    r = RecorderWithCallback(config, device, filename, True)
    r.run()
