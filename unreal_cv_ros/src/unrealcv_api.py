import unrealcv
import cv2
import numpy as np
import math
import time
import os
import re
from io import BytesIO
import PIL.Image
import sys
from unrealcv.util import ResChecker, time_it
import warnings

"APIs for UnrealCV, a toolkit for using Unreal Engine (UE) in Python."

class UnrealCv_API(object):
    def __init__(self, port, ip, resolution, mode='tcp'):
        # if ip == '127.0.0.1':
        #     self.docker = False
        # else:
        #     self.docker = True
        # self.envdir = env
        self.ip = ip
        self.resolution = resolution
        self.decoder = MsgDecoder(resolution)
        self.checker = ResChecker()
        self.obj_dict = dict()
        self.cam = dict()
        # build a client to connect to the env
        self.client = self.connect(ip, port, mode)
        self.client.message_handler = self.message_handler
        self.init_map()

    def connect(self, ip, port, mode='tcp'):
        client = unrealcv.Client((ip, port))
        client.connect()
        if mode == 'unix':
            if 'linux' in sys.platform and unrealcv.__version__ >= '1.0.0':  # new socket for linux
                unix_socket_path = os.path.join('/tmp/unrealcv_{port}.socket'.format(port=port))  # clean the old socket
                os.remove(unix_socket_path) if os.path.exists(unix_socket_path) else None
                client.disconnect() # disconnect the client for creating a new socket in linux
                time.sleep(2)
                if unix_socket_path is not None and os.path.exists(unix_socket_path):
                    client = unrealcv.Client(unix_socket_path, 'tcp')
                else:
                    client = unrealcv.Client((ip, port)) # reconnect to the tcp socket
                client.connect()
            else:
                warnings.warn('unix socket mode is not supported in this platform, switch to tcp mode.')
        return client

    def init_map(self):
        self.cam = self.get_camera_config()
        self.obj_dict = self.build_color_dict(self.get_objects())

    def camera_info(self):
        return self.cam

    def config_ue(self, resolution=(320, 240), low_quality=False, disable_all_screen_messages=True):
        self.check_connection()
        [w, h] = resolution
        self.client.request(f'vrun setres {w}x{h}w', -1)  # set resolution of the display window
        if disable_all_screen_messages:
            self.client.request('DisableAllScreenMessages', -1)  # disable all screen messages
        if low_quality:
            self.client.request('vrun sg.ShadowQuality 0', -1)  # set shadow quality to low
            self.client.request('vrun sg.TextureQuality 0', -1)  # set texture quality to low
            self.client.request('vrun sg.EffectsQuality 0', -1)  # set effects quality to low
        time.sleep(0.1)

    def message_handler(self, message):
        msg = message
        print(msg)

    def check_connection(self):
        while self.client.isconnected() is False:
            warnings.warn('UnrealCV server is not running. Please try again')
            time.sleep(1)
            self.client.connect()

    def get_camera_config(self):
        num_cameras = self.get_camera_num()
        cam = dict()
        for i in range(num_cameras):
            cam[i] = dict(
                 location=self.get_cam_location(i, syns=False),
                 rotation=self.get_cam_rotation(i, syns=False),
                 fov=self.get_cam_fov(i)
            )
        return cam

    def get_camera_num(self):
        return len(self.client.request('vget /cameras').split())

    def get_objects(self):  # get all objects name in the map
        objects = self.client.request('vget /objects').split()
        return objects

    # batch_functions for multiple commands
    def batch_cmd(self, cmds, decoders, **kwargs):
        # cmds = [cmd1, cmd2, ...]
        # decoder is a list of decoder functions
        res_list = self.client.request(cmds)
        if decoders is None: # vset commands do not decode return
            return res_list
        for i, res in enumerate(res_list):
            res_list[i] = decoders[i](res, **kwargs)
        return res_list

    def save_image(self, cam_id, viewmode, path, return_cmd=False):
        # Note: depth is in npy format
        cmd = f'vget /camera/{cam_id}/{viewmode} {path}'
        if return_cmd:
            return cmd
        # check file extension
        try:
            if viewmode == 'depth':
                expect_extension = 'npy'
            else:
                expect_extension = ['bmp', 'png']
            if not self.checker.is_expected_file_extension(path, expect_extension):
                raise ValueError(f'Invalid file extension for {viewmode} image, it should be {expect_extension}',)
        except ValueError as e:
            if viewmode == 'depth':
                path += '.npy'
            else:
                path += '.png'

        self.client.request(cmd)
        img_dirs = self.client.request(cmd)

        return img_dirs

    def get_image(self, cam_id, viewmode, mode='bmp', return_cmd=False, show=False):
        # cam_id:0 1 2 ...
        # viewmode:lit, normal, object_mask, depth
        # mode: bmp, png, npy
        # Note: depth is in npy format
        if viewmode == 'depth':
            return self.get_depth(cam_id, return_cmd=return_cmd, show=show)
        cmd = f'vget /camera/{cam_id}/{viewmode} {mode}'
        if return_cmd:
            return cmd
        image = self.decoder.decode_img(self.client.request(cmd), mode)
        if show:
            cv2.imshow('image', image)
            cv2.waitKey(1)
        return image

    def get_depth(self, cam_id, inverse=False, return_cmd=False, show=False):  # get depth from unrealcv in npy format
        cmd = f'vget /camera/{cam_id}/depth npy'
        if return_cmd:
            return cmd
        res = self.client.request(cmd)
        depth = self.decoder.decode_depth(res, inverse)
        if show:
            cv2.imshow('image', depth/depth.max())  # normalize the depth image
            cv2.waitKey(10)
        return depth

    def get_image_multicam(self, cam_ids, viewmode='lit', mode='bmp', inverse=True):
        # get image from multiple cameras with the same viewmode
        # viewmode : {'lit', 'depth', 'normal', 'object_mask'}
        # mode : {'bmp', 'npy', 'png'}
        # inverse : whether to inverse the depth
        cmds = [self.get_image(cam_id, viewmode, mode, return_cmd=True) for cam_id in cam_ids]
        decoders = [self.decoder.decode_img for i in cam_ids]
        img_list = self.batch_cmd(cmds, decoders, mode=mode, inverse=inverse)
        return img_list

    def get_image_multimodal(self, cam_id, viewmodes=['lit', 'depth'], modes=['bmp', 'npy']): # get rgb and depth image
        # default is to get RGB-D image
        cmds = [self.get_image(cam_id, viewmode, mode, return_cmd=True) for viewmode, mode in zip(viewmodes, modes)]
        decoders = [self.decoder.decode_map[mode] for mode in modes]
        res = self.batch_cmd(cmds, decoders)
        concat_img = np.concatenate(res, axis=2)
        return concat_img

    def get_img_batch(self, cam_info):
        # get image from multiple cameras with the same viewmode
        # viewmode : {'lit', 'depth', 'normal', 'object_mask'}
        # mode : {'bmp', 'npy', 'png'}
        # inverse : whether to inverse the depth
        # one camera id can be of multiple viewmodes, but one viewmode can only have one encoding mode
        # cam_info : {cam_id: {viewmode: {'mode': 'bmp', 'inverse': True, 'img': None}}}
        cmd_list = []
        # prepare command list
        for cam_id in cam_info.keys():
            for viewmode in cam_info[cam_id].keys():
                mode = cam_info[cam_id][viewmode]['mode']
                cmd_list.append(self.get_image(cam_id, viewmode, mode, return_cmd=True))

        res_list = self.client.request(cmd_list)
        # decode images and store in cam_info
        for cam_id in cam_info.keys():
            for viewmode in cam_info[cam_id].keys():
                mode = cam_info[cam_id][viewmode]['mode']
                inverse = cam_info[cam_id][viewmode]['inverse']
                cam_info[cam_id][viewmode]['img'] = self.decoder.decode_img(res_list.pop(0), mode, inverse)
        return cam_info


    def set_cam_pose(self, cam_id, pose):  # set camera pose, pose = [x, y, z, pitch, yaw, roll]
        [x, y, z, roll, yaw, pitch] = pose
        self.set_cam_rotation(cam_id, [roll, yaw, pitch])
        self.set_cam_location(cam_id, [x, y, z])
        # cmd = f'vset /camera/{cam_id}/pose {x} {y} {z} {pitch} {yaw} {roll}'
        # self.client.request(cmd, -1)
        self.cam[cam_id]['location'] = [x, y, z]
        self.cam[cam_id]['rotation'] = [roll, yaw, pitch]
        

    def get_cam_pose(self, cam_id, mode='hard'):  # get camera pose, pose = [x, y, z, roll, yaw, pitch]
        if mode == 'soft':
            pose = self.cam[cam_id]['location']
            pose.extend(self.cam[cam_id]['rotation'])
            return pose
        if mode == 'hard':
            cmds = [self.get_cam_location(cam_id, return_cmd=True), self.get_cam_rotation(cam_id, return_cmd=True)]
            decoders = [self.decoder.decode_map[self.decoder.cmd2key(cmd)] for cmd in cmds]
            res = self.batch_cmd(cmds, decoders)
            self.cam[cam_id]['location'] = res[0]
            self.cam[cam_id]['rotation'] = res[1]
            return res[0] + res[1]

    def set_cam_fov(self, cam_id, fov):  # set camera field of view (fov)
        if fov == self.cam[cam_id]['fov']:
            return fov
        cmd = f'vset /camera/{cam_id}/fov {fov}'
        self.client.request(cmd, -1)
        self.cam[cam_id]['fov'] = fov
        return fov

    def get_cam_fov(self, cam_id):  # set camera field of view (fov)
        cmd = f'vget /camera/{cam_id}/fov'
        fov = self.client.request(cmd)
        return fov

    def set_cam_location(self, cam_id, loc):  # set camera location, loc=[x,y,z]
        [x, y, z] = loc
        cmd = f'vset /camera/{cam_id}/location {x} {y} {z}'
        self.client.request(cmd, -1)
        self.cam[cam_id]['location'] = loc

    def get_cam_location(self, cam_id, newest=True, return_cmd=False, syns=True):
        # get camera location, loc=[x,y,z]
        # hard mode will get location from unrealcv, soft mode will get location from self.cam
        if newest:
            cmd = f'vget /camera/{cam_id}/location'
            if return_cmd:
                return cmd
            res = None
            while res is None:
                res = self.client.request(cmd)
            res = self.decoder.string2floats(res)
            if syns:
                self.cam[cam_id]['location'] = res
        else:
            return self.cam[cam_id]['location']
        return res

    def set_cam_rotation(self, cam_id, rot, rpy=False):  # set camera rotation, rot = [roll, yaw, pitch]
        if rpy:
            [roll, yaw, pitch] = rot
        else:
            [pitch, yaw, roll] = rot
        cmd = f'vset /camera/{cam_id}/rotation {pitch} {yaw} {roll}'
        self.client.request(cmd, -1)
        self.cam[cam_id]['rotation'] = [pitch, yaw, roll]

    def get_cam_rotation(self, cam_id, newest=True, return_cmd=False, syns=True):
        # get camera rotation, rot = [pitch, yaw, roll]
        # newest mode will get rotation from unrealcv, if not will get rotation from self.cam
        if newest:
            cmd = f'vget /camera/{cam_id}/rotation'
            if return_cmd:
                return cmd
            res = None
            while res is None:
                res = self.client.request(cmd)
            res = [float(i) for i in res.split()]
            if syns:
                self.cam[cam_id]['rotation'] = res
            return res
        else:
            return self.cam[cam_id]['rotation']

    def move_cam(self, cam_id, loc):  # move camera to location with physics simulation
        [x, y, z] = loc
        cmd = f'vset /camera/{cam_id}/moveto {x} {y} {z}'
        self.client.request(cmd)

    def move_cam_forward(self, cam_id, yaw, distance, height=0, pitch=0):
        # move camera as a mobile robot
        # yaw is the delta angle between camera and x axis
        # distance is the absolute distance from the initial location to the target location
        # return the collision information
        yaw_exp = (self.cam[cam_id]['rotation'][1] + yaw) % 360
        pitch_exp = (self.cam[cam_id]['rotation'][0] + pitch) % 360
        assert abs(height) < distance, 'height should be smaller than distance'
        if height != 0:
            distance_plane = np.sqrt(distance**2 - height**2)
        else:
            distance_plane = distance
        delt_x = distance_plane * math.cos(yaw_exp / 180.0 * math.pi)
        delt_y = distance_plane * math.sin(yaw_exp / 180.0 * math.pi)

        location_now = self.get_cam_location(cam_id)
        location_exp = [location_now[0] + delt_x, location_now[1]+delt_y, location_now[2]+height]

        self.move_cam(cam_id, location_exp)
        if yaw != 0 or pitch != 0:
            self.set_cam_rotation(cam_id, [0, yaw_exp, pitch_exp])

        location_now = self.get_cam_location(cam_id)
        error = self.get_distance(location_now, location_exp, 3)

        if error < 10:
            return False
        else:
            return True

    def get_distance(self, pos_now, pos_exp, n=2):  # get distance between two points, n is the dimension
        error = np.array(pos_now[:n]) - np.array(pos_exp[:n])
        distance = np.linalg.norm(error)
        return distance

    def set_keyboard(self, key, duration=0.01):  # Up Down Left Right
        cmd = 'vset /action/keyboard {key} {duration}'
        return self.client.request(cmd.format(key=key, duration=duration), -1)

    def get_obj_color(self, obj, return_cmd=False):  # get object color in object mask, color = [r,g,b]
        cmd = f'vget /object/{obj}/color'
        if return_cmd:
            return cmd
        res = self.client.request(cmd)
        return self.decoder.string2color(res)[:-1]

    def set_obj_color(self, obj, color, return_cmd=False):  # set object color in object mask, color = [r,g,b]
        [r, g, b] = color
        cmd = f'vset /object/{obj}/color {r} {g} {b}'
        self.obj_dict[obj] = color
        if return_cmd:
            return cmd
        self.client.request(cmd, -1)  # -1 means async mode

    def set_obj_location(self, obj, loc):  # set object location, loc=[x,y,z]
        [x, y, z] = loc
        cmd = f'vset /object/{obj}/location {x} {y} {z}'
        self.client.request(cmd, -1)  # -1 means async mode

    def set_obj_rotation(self, obj, rot):  # set object rotation, rot = [roll, yaw, pitch]
        [roll, yaw, pitch] = rot
        cmd = f'vset /object/{obj}/rotation {pitch} {yaw} {roll}'
        self.client.request(cmd, -1)

    def get_mask(self, object_mask, obj, threshold=3):  # get an object's mask
        [r, g, b] = self.obj_dict[obj]
        lower_range = np.array([b-threshold, g-threshold, r-threshold])
        upper_range = np.array([b+threshold, g+threshold, r+threshold])
        mask = cv2.inRange(object_mask, lower_range, upper_range)
        return mask

    def get_bbox(self, object_mask, obj, normalize=True):  # get an object's bounding box
        # get an object's bounding box
        width = object_mask.shape[1]
        height = object_mask.shape[0]
        mask = self.get_mask(object_mask, obj)
        nparray = np.array([[[0, 0]]])
        pixelpointsCV2 = cv2.findNonZero(mask)

        if type(pixelpointsCV2) == type(nparray):  # exist target in image
            x_min = pixelpointsCV2[:, :, 0].min()
            x_max = pixelpointsCV2[:, :, 0].max()
            y_min = pixelpointsCV2[:, :, 1].min()
            y_max = pixelpointsCV2[:, :, 1].max()
            if normalize:
                box = ((x_min/float(width), y_min/float(height)),  # left top
                       (x_max/float(width), y_max/float(height)))  # right down
            else:
                box = [x_min, y_min, x_max-x_min, y_max-y_min]
        else:
            if normalize:
                box = ((0, 0), (0, 0))
            else:
                box = [0, 0, 0, 0]

        return mask, box

    def get_obj_bboxes(self, object_mask, objects, return_dict=False):
        #  get objects' bounding boxes in a image given object list, return a list
        boxes = []
        for obj in objects:
            mask, box = self.get_bbox(object_mask, obj)
            boxes.append(box)
        if return_dict:
            return dict(zip(objects, boxes))
        else:
            return boxes

    def build_color_dict(self, objects, batch=True):  # build a color dictionary for objects
        color_dict = dict()
        if batch:
            cmds = [self.get_obj_color(obj, return_cmd=True) for obj in objects]
            decoders = [self.decoder.string2color for _ in objects]
            res = self.batch_cmd(cmds, decoders)
        else:
            res = [self.get_obj_color(obj) for obj in objects]
        for obj, color in zip(objects, res):
            color_dict[obj] = color
        self.obj_dict = color_dict
        return color_dict

    def get_obj_location(self, obj, return_cmd=False):  # get object location
        cmd = f'vget /object/{obj}/location'
        if return_cmd:
            return cmd
        res = None
        while res is None:
            res = self.client.request(cmd)
        return self.decoder.string2floats(res)

    def get_obj_rotation(self, obj, return_cmd=False):  # get object rotation
        cmd = f'vget /object/{obj}/rotation'
        if return_cmd:
            return cmd
        res = None
        while res is None:
            res = self.client.request(cmd)
        return self.decoder.string2floats(res)

    def get_obj_pose(self, obj):  # get object pose
        cmds = [self.get_obj_location(obj, return_cmd=True), self.get_obj_rotation(obj, return_cmd=True)]
        decoders = [self.decoder.decode_map[self.decoder.cmd2key(cmd)] for cmd in cmds]
        res = self.batch_cmd(cmds, decoders)
        return res[0] + res[1]

    def build_pose_dic(self, objects):  # build a pose dictionary for objects
        pose_dic = dict()
        for obj in objects:
            pose = self.get_obj_location(obj)
            pose.extend(self.get_obj_rotation(obj))
            pose_dic[obj] = pose
        return pose_dic

    def get_obj_bounds(self, obj, return_cmd=False): # get object location
        cmd = f'vget /object/{obj}/bounds'
        if return_cmd:
            return cmd
        res = None
        while res is None:
            res = self.client.request(cmd)
        return self.decoder.string2floats(res)  # min x,y,z  max x,y,z

    def get_obj_size(self, obj, box=True):
        # return the size of the bounding box
        self.set_obj_rotation(obj, [0, 0, 0])  # init
        bounds = self.get_obj_bounds(obj)
        x = bounds[3] - bounds[0]
        y = bounds[4] - bounds[1]
        z = bounds[5] - bounds[2]
        if box:
            return [x, y, z]
        else:
            return x*y*z

    def get_obj_scale(self, obj, return_cmd=False):
        # set object scale
        cmd = f'vget /object/{obj}/scale'
        if return_cmd:
            return cmd
        res = None
        while res is None:
            res = self.client.request(cmd)
        print(obj, res)
        return self.decoder.string2floats(res)  # [scale_x, scale_y, scale_z]

    def set_obj_scale(self, obj, scale=[1, 1, 1], return_cmd=False):
        # set object scale
        [x, y, z] = scale
        cmd = f'vset /object/{obj}/scale {x} {y} {z}'
        if return_cmd:
            return cmd
        self.client.request(cmd, -1)

    def set_hide_obj(self, obj, return_cmd=False):  # hide an object, make it invisible, but still there in physics engine
        cmd = f'vset /object/{obj}/hide'
        if return_cmd:
            return cmd
        self.client.request(cmd, -1)

    def set_show_obj(self, obj, return_cmd=False):  # show an object, make it visible
        cmd = f'vset /object/{obj}/show'
        if return_cmd:
            return cmd
        self.client.request(cmd, -1)

    def set_hide_objects(self, objects):
        cmds = [self.set_hide_obj(obj, return_cmd=True) for obj in objects]
        self.client.request(cmds, -1)

    def set_show_objects(self, objects):
        cmds = [self.set_show_obj(obj, return_cmd=True) for obj in objects]
        self.client.request(cmds, -1)

    def destroy_obj(self, obj): # destroy an object, remove it from the scene
        self.client.request(f'vset /object/{obj}/destroy', -1)
        self.obj_dict.pop(obj)
        # TODO: remove the cameras mounted at the object

    def get_camera_num(self):
        res = self.client.request('vget /cameras')
        return len(res.split())

    def get_camera_list(self):
        res = self.client.request('vget /cameras')
        return res.split()

    def set_new_camera(self):
        res = self.client.request('vset /cameras/spawn')
        cam_id = len(self.cam)
        self.register_camera(cam_id)
        return res   # return the object name of the new camera

    def register_camera(self, cam_id, obj_name=None):
        self.cam[cam_id] = dict(
            obj_name=obj_name,
            location=self.get_cam_location(cam_id, syns=False),
            rotation=self.get_cam_rotation(cam_id, syns=False),
            fov=self.get_cam_fov(cam_id),
        )

    def set_new_obj(self, class_name, obj_name):
        cmd = f'vset /object/spawn {class_name} {obj_name}'
        res = self.client.request(cmd)
        if self.checker.is_error(res):
            warnings.warn(res)
        else:  # add object to the object list, check if new cameras are added
            # assign a random color to the object
            color = np.random.randint(0, 255, 3)
            used_colors = self.obj_dict.values()
            while color in used_colors:
                color = np.random.randint(0, 255, 3)
            self.obj_dict[obj_name] = color
            self.set_obj_color(obj_name, color)
            # check if new cameras are added
            while len(self.cam) < self.get_camera_num():
                self.register_camera(len(self.cam), obj_name)
            return obj_name

    def get_vertex_locations(self, obj, return_cmd=False):
        cmd = f'vget /object/{obj}/vertex_location'
        if return_cmd:
            return cmd
        res = None
        while res is None:
            res = self.client.request(cmd)
        return self.decoder.decode_vertex(res)

    def get_obj_uclass(self, obj, return_cmd=False):
        cmd = f'vget /object/{obj}/uclass_name'
        if return_cmd:
            return cmd
        res = None
        while res is None:
            res = self.client.request(cmd)
        return res

    def set_map(self, map_name, return_cmd=False):  # change to a new level map
        cmd = f'vset /action/game/level {map_name}'
        if return_cmd:
            return cmd
        res = self.client.request(cmd)
        if self.checker.not_error(res):
            self.init_map()

    def set_pause(self, return_cmd=False):
        cmd = f'vset /action/game/pause'
        if return_cmd:
            return cmd
        self.client.request(cmd)

    def set_resume(self, return_cmd=False):
        cmd = f'vset /action/game/resume'
        if return_cmd:
            return cmd
        self.client.request(cmd)

    def get_is_paused(self):
        res = self.client.request('vget /action/game/is_paused')
        return res == 'true'

    def set_global_time_dilation(self, time_dilation, return_cmd=False):
        cmd = f'vrun slomo {time_dilation}'
        if return_cmd:
            return cmd
        self.client.request(cmd, -1)

    def set_max_FPS(self, max_fps, return_cmd=False):
        cmd = f'vrun t.maxFPS {max_fps}'
        if return_cmd:
            return cmd
        self.client.request(cmd, -1)


class MsgDecoder(object):
    def __init__(self, resolution):
        self.resolution = resolution
        self.decode_map = {
            'vertex_location': self.decode_vertex,
            'color': self.string2color,
            'rotation': self.string2floats,
            'location': self.string2floats,
            'bounds': self.string2floats,
            'scale': self.string2floats,
            'png': self.decode_png,
            'bmp': self.decode_bmp,
            'npy': self.decode_npy
        }

    def cmd2key(self, cmd):  # extract the last word of the command as key
        return re.split(r'[/\s]+', cmd)[-1]

    def decode(self, cmd, res):  # universal decode function
        key = self.cmd2key(cmd)
        decode_func = self.decode_map.get(key)
        return decode_func(res)

    def string2list(self, res):
        return res.split()

    def string2floats(self, res):  # decode number
        return [float(i) for i in res.split()]

    def string2color(self, res):  # decode color
        object_rgba = re.findall(r"\d+\.?\d*", res)
        color = [int(i) for i in object_rgba]  # [r,g,b,a]
        return color[:-1]  # [r,g,b]

    def string2vector(self, res):  # decode vector
        res = re.findall(r"[+-]?\d+\.?\d*", res)
        vector = [float(i) for i in res]
        return vector

    def bpstring2floats(self, res):  # decode number
        valuse = re.findall(r'"([\d]+\.?\d*)"', res)
        if len(valuse) == 1:
            return float(valuse[0])
        else:
            return [float(i) for i in valuse]
    def bpvector2floats(self, res):  # decode number
        values = re.findall(r'([XYZ]=\d+\.\d+)', res)
        return [[float(i) for i in value] for value in values]

    def decode_vertex(self, res):  # decode vertex
        # input: string
        # output: list of list of floats
        lines = res.split('\n')
        lines = [line.strip() for line in lines]
        vertex_locations = [list(map(float, line.split())) for line in lines]
        return vertex_locations

    def decode_img(self, res, mode, inverse=False):  # decode image
        if mode == 'png':
            img = self.decode_png(res)
        if mode == 'bmp':
            img = self.decode_bmp(res)
        if mode == 'npy':
            img = self.decode_depth(res, inverse)
        return img

    def decode_png(self, res):  # decode png image
        img = np.asarray(PIL.Image.open(BytesIO(res)))
        img = img[:, :, :-1]  # delete alpha channel
        img = img[:, :, ::-1]  # transpose channel order
        return img

    def decode_bmp(self, res, channel=4):  # decode bmp image
        # TODO: configurable resolution
        img = np.fromstring(res, dtype=np.uint8)
        img = img[-self.resolution[1]*self.resolution[0]*channel:]
        img = img.reshape(self.resolution[1], self.resolution[0], channel)
        return img[:, :, :-1]  # delete alpha channel

    def decode_npy(self, res):  # decode npy image
        img = np.load(BytesIO(res))
        if len(img.shape) == 2:
            img = np.expand_dims(img, axis=-1)
        return img

    def decode_depth(self, res, inverse=False, bytesio=True):  # decode depth image
        if bytesio:
            depth = np.load(BytesIO(res))
        else:
            depth = np.fromstring(res, np.float32)
            depth = depth[-self.resolution[1] * self.resolution[0]:]
            depth = depth.reshape(self.resolution[1], self.resolution[0], 1)
        if inverse:
            depth = 1/depth
        return np.expand_dims(depth, axis=-1)

    def empty(self, res):
        return res