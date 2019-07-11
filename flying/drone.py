import sys
if "E:\\git\\AirSim\\PythonClient" not in sys.path:
    sys.path.append("E:\\git\\AirSim\\PythonClient")
    print(sys.path)
import airsim
from airsim.types import *
import math
import numpy as np

import model_preparation

import tensorflow as tf

#VOA: view, offset, altitude / #PITCH: camera pitch
VOA_MODEL_PATH = [None]*4
PITCH_MODEL_PATH = [None]*4

#Resnet18
VOA_MODEL_PATH[0] = 'C:\\Users\\hoang\\WorkingSpace\\TrainingModels\\keras\\results\\model_resnet18_view-offset-altitude_12.hdf5'
PITCH_MODEL_PATH[0] = 'C:\\Users\\hoang\\WorkingSpace\\TrainingModels\\keras\\results\\model_resnet18_cam-pitch_transfer-learning_08.hdf5' 

#MobileNetV2
VOA_MODEL_PATH[1] = 'C:\\Users\\hoang\\WorkingSpace\\TrainingModels\\keras\\results\\model_mobilenetv2_view-offset-altitude_08.hdf5'
PITCH_MODEL_PATH[1] = 'C:\\Users\\hoang\\WorkingSpace\\TrainingModels\\keras\\results\\model_mobilenetv2_cam-pitch_transfer-learning_09.hdf5' 

#DenseNet121
VOA_MODEL_PATH[2] = 'C:\\Users\\hoang\\WorkingSpace\\TrainingModels\\keras\\results\\model_densenet121_view-offset-altitude_11.hdf5'
PITCH_MODEL_PATH[2] = 'C:\\Users\\hoang\\WorkingSpace\\TrainingModels\\keras\\results\\model_densenet121_cam_pitch_transfer-learning_09.hdf5' 

#NasnetMobile
VOA_MODEL_PATH[3] = 'C:\\Users\\hoang\\WorkingSpace\\TrainingModels\\keras\\results\\model_nasnetmobile_view-offset-altitude_08.hdf5'
PITCH_MODEL_PATH[3] = 'C:\\Users\\hoang\\WorkingSpace\\TrainingModels\\keras\\results\\model_nasnetmobile_cam_pitch_transfer-learning_17.hdf5'

# detection model
PATH_TO_FROZEN_GRAPH = 'C:\\Users\\hoang\\WorkingSpace\\TrainingModels\\object_detection\\models\\frozen_inference_graph.pb'


def toDegree(rad):
    return rad*180.0/math.pi

def toRad(angle):
    return angle*math.pi/180.0


class Drone:
    
    def __init__(self, client):
        self.client = client       
        self.camera_pitch = 0 # in degree
        self.navigation_model = None
        self.detection_graph = None
        
    
    def connect(self):
        print ("drone is starting") 
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        
        
    # model_id: 0 - Resnet18, 1 - MobileNetV2, 2 - DenseNet121, 3 - NasNetMobile
    def load_navigation_model (self, model_id):
        print ("loading navigation model into the drone")
        self.navigation_model = model_preparation.build_model(VOA_MODEL_PATH[model_id], PITCH_MODEL_PATH[model_id], model_id)
        return self.navigation_model
        

    def load_detection_graph (self):
        print ("loading detection model into the drone")
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        
        
    def takeoff(self, z=-1):
        print ("drone is taking off")       
        self.client.takeoffAsync().join()
        # z=-1, move up 1 meter from the starting point
        self.client.moveToZAsync(z, 1).join()
        
    
    # up: negative value, down: positive value
    def change_camera_pitch(self, vertical_changed_angle):
        self.camera_pitch += vertical_changed_angle
        self.client.simSetCameraOrientation("0", airsim.to_quaternion(toRad(-45 - self.camera_pitch), 0, 0)) # default is 45 degree down
    
    
    def observe(self, camera_id):
        response = self.client.simGetImages([airsim.ImageRequest(camera_id, airsim.ImageType.Scene, False, False)])[0]
        img1d_rgba = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        img4d_rgba = img1d_rgba.reshape(512, 512, 4)
        img3d_rbg = img4d_rgba[:, :, :3] #512*512*3
        return img3d_rbg
    
    
    def estimate_position_and_orientation(self, img3d_rbg):
        if (self.navigation_model == None):
            print ("navigation model haven't been loaded, returning dump value")
            return ([0,1,0][0,1,0][0,1,0][0,1,0])
        
        img3d_rbg_reshape = np.reshape(img3d_rbg,[1,512,512,3])
        img_final = img3d_rbg_reshape/255.

        probs = self.navigation_model.predict(img_final)
        offset_prob = probs[0][0]
        view_prob = probs[1][0]
        height_prob = probs[2][0]  
        pitch_prob = probs[3][0]
        
        return (pitch_prob, height_prob, offset_prob, view_prob)
    
    
    def detect_objects(img3d_rbg, object_id):
        img3d_rbg_reshape = np.reshape(img3d_rbg,[1,512,512,3])
        output_dict = run_inference_for_single_image(img3d_rbg_reshape, self.detection_graph)
        n = output_dict['num_detections']

        capture_points = []
        for j in range (0,n):
            if output_dict['detection_scores'][j] > 0.5: 
                print ("{}, {}, {}\n".format(output_dict['detection_classes'][j], 
                                             output_dict['detection_scores'][j], 
                                             output_dict['detection_boxes'][j]))
                class_id = output_dict['detection_classes'][j]
                if class_id == object_id:
                    capture_points.append(output_dict['detection_boxes'][j])

        return capture_points
    
    
    def moveByYawZ(self, yaw_rad_to_change, z_offset, velocity, headless= False, file=None, recording=False): 
        rad_pitch = toRad(self.camera_pitch)
        _, _, cur_yaw_rad = self.getPitchRollYawInRad()

        cur_yaw_degree = toDegree(cur_yaw_rad)
        cur_z = self.getZ()
        
        yaw = cur_yaw_rad + yaw_rad_to_change

        gv = (velocity - z_offset*math.tan(rad_pitch)) * math.cos(rad_pitch)
        z_new = (velocity-z_offset*math.tan(rad_pitch)) * math.sin(rad_pitch) + z_offset/math.cos(rad_pitch)

        vx = gv*math.cos(yaw)
        vy = gv*math.sin(yaw)

        if recording == True:
            file.write('cur_yaw {:.3f}, cur_z {:.3f}\n'.format(cur_yaw_degree, cur_z))
            file.write('headless {}, vx {:.3f}, vy {:.3f}, z_new {:.3f}, yaw {}\n'.format(headless, vx, vy, z_new, -toDegree(yaw_rad_to_change)))

        if headless == True:
            self.client.moveByVelocityZAsync(vx, vy, cur_z+z_new, 0.2, drivetrain=DrivetrainType.ForwardOnly,
                                             yaw_mode = YawMode(False,-toDegree(yaw_rad_to_change))).join()
        else:
            self.client.moveByVelocityZAsync(vx, vy, cur_z+z_new, 0.2, drivetrain=DrivetrainType.ForwardOnly,
                                             yaw_mode = YawMode(False,0)).join()

    
    # right: positive value, left: negative value
    def rotate(self, yaw_degree_to_change):
        _, _, cur_yaw_rad = self.getPitchRollYawInRad()
        cur_yaw_degree = toDegree(cur_yaw_rad)
        self.client.rotateToYawAsync(cur_yaw_degree + yaw_degree_to_change, 1, 1).join()
    
    
    # up: possitive value, down: negative value
    def move_vertically(self, z_to_change):
        self.client.moveToZAsync(self.getZ()-z_to_change, 1).join()
        
    
    def shilf_horizontally(self, right_or_left):
        if (right_or_left == 'right'):
            angle = 90
        else:
            angle = -90
        yaw_rad_to_change = toRad(angle)
        _, _, cur_yaw_rad = self.getPitchRollYawInRad()
        cur_yaw_degree = toDegree(cur_yaw_rad)

        self.moveByYawZ(yaw_rad_to_change, 0, 1, True)

            
    def hover(self):
        print ("hovering")
        # work around to stop the drone, because the hoverAysnc() seems doesn't stop the drone
        self.client.moveByVelocityZAsync(0, 0, self.getZ()-0.25, 0.005, 
                                         drivetrain=DrivetrainType.ForwardOnly, yaw_mode = YawMode(False,0)).join() 
        self.client.hoverAsync().join()
        
            
    def land(self):
        print ("")
        
        
    def getPitchRollYawInRad(self):
        return airsim.to_eularian_angles(self.client.simGetGroundTruthKinematics().orientation)
    
    
    def getZ(self):
        return self.client.simGetGroundTruthKinematics().position.z_val
    
    
    def run_inference_for_single_image(image, graph):
        with graph.as_default():
            with tf.Session() as sess:
                # Get handles to input and output tensors
                ops = tf.get_default_graph().get_operations()
                all_tensor_names = {output.name for op in ops for output in op.outputs}
                tensor_dict = {}
                for key in [
                  'num_detections', 'detection_boxes', 'detection_scores',
                  'detection_classes', 'detection_masks'
                ]:
                    tensor_name = key + ':0'
                    if tensor_name in all_tensor_names:
                        tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(tensor_name)
                if 'detection_masks' in tensor_dict:
                    # The following processing is only for single image
                    detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
                    detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])
                    # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
                    real_num_detection = tf.cast(tensor_dict['num_detections'][0], tf.int32)
                    detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
                    detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
                    detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                        detection_masks, detection_boxes, image.shape[1], image.shape[2])
                    detection_masks_reframed = tf.cast(
                        tf.greater(detection_masks_reframed, 0.5), tf.uint8)
                    # Follow the convention by adding back the batch dimension
                    tensor_dict['detection_masks'] = tf.expand_dims(detection_masks_reframed, 0)
                image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')

                # Run inference
                output_dict = sess.run(tensor_dict, feed_dict={image_tensor: image})

                # all outputs are float32 numpy arrays, so convert types as appropriate
                output_dict['num_detections'] = int(output_dict['num_detections'][0])
                output_dict['detection_classes'] = output_dict[
                    'detection_classes'][0].astype(np.int64)
                output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
                output_dict['detection_scores'] = output_dict['detection_scores'][0]
                if 'detection_masks' in output_dict:
                    output_dict['detection_masks'] = output_dict['detection_masks'][0]
        return output_dict
    