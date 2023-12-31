#!/usr/bin/env python3

import rospy
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from cv_bridge import CvBridge
from pathlib import Path
import os
import sys
from rostopic import get_topic_type

from sensor_msgs.msg import Image
from perception_msgs.msg import BoundingBox, BoundingBoxes, TrafficState


# add yolov5 submodule to path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] / "yolov5"
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative path

# import from yolov5 submodules
from models.common import DetectMultiBackend
from utils.general import (
    check_img_size,
    check_requirements,
    non_max_suppression,
    scale_coords
)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device
from utils.augmentations import letterbox


@torch.no_grad()
class Yolov5Detector:
    def __init__(self):
        self.conf_thres = rospy.get_param("~confidence_threshold")
        self.iou_thres = rospy.get_param("~iou_threshold")
        self.agnostic_nms = rospy.get_param("~agnostic_nms")
        self.max_det = rospy.get_param("~maximum_detections")
        self.classes = rospy.get_param("~classes", None)
        self.line_thickness = rospy.get_param("~line_thickness")
        self.view_image = rospy.get_param("~view_image")
        # Initialize weights 
        weights = rospy.get_param("~weights")
        # Initialize model
        self.device = select_device(str(rospy.get_param("~device","")))
        self.model = DetectMultiBackend(weights, device=self.device, dnn=rospy.get_param("~dnn"), data=rospy.get_param("~data"))
        self.stride, self.names, self.pt, self.jit, self.onnx, self.engine = (
            self.model.stride,
            self.model.names,
            self.model.pt,
            self.model.jit,
            self.model.onnx,
            self.model.engine,
        )

        '''
        rospy.loginfo('The value of stride is: %d', self.model.stride)
        rospy.loginfo('The value of names is: %s', self.model.names)
        rospy.loginfo('The value of names is: %s', self.model.pt)
        rospy.loginfo('The value of names is: %s', self.model.jit)
        rospy.loginfo('The value of names is: %s', self.model.onnx)
        rospy.loginfo('The value of names is: %s', self.model.engine)

        [INFO] [1689760378.233546]: The value of stride is: 32
        [INFO] [1689760378.234547]: The value of names is: {0: 'Green', 1: 'Red', 2: 'Yellow'}
        [INFO] [1689760378.235219]: The value of pt is: True
        [INFO] [1689760378.235766]: The value of jit is: False
        [INFO] [1689760378.236231]: The value of onnx is: False
        [INFO] [1689760378.236678]: The value of engine is: False
        '''

        # Setting inference size
        self.img_size = [rospy.get_param("~inference_size_w", 640), rospy.get_param("~inference_size_h",480)]
        self.img_size = check_img_size(self.img_size, s=self.stride)

        # Half
        '''
        Description: Models are converted to half-precision floating point numbers,
                     which can improve memory usage efficiency  
        '''
        self.half = rospy.get_param("~half", False)
        self.half &= (
            self.pt or self.jit or self.onnx or self.engine
        ) and self.device.type != "cpu"  # FP16 supported on limited backends with CUDA
        if self.pt or self.jit:
            self.model.model.half() if self.half else self.model.model.float()
        bs = 1  # batch_size
        cudnn.benchmark = True  # set True to speed up constant image size inference
        # warmup 
        # Warmup model by running inference once
        self.model.warmup()         
        
        # Initialize subscriber to Image topic
        input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~input_image_topic"), blocking = True)
        self.image_sub = rospy.Subscriber(input_image_topic, Image, self.callback, queue_size=1)

        # Initialize prediction publisher
        self.pred_pub = rospy.Publisher(
            rospy.get_param("~output_topic"), BoundingBoxes, queue_size=1
        )

        # Initialize traffic state publisher
        self.traffic_state_pub = rospy.Publisher(
            rospy.get_param("~output_traffic_state_topic"), TrafficState, queue_size=1
        )

        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image")
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                rospy.get_param("~output_image_topic"), Image, queue_size=1
            )
        
        # Initialize CV_Bridge
        self.bridge = CvBridge()

    def callback(self, data):
        """
        Description: Callback function of subscriber
        
        Output: 
        Three topics are published:
        1. /perception/detections (perception_msgs/BoundingBoxes):
            contains the bounding boxes of the detected objects
        2. /perception/traffic_state (perception_msgs/TrafficState): 
            contains the traffic state of the detected objects
        3. /perception/boundingbox_image(sensor_msgs/Image):
            contains the image with bounding boxes
        """
        im = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        im, im0 = self.preprocess(im)

        # Run inference
        im = torch.from_numpy(im).to(self.device) 
        im = im.half() if self.half else im.float()
        im /= 255

        # Add a dimension if the dimension of the image equaπls 3, which means it doesn't have batch
        if len(im.shape) == 3:
            im = im[None]

        pred = self.model(im, augment=False, visualize=False)
        pred = non_max_suppression(
            pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det
        )

        ### To-do move pred to CPU and fill BoundingBox messages
        
        # Process predictions 
        det = pred[0].cpu().numpy()

        bounding_boxes = BoundingBoxes()
        bounding_boxes.header = data.header
        bounding_boxes.image_header = data.header
        
        annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))

        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

            # Write results
            for *xyxy, conf, cls in reversed(det): # the bounding box with higher confidence will be processed first
                bounding_box = BoundingBox()
                traffic_state = TrafficState()
                c = int(cls)
                # Fill in bounding box message
                bounding_box.Class = self.names[c]
                bounding_box.probability = conf 
                bounding_box.xmin = int(xyxy[0])
                bounding_box.ymin = int(xyxy[1])
                bounding_box.xmax = int(xyxy[2])
                bounding_box.ymax = int(xyxy[3])

                # Fill in traffic state message
                # Green: 0, Red: 1, Yellow: 2
                traffic_state.state = self.names[c] == "Red"
                rospy.loginfo("The traffic state is: %s", traffic_state.state)

                # Publish traffic state
                # The range is 0.4 to 0.6 of the image width 
                # Only one traffic light in the middle is used to sentence the traffic state
                if bounding_box.xmin > 0.4 * im0.shape[1] and bounding_box.xmax < 0.6 * im0.shape[1]:
                    self.traffic_state_pub.publish(traffic_state)

                # The range is 0.3 to 0.7 of the image width and the range is 0 to 0.5 of the image height
                # Multiple bounding boxes can be published at the same time.
                if bounding_box.xmin > 0.3 * im0.shape[1] and bounding_box.xmax < 0.7 * im0.shape[1] and bounding_box.ymax < 0.5 * im0.shape[0]:
                    bounding_boxes.bounding_boxes.append(bounding_box)

                # Annotate the image
                if self.publish_image or self.view_image:  # Add bbox to image
                      # integer class
                    label = f"{self.names[c]} {conf:.2f}"
                    annotator.box_label(xyxy, label, color=colors(c, True))       

                ### POPULATE THE DETECTION MESSAGE HERE

            # Stream results
            im0 = annotator.result()

        # Publish prediction
        self.pred_pub.publish(bounding_boxes)

        # Publish & visualize images
        if self.view_image:
            cv2.imshow(str(0), im0)
            cv2.waitKey(1)  # 1 millisecond
        if self.publish_image:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im0, "bgr8"))
        

    def preprocess(self, img):
        """
        Input:
        img: It is the original image without preprocessing.

        Output:
        im: It is the preprocessed image. im is a numpy array
            with the channel arrangement [batch_size, channel, height, width].
            The preprocessing includes various operations, such as resizing the image to the model's input size and converting color channels.
            im is used for model input

        im0: It is the original image without preprocessing.
             It is a numpy array with the channel arrangement [height, width, channel], which is the common BGR (Blue-Green-Red) channel arrangement.
             im0 is used for visualization and further processing
        """

        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]])
        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        return img, img0 


if __name__ == "__main__":

    check_requirements(exclude=("tensorboard", "thop"))
    
    rospy.init_node("yolov5", anonymous=True)
    detector = Yolov5Detector()
    
    rospy.spin()
