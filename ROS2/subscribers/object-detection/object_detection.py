#!/usr/bin/env python
from __future__ import print_function

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from torchvision.models import detection
import numpy as np
import torch
from util import load_classes
import imutils
import datetime
import os

curr_dir = os.path.dirname(os.path.realpath(__file__))  # get's the path of the script
os.chdir(curr_dir)

modelMain = "frcnn-mobilenet"
labelsMain = "coco.names"
confidenceLimit = 0.5
# set the device we will be using to run the model
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# load the list of categories in the COCO dataset and then generate a
# set of bounding box colors for each class
CLASSES = load_classes(labelsMain)
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

MODELS = {
	"frcnn-resnet": detection.fasterrcnn_resnet50_fpn,
	"frcnn-mobilenet": detection.fasterrcnn_mobilenet_v3_large_320_fpn,
	"retinanet": detection.retinanet_resnet50_fpn
}
# load the model and set it to evaluation mode
model = MODELS[modelMain](pretrained=True, progress=True,
	num_classes=len(CLASSES), pretrained_backbone=True).to(DEVICE)
model.eval()

class image_converter(Node):
    def __init__(self):
        super().__init__('object_detection')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.callback,
            1)
        self.subscription  # prevent unused variable warning

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image = imutils.resize(cv_image, width=400)
            orig = image.copy()
            # added
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = image.transpose((2, 0, 1))
            # add the batch dimension, scale the raw pixel intensities to the
            # range [0, 1], and convert the image to a floating point tensor
            image = np.expand_dims(image, axis=0)
            image = image / 255.0
            image = torch.FloatTensor(image)
            # send the input to the device and pass the it through the network to
            # get the detections and predictions
            image = image.to(DEVICE)
            detections = model(image)[0]
            ct2 = datetime.datetime.now()
            # print(detections)

            for i in range(0, len(detections["boxes"])):
                # extract the confidence (i.e., probability) associated with the
                # prediction
                confidence = detections["scores"][i]
                # filter out weak detections by ensuring the confidence is
                # greater than the minimum confidence
                if confidence > confidenceLimit:
                    # extract the index of the class label from the detections,
                    # then compute the (x, y)-coordinates of the bounding box
                    # for the object
                    idx = int(detections["labels"][i]) - 1
                    box = detections["boxes"][i].detach().cpu().numpy()
                    (startX, startY, endX, endY) = box.astype("int")
                    # display the prediction to our terminal
                    label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
                    print("[INFO] {}".format(label))
                    # draw the bounding box and label on the image
                    cv2.rectangle(orig, (startX, startY), (endX, endY),
                                  COLORS[idx], 2)
                    y = startY - 15 if startY - 15 > 15 else startY + 15
                    cv2.putText(orig, label, (startX, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

            cv2.imshow("Output", orig)
            cv2.waitKey(2)
        except CvBridgeError as e:
            print(e)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = image_converter()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()