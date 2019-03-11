import os
import cv2
import numpy as np
import rospy
import tensorflow as tf
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self, model_file):
        # TODO load classifier
        print(model_file)
        self.current_light = TrafficLight.UNKNOWN
        cwd = os.path.dirname(os.path.realpath(__file__))
        model_path = cwd + "/models/" + model_file

        # load frozen tensorflow model
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            with tf.gfile.GFile(model_path, 'rb') as fid:
                od_graph_def = tf.GraphDef()
                od_graph_def.ParseFromString(fid.read())
                tf.import_graph_def(od_graph_def, name='')

        is_site = rospy.get_param('is_site')
        # Create a list of labels
        if is_site:
            self.category_index = {1: {'id': 1, 'name': 'Red'}, 2: {'id': 2, 'name': 'Yellow'},
                                   3: {'id': 3, 'name': 'Green'}, 4: {'id': 4, 'name': 'off'}}
        else:
            self.category_index = {1: {'id': 1, 'name': 'Green'}, 2: {'id': 2, 'name': 'Red'},
                                   3: {'id': 3, 'name': 'Yellow'}, 4: {'id': 4, 'name': 'off'}}

        # create tensorflow session for detection
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        # end
        self.sess = tf.Session(graph=self.detection_graph, config=config)

        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    def get_classification(self, image):

        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # return TrafficLight.RED
        # TODO implement light color prediction
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        (im_width, im_height, _) = image_rgb.shape
        image_np = np.expand_dims(image_rgb, axis=0)

        # Actual detection.
        with self.detection_graph.as_default():
            (boxes, scores, classes, num) = self.sess.run(
                [self.detection_boxes, self.detection_scores,
                 self.detection_classes, self.num_detections],
                feed_dict={self.image_tensor: image_np})

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        self.current_light = TrafficLight.UNKNOWN
        min_score_thresh = .5
        count = 0
        if scores is not None:
            idx_max_score = np.argmax(scores)
            if scores[idx_max_score] > min_score_thresh:
                class_name = self.category_index[classes[idx_max_score]]['name']
                if class_name == 'Red':
                    self.current_light = TrafficLight.RED
                    print("---RED---")
                    count+=1
                elif class_name == 'Yellow':
                    self.current_light = TrafficLight.YELLOW
                    print("---YELLOW---")
                elif class_name == 'Green':
                    self.current_light = TrafficLight.GREEN
                    print("---GREEN---")

        return self.current_light

