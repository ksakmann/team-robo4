import numpy as np
import tensorflow as tf

import rospy
from styx_msgs.msg import TrafficLight


CLASSIFICATION_THRESHOLD = 0.5 # Value above which classiciation is a hit


class TLClassifier(object):
    def __init__(self):

        self.current_light = TrafficLight.UNKNOWN
        self.model_path = rospy.get_param('~model')

        # Build the model
        self.detection_graph = tf.Graph()
        # create config
        config = tf.ConfigProto()

        # Create the graph
        with self.detection_graph.as_default():
            graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.model_path, 'rb') as fid:
                serialized_graph = fid.read()
                graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(graph_def, name='')
                rospy.loginfo('Loaded frozen tensorflow model: %s', self.model_path)

            # Create a reusable sesion attribute
            self.sess = tf.Session(graph=self.detection_graph, config=config)

        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
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

        image_np_expanded = np.expand_dims(image, axis=0)

        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                t0 = rospy.Time.now()
                out = self.sess.run([self.detection_boxes, self.detection_scores,
                                     self.detection_classes, self.num_detections],
                                    feed_dict={self.image_tensor: image_np_expanded})
                dt = rospy.Time.now() - t0
                rospy.loginfo('Classification CPU Time (s): %f', dt.to_sec())
            
        boxes, scores, classes, num = out

        # create np arrays
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        self.current_light = TrafficLight.UNKNOWN

        if scores is not None and scores[0] > CLASSIFICATION_THRESHOLD: # If highest score is above 50% it's a hit
            if classes[0] == 1:
                self.current_light = TrafficLight.RED
            elif classes[0] == 2:
                self.current_light = TrafficLight.YELLOW
            elif classes[0] == 3:
                self.current_light = TrafficLight.GREEN
            
        return self.current_light
