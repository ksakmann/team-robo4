import numpy as np
import tensorflow as tf

import rospy
from styx_msgs.msg import TrafficLight


# PATH_TO_CKPT = '../../../classifier/models/ssd_mobilenet_v1_coco_udacity/exported_model_dir/frozen_inference_graph.pb'
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

        image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        #TODO implement light color prediction
        # image_np = load_image_into_numpy_array(image)
        image_np_expanded = np.expand_dims(image_np, axis=0)

        with self.detection_graph.as_default():
            with tf.Session(graph=detection_graph) as sess:
                out = self.sess.run([detection_boxes, detection_scores,
                                     detection_classes, num_detections],
                                    feed_dict={self.image_tensor: image_np_expanded})
            
        boxes, scores, classes, num = out

        self.current_light = TrafficLight.UNKNOWN

        if scores[0] > CLASSIFICATION_THRESHOLD: # If highest score is above 50% it's a hit
            if classes[0] == 1:
                self.current_light = TrafficLight.RED
            elif classes[0] == 2:
                self.current_light = TrafficLight.YELLOW
            elif classis[0] == 3:
                self.current_light = TrafficLight.GREEN
            else:
                rospy.logerror('Unrecognized traffic light class detection')
            
        return self.current_light