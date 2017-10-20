import numpy as np
import tensorflow as tf

import rospy
from styx_msgs.msg import TrafficLight
import label_map_util
import visualization_utils as vis_util
import os
import cv2

CLASSIFICATION_THRESHOLD = 0.5  # Value above which classiciation is a hit


class TLClassifier(object):
    def __init__(self):

        self.saved_image_limit = 500
        self.saved_image_counter = 1
        self.save_images = True
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
        self.path = './light_classification/UTL_label_map.pbtxt'
        print(self.path)

        PATH_TO_LABELS = self.path
        NUM_CLASSES = 3

        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,
                                                                    use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)
        self.count = 1

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image_np_expanded = np.expand_dims(image, axis=0)
        # rospy.logwarn('path %s', self.path)

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

        vis_util.visualize_boxes_and_labels_on_image_array(
            image, boxes, classes, scores,
            self.category_index,
            use_normalized_coordinates=True,
            line_thickness=4)

        if self.save_images == True and self.saved_image_counter <= self.saved_image_limit:
            if not (os.path.exists("./tl_images_infer")):
                os.mkdir("./tl_images_infer")
            cv2.imwrite("./tl_images_infer/infer_image{0:0>4}.jpeg".format(self.saved_image_counter),cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
            self.saved_image_counter += 1

        self.current_light = TrafficLight.UNKNOWN

        if scores is not None and scores[0] > CLASSIFICATION_THRESHOLD:  # If highest score is above 50% it's a hit
            if classes[0] == 1:
                self.current_light = TrafficLight.RED
            elif classes[0] == 2:
                self.current_light = TrafficLight.YELLOW
            elif classes[0] == 3:
                self.current_light = TrafficLight.GREEN

        return self.current_light
