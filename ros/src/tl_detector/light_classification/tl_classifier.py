from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import datetime

class TLClassifier(object):
    def __init__(self):
        TrainedModelPath = r'light_classification/model/faster_rcnn_frozen_inference_graph.pb'
        self.graph = tf.Graph()
        self.threshold = .5

        with self.graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(TrainedModelPath, 'rb') as fid:
                od_graph_def.ParseFromString(fid.read())
                tf.import_graph_def(od_graph_def, name='')

            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            self.scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.graph.get_tensor_by_name(
                'num_detections:0')

        self.sess = tf.Session(graph=self.graph)

    def get_classification(self, image):
        """Classifies the Traffic light as red, green, yellow, or unknown
        Args:
            image taken from egovehicle
        Returns:
            int: ID of traffic light color
        """
        with self.graph.as_default():
            img_expand = np.expand_dims(image, axis=0)
            start = datetime.datetime.now()
            (boxes, scores, classes, num_detections) = self.sess.run(
                [self.boxes, self.scores, self.classes, self.num_detections],
                feed_dict={self.image_tensor: img_expand})
            end = datetime.datetime.now()
            c = end - start
            print(c.total_seconds())

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        print('SCORES: ', scores[0])
        print('CLASSES: ', classes[0])

        if scores[0] > self.threshold:
            if classes[0] == 1:
                print('GREEN')
                return TrafficLight.GREEN
            elif classes[0] == 2:
                print('RED')
                return TrafficLight.RED
            elif classes[0] == 3:
                print('YELLOW')
                return TrafficLight.YELLOW

        return TrafficLight.UNKNOWN
