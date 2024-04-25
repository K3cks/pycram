#!/usr/bin/env python
import logging

import rospy
import cv2

from threading import Thread
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class HeadDisplayPublisher:
    """
    Publisher to send locally available images to the head display of the hsrb
    """

    def __init__(self, topic="/head_display/receiver"):
        """
        Create a publisher for images

        :param topic: The topic to publish images to. Default is the same as on the visualizer script of the hsrb.
        """
        self.stop_thread = False
        self.thread = None
        self.bridge = CvBridge()

        self.image_path = None
        self.frequency = None

        # Create a publisher for the image
        self.publisher = rospy.Publisher(topic, Image, queue_size=10)

        logging.info("Publisher started")

    def publish_image(self, path, frequency=0.3):
        """
        Publish an image constantly to the given ros topic.

        :param path: Path of the picture to send.
        :param frequency: How often the image should be sent again
        """
        self.image_path = path
        self.frequency = frequency

        self.thread = Thread(target=self._pub)
        self.thread.start()

    def stop_publishing(self):
        """
        Stop the thread that publishes an image.
        """
        if self.thread.is_alive():
            self.stop_thread = True
            self.thread.join()
        else:
            logging.warning("No thread for publishing images is running")

    def _pub(self):
        image = cv2.imread(self.image_path)
        self.stop_thread = False

        if image is not None:
            # Convert the OpenCV image to a ROS image message
            ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")

            while not self.stop_thread:
                # Publish the ROS image message
                self.publisher.publish(ros_image)
                logging.debug("Published image for head display")
                rospy.sleep(self.frequency)
        else:
            logging.error("Failed to load image: %s", self.image_path)
