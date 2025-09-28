# How to use it:
#
#	$ ros2 run yolo_node yolo --ros-args --param verbose:=1

import rclpy
from rclpy.node      	import Node
from rclpy.qos       	import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from ultralytics     import YOLO
from cv_bridge       import CvBridge, CvBridgeError

import cv2

class YoloNode(Node):
	def __init__(self):
		super().__init__('yolo_node')
        
		qos_profile = QoSProfile(
			reliability = QoSReliabilityPolicy.RELIABLE,                     # QoSReliabilityPolicy.BEST_EFFORT  -  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
			durability  = QoSDurabilityPolicy.VOLATILE,    			 		 # RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,  QoSDurabilityPolicy.TRANSIENT_LOCAL
			history     = QoSHistoryPolicy.KEEP_LAST,                        # RMW_QOS_POLICY_HISTORY_KEEP_LAST,
			depth       = 10 )

	# creates a parameter(s) with the name and default value(s)
		verbose_descriptor = ParameterDescriptor(description='Add verbosity to the running node.')
		self.declare_parameter('verbose', 0, verbose_descriptor)
		
		#get parameters        
		self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value

	# Create a new subscriptio
		self.subscriber_sub = self.create_subscription( Image, '/image_raw', self.image_callback, qos_profile)

	# Create new publisher for the image with detections
		self.image_publisher  = self.create_publisher ( Image, '/image_with_detections', qos_profile)   

	# Create new publisher for the detections
		self.detect_publisher = self.create_publisher ( Detection2DArray, '/detection2d', qos_profile)   
        
		self.yolo   = YOLO('yolo11n.pt')   # Load YOLOv11n model
		self.labels = self.yolo.names      # Get the label mapping

		self.bridge = CvBridge()
		if self.verbose == 1: self.get_logger().info("All init. of the yolo_node finished!")

	def image_callback(self, msg):
		# Convert ROS Image message to OpenCV image
		try:
			cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		except CvBridgeError as e:
			if self.verbose == 1:	self.get_logger().info(f"Input imgage acq. error: {e}")
			return

		# Process image with YOLOv8
		results = self.yolo(cv_img)
        
		# Create a Detection2DArray message
		detection_array_msg        = Detection2DArray()
		detection_array_msg.header = msg.header  # Copy header from input image
        
		for result in results:
			# if the yolo detection is executed into CPU() use the following code
			# for box, conf, label in zip(result.boxes.xyxy.numpy(), result.boxes.conf.numpy(), result.boxes.cls.numpy()):
            
			# if the yolo detection is executed on GPU, first moved to cpu and then convert it to numpy array.
			for box, conf, label in zip(result.cpu().boxes.xyxy.numpy(), result.cpu().boxes.conf.numpy(), result.cpu().boxes.cls.numpy()):
                
				x1, y1, x2, y2 = map(float, box) # Convert to float
                
				# Create a Detection2D message
				detection_msg        = Detection2D()
				detection_msg.header = msg.header  # Copy header from input image
                
				# Create an ObjectHypothesisWithPose message for class probability and label id
				hypothesis = ObjectHypothesisWithPose()
				hypothesis.hypothesis.class_id = str(int(label))
				hypothesis.hypothesis.score    = float(conf)
				detection_msg.results.append(hypothesis)
                
				# Set the bounding box
				detection_msg.bbox.center.position.x = (x1 + x2) / 2
				detection_msg.bbox.center.position.y = (y1 + y2) / 2
				detection_msg.bbox.size_x            = x2 - x1
				detection_msg.bbox.size_y            = y2 - y1

				x1, y1, x2, y2 = map(int, box)  # Convert to int for OpenCV functions

				# Draw bounding box, label, and score on the image
				cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
				label_text = f"{self.labels[label]}: {conf:.2f}"
				cv2.putText(cv_img, label_text, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
 
				# Add Detection2D message to Detection2DArray message
				detection_array_msg.detections.append(detection_msg)
        
        
		# Convert the OpenCV image with detections back to a ROS Image message
		image_with_detections_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')

		# Publish the image with detections
		# Publish to image_with_detection topic
		self.image_publisher.publish (image_with_detections_msg)  

		# Publish bounding boxes
		self.detect_publisher.publish (detection_array_msg)

def main(args=None):
	rclpy.init(args=args)

	yolo_node = YoloNode()
	rclpy.spin(yolo_node)

	yolo_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
