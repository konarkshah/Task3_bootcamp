import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
import cv2
import numpy as np
import os
import math
import message_filters

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.get_logger().info('Perception Node Started (Synchronized)')

        # Parameters
        self.declare_parameter('model_path', 'models/yolov5s.onnx')
        self.declare_parameter('confidence_threshold', 0.5)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        # Load YOLOv5 ONNX Model (Replaced Caffe due to OpenCV 5 deprecation)
        try:
            self.net = cv2.dnn.readNet(model_path)
            self.get_logger().info('YOLOv5 Model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            self.net = None

        # COCO 80 Classes for YOLOv5
        self.CLASSES = [
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
            "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
            "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
            "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
            "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
            "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
            "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
            "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
            "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
            "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
        ]

        # Subscribers with Message Filters for Synchronization
        self.rgb_sub = message_filters.Subscriber(self, Image, 'camera/rgb/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, 'camera/depth/image_raw')
        self.info_sub = message_filters.Subscriber(self, CameraInfo, 'camera/rgb/camera_info')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.info_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)

        self.detection_pub = self.create_publisher(Detection3DArray, 'perception/detections_3d', 10)

        self.tracks = {}
        self.next_track_id = 0

    def imgmsg_to_cv2(self, msg, encoding="bgr8"):
        """ Bypasses cv_bridge entirely to avoid NumPy 2.x crash """
        if encoding == "bgr8":
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        elif encoding == "passthrough":
            if msg.encoding == "16UC1":
                return np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            elif msg.encoding == "32FC1":
                return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        return None

    def sync_callback(self, rgb_msg, depth_msg, info_msg):
        if self.net is None:
            return

        try:
            frame = self.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_image = self.imgmsg_to_cv2(depth_msg, "passthrough")
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return
            
        if frame is None or depth_image is None:
            return

        h_rgb, w_rgb = frame.shape[:2]
        h_depth, w_depth = depth_image.shape[:2]

        # YOLOv5 inference (640x640 input)
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (640, 640), swapRB=True, crop=False)
        self.net.setInput(blob)
        preds = self.net.forward()[0] # Shape: (25200, 85)

        detection_array = Detection3DArray()
        detection_array.header = rgb_msg.header

        current_centroids = []
        
        # Filter by confidence
        mask = preds[:, 4] > self.conf_threshold
        filtered_preds = preds[mask]
        
        for pred in filtered_preds:
            cx, cy, w, h, conf = pred[:5]
            class_scores = pred[5:]
            class_id = np.argmax(class_scores)
            class_conf = class_scores[class_id]
            
            if conf * class_conf > self.conf_threshold:
                class_name = self.CLASSES[class_id]
                
                # Scale from 640x640 back to original image
                orig_cx = int(cx * (w_rgb / 640.0))
                orig_cy = int(cy * (h_rgb / 640.0))
                
                # Scale coordinates to Depth image dimensions if they differ
                depth_cX = int(orig_cx * (w_depth / w_rgb))
                depth_cY = int(orig_cy * (h_depth / h_rgb))
                
                # Clamp depth coordinates
                depth_cX = min(w_depth - 1, max(0, depth_cX))
                depth_cY = min(h_depth - 1, max(0, depth_cY))

                # Get Depth
                depth = float(depth_image[depth_cY, depth_cX])
                if depth_image.dtype == np.uint16:
                    depth *= 0.001
                
                if math.isnan(depth) or depth <= 0 or math.isinf(depth):
                    continue # Invalid depth

                # 3D Projection using Camera Intrinsics
                fx = info_msg.k[0]
                fy = info_msg.k[4]
                cam_cx = info_msg.k[2]
                cam_cy = info_msg.k[5]

                x3d = (orig_cx - cam_cx) * depth / fx
                y3d = (orig_cy - cam_cy) * depth / fy
                z3d = depth

                current_centroids.append((orig_cx, orig_cy, class_name, float(conf * class_conf), x3d, y3d, z3d))

        # Simple ID assignment (Tracking)
        assigned_tracks = {}
        current_time = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
        used_tids = set()

        for (cX, cY, class_name, conf, x3d, y3d, z3d) in current_centroids:
            matched_id = -1
            min_dist = float('inf')
            
            for tid, track_info in self.tracks.items():
                if tid in used_tids:
                    continue
                if track_info['class_name'] == class_name:
                    dist = math.hypot(cX - track_info['cX'], cY - track_info['cY'])
                    if dist < 50 and dist < min_dist:
                        min_dist = dist
                        matched_id = tid
            
            if matched_id != -1:
                used_tids.add(matched_id)
            
            is_dynamic = False
            if matched_id == -1:
                matched_id = self.next_track_id
                self.next_track_id += 1
            else:
                prev_x, prev_y, prev_z = self.tracks[matched_id]['pos_3d']
                prev_time = self.tracks[matched_id]['time']
                dt = current_time - prev_time
                if dt > 0.05:
                    dist_3d = math.sqrt((x3d - prev_x)**2 + (y3d - prev_y)**2 + (z3d - prev_z)**2)
                    velocity = dist_3d / dt
                    if velocity > 0.15:
                        is_dynamic = True
                    else:
                        is_dynamic = self.tracks[matched_id].get('is_dynamic', False)
                else:
                    is_dynamic = self.tracks[matched_id].get('is_dynamic', False)

            assigned_tracks[matched_id] = {
                'cX': cX, 'cY': cY,
                'class_name': class_name,
                'pos_3d': (x3d, y3d, z3d),
                'time': current_time,
                'is_dynamic': is_dynamic
            }
            
            det3d = Detection3D()
            det3d.header = rgb_msg.header
            
            hyp = ObjectHypothesisWithPose()
            state_str = "dynamic" if is_dynamic else "static"
            hyp.hypothesis.class_id = f"{class_name}_{state_str}"
            hyp.hypothesis.score = conf
            hyp.pose.pose.position.x = float(x3d)
            hyp.pose.pose.position.y = float(y3d)
            hyp.pose.pose.position.z = float(z3d)
            
            det3d.results.append(hyp)
            det3d.id = str(matched_id)
            detection_array.detections.append(det3d)

        self.tracks = assigned_tracks
        self.detection_pub.publish(detection_array)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
