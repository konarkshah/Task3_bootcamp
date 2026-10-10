#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
import sqlite3
import datetime
import os

class DatabaseNode(Node):
    def __init__(self):
        super().__init__('database_node')
        
        self.declare_parameter('global_frame', 'odom')
        self.global_frame = self.get_parameter('global_frame').value
        self.declare_parameter('db_path', 'spatial_memory.db')
        self.db_path = self.get_parameter('db_path').value

        # Use absolute path for DB
        self.db_path = os.path.abspath(self.db_path)

        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Database Setup
        self.init_db()

        # Subscriber
        self.subscription = self.create_subscription(
            Detection3DArray,
            'perception/detections_3d',
            self.detection_callback,
            10
        )
        self.get_logger().info(f"Memory Persistence Database Started!")
        self.get_logger().info(f"Storing spatial data to {self.db_path} in frame '{self.global_frame}'")

    def init_db(self):
        self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self.cursor = self.conn.cursor()
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS spatial_memory (
                track_id TEXT PRIMARY KEY,
                class_name TEXT,
                state TEXT,
                x REAL,
                y REAL,
                z REAL,
                last_seen TIMESTAMP
            )
        ''')
        self.conn.commit()

    def detection_callback(self, msg):
        if not msg.detections:
            return

        try:
            # Look up transform from camera frame to global frame (e.g. odom)
            # We use rclpy.time.Time() to get the latest available transform
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                msg.header.frame_id,
                rclpy.time.Time()
            )
        except Exception as e:
            # Usually happens on the first few frames before TF tree is fully built
            self.get_logger().warn(f"TF Error: Could not transform {msg.header.frame_id} to {self.global_frame}: {e}")
            return

        for det in msg.detections:
            track_id = det.id
            if not det.results:
                continue
                
            hyp = det.results[0]
            class_id_str = hyp.hypothesis.class_id # e.g. "person_dynamic"
            
            # Split the string to get class name and state
            if "_" in class_id_str:
                class_name, state = class_id_str.rsplit("_", 1)
            else:
                class_name = class_id_str
                state = "unknown"

            # Transform the pose
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = hyp.pose.pose
            
            global_pose_stamped = tf2_geometry_msgs.do_transform_pose(pose_stamped.pose, transform)
            
            gx = global_pose_stamped.position.x
            gy = global_pose_stamped.position.y
            gz = global_pose_stamped.position.z
            
            now_str = datetime.datetime.now().isoformat()

            # Insert or Replace into SQLite Database
            self.cursor.execute('''
                INSERT OR REPLACE INTO spatial_memory (track_id, class_name, state, x, y, z, last_seen)
                VALUES (?, ?, ?, ?, ?, ?, ?)
            ''', (track_id, class_name, state, gx, gy, gz, now_str))
            
        self.conn.commit()
        self.get_logger().info(f"Saved {len(msg.detections)} objects to Spatial Memory Database")

def main(args=None):
    rclpy.init(args=args)
    node = DatabaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

