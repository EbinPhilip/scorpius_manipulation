
import rospy

from dynamic_reconfigure.server import Server
from scorpius_manipulation.cfg import transformConfig
import tf

import tf2_ros
import geometry_msgs.msg

import math

broadcaster = None

def deg2rad(deg):
    return float((float(deg)/180.0) * math.pi)

def callback(config:transformConfig, level):
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = str(config.parent_frame)
    static_transformStamped.child_frame_id = "tf"

    static_transformStamped.transform.translation.x = config.x
    static_transformStamped.transform.translation.y = config.y
    static_transformStamped.transform.translation.z = config.z

    quat = tf.transformations.quaternion_from_euler(
                deg2rad(config.roll),deg2rad(config.pitch),deg2rad(config.yaw))
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    # pickup_pose = geometry_msgs.msg.TransformStamped()
    # pickup_pose.header.stamp = rospy.Time.now()
    # pickup_pose.header.frame_id = "disc_orientation"
    # pickup_pose.child_frame_id = "pickup_pose"
    # pickup_pose.transform.translation.x = float(config.x_offset)*0.001
    # pickup_pose.transform.rotation.w = 1

    # gripper_transform = geometry_msgs.msg.TransformStamped()
    # gripper_transform.header.stamp = rospy.Time.now()
    # gripper_transform.header.frame_id = "pickup_pose"
    # gripper_transform.child_frame_id = "gripper_pose"
    # gripper_transform.transform.translation.x = -.109
    # gripper_transform.transform.rotation.w = 1

    broadcaster.sendTransform(static_transformStamped)
    # broadcaster.sendTransform(pickup_pose)
    # broadcaster.sendTransform(gripper_transform)
    return config

if __name__ == "__main__":
    rospy.init_node("frame_dyn_config", anonymous = False)
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    srv = Server(transformConfig, callback)
    rospy.spin()