from pcl_object_extraction.msg import ObjectBoundingBox
import tf2_ros
import tf2_msgs.msg
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

import rospy

class GraspCalculator:
    def getGrasp(self, box:ObjectBoundingBox,
            parentFrame:str, childFrameName:str)->TransformStamped:
        pose = TransformStamped()
        pose.transform.translation.x = (box.xmax+box.xmin)/2.0
        pose.transform.translation.y = (box.ymax+box.ymin)/2.0
        pose.transform.translation.z = (box.zmax+box.zmin)/2.0

        if (((box.xmax-box.xmin)/(box.ymax-box.ymin)) >= 0.7):
            q = quaternion_from_euler(0,1.5707, 0)
        else:
            q = quaternion_from_euler(1.5707,1.5707, 0)
        
        pose.transform.rotation.x = q[0]
        pose.transform.rotation.y = q[1]
        pose.transform.rotation.z = q[2]
        pose.transform.rotation.w = q[3]

        pose.header.frame_id = parentFrame
        pose.header.stamp = rospy.Time.now()
        pose.child_frame_id = childFrameName

        return pose

