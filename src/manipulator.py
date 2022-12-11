import moveit_commander
from moveit_commander.move_group import MoveGroupCommander
import actionlib

from control_msgs.msg import GripperCommandAction as GCA
from control_msgs.msg import GripperCommandGoal as GCGoal
from control_msgs.msg import GripperCommandResult as GCResult 
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
import moveit_msgs

import rospy
import tf2_ros

prePickPose = TransformStamped()
prePickPose.child_frame_id = "pre_pick"
prePickPose.transform.translation.x = -0.21
prePickPose.transform.translation.y = -0.005
prePickPose.transform.translation.z = 0.03
prePickPose.transform.rotation.x = 0.0
prePickPose.transform.rotation.y = 0.0
prePickPose.transform.rotation.z = 0.0
prePickPose.transform.rotation.w = 1.0

pickPose = TransformStamped()
pickPose.child_frame_id = "pick"
pickPose.transform.translation.x = -0.15
pickPose.transform.translation.y = -0.005
pickPose.transform.translation.z = 0.03
pickPose.transform.rotation.x = 0.0
pickPose.transform.rotation.y = 0.0
pickPose.transform.rotation.z = 0.0
pickPose.transform.rotation.w = 1.0

poseGoal = Pose()
poseGoal.orientation.w = 1.0
poseGoal.position.x = 0.0
poseGoal.position.y = 0.0
poseGoal.position.z = 0.0

class Manipulator:
    def __init__(self, planningGroup: str, gripperActionName: str = 'sting_gripper_action'):
        moveit_commander.roscpp_initialize([])
        self.moveGroup: MoveGroupCommander
        self.moveGroup = moveit_commander.MoveGroupCommander(planningGroup)
        self.client = actionlib.SimpleActionClient(gripperActionName, GCA)
        self.client.wait_for_server()
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

    def _executeGripperAction(self, goal: GCGoal) -> GCResult :
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result: GCResult
        result = self.client.get_result()
        if result.reached_goal != True or result.stalled == True:
            raise RuntimeError("gripper action failed. goal: {}".format(goal.command.position))
        return result
    
    def openGripper(self, position = 0.01, effort = 0.270):
        goal = GCGoal()
        goal.command.position = position
        goal.command.max_effort = effort
        self._executeGripperAction(goal)
    

    def closeGripper(self, position = 0.09, effort = 0.270):
        goal = GCGoal()
        goal.command.position = position
        goal.command.max_effort = effort
        self._executeGripperAction(goal)
    
    def moveToPose(self, goalFrame: str):
        
        self.moveGroup.set_start_state_to_current_state()
        self.moveGroup.set_pose_reference_frame(goalFrame)
        self.moveGroup.set_pose_target(poseGoal)
        _, plan,_,_ = self.moveGroup.plan()

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = moveit_commander.RobotCommander() \
                .get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        result = self.moveGroup.execute(plan)
        self.moveGroup.clear_pose_targets()
        return result

    def pick(self, objectName: str):
        prePickPose.header.stamp = rospy.Time.now()
        prePickPose.header.frame_id = objectName
        self.broadcaster.sendTransform(prePickPose)

        pickPose.header.stamp = rospy.Time.now()
        pickPose.header.frame_id = objectName
        self.broadcaster.sendTransform(pickPose)
        
        self.moveToPose("pre_pick")
        self.moveToPose("pick")

        self.closeGripper()
        self.moveGroup.attach_object(objectName, "lower_arm_front", \
             ["left_finger", "right_finger"])

    def place(self, objectName:str, placeFrame = "place_frame"):
        self.moveToPose(placeFrame)
        self.openGripper()
        self.moveGroup.detach_object(objectName)
        

    def pickAndPlace(self, objectName: str, placePose: str):
        self.pick(objectName)
        self.place(objectName, placePose)