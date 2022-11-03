from moveit_commander.planning_scene_interface import PlanningSceneInterface
from moveit_msgs.msg import CollisionObject

from geometry_msgs.msg import Pose

from scorpius_manipulation.msg import SceneObject, SceneObjectArray

class SceneUpdater:
    def __init__(self):
        self.scene = PlanningSceneInterface()

    def updateScene(self, objects:SceneObjectArray, frame):
        self.scene.remove_world_object()
        sceneObject:SceneObject
        for sceneObject in objects.objects:
            collisionObject = self._createCollisionObjectFromMesh(frame, sceneObject)
            self.scene.add_object(collisionObject)
    
    def _createCollisionObjectFromMesh(self, frame:str, object:SceneObject)->CollisionObject:
        collisionObject = CollisionObject()

        collisionObject.id = object.name
        collisionObject.meshes.append(object.mesh)
        pose = Pose()
        pose.orientation.w = 1.0
        collisionObject.mesh_poses.append(pose)
        collisionObject.header.frame_id = frame
        collisionObject.operation = collisionObject.ADD

        return collisionObject
        