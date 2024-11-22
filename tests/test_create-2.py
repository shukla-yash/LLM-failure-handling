import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, 0)


def create_tool():
    basePosition = [0.3, 0, 0]
    baseOrientation = [0, 0, 0, 1]
    
    connector_position = [0.3, 0.02, 0.015]
    tip_position = [0.3, 0.34, 0]
    # create a tool that can bring the object closer to the robot
    cube_1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02])
    cube_2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02])
    cube_connection = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.01, 0.15, 0.01])

    # tool_base  = p.createMultiBody(0.1, cube_1, -1, basePosition, baseOrientation)
    # tool_connector = p.createMultiBody(0.1, cube_connection, -1, connector_position, baseOrientation)
    # tool_tip = p.createMultiBody(0.1, cube_2, -1, tip_position, baseOrientation)

    link_Masses = [0.1, 0.1, 0.1]
    linkCollisionShapeIndices = [cube_1, cube_connection, cube_2]
    linkVisualShapeIndices = [-1, -1, -1]
    linkPositions = [[0, 0, 0], [0, 0.15, 0.015], [0, 0.3, 0]]
    linkOrientations = [[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]]
    linkParentIndices = [0, 0, 0]
    linkJointTypes = [p.JOINT_FIXED, p.JOINT_FIXED, p.JOINT_FIXED]
    linkInertialFramePositions = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
    linkInertialFrameOrientations = [[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]]
    linkJointAxis = [[0, 0, 1], [0, 0, 1], [0, 0, 1]]

    tool = p.createMultiBody(0.1, cube_1, -1, basePosition, baseOrientation,
                             
                             linkMasses=link_Masses,
                             linkCollisionShapeIndices=linkCollisionShapeIndices,
                             linkVisualShapeIndices=linkVisualShapeIndices,
                             linkPositions=linkPositions,
                              linkOrientations=linkOrientations,
                             linkParentIndices=linkParentIndices,
                             linkJointTypes=linkJointTypes, 
                             linkInertialFramePositions=linkInertialFramePositions,
                             linkInertialFrameOrientations=linkInertialFrameOrientations, 
                             linkJointAxis=linkJointAxis)

    return tool

create_tool()
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)


while (1):
  keys = p.getKeyboardEvents()
#   print(keys)

  time.sleep(0.01)







