import pybullet as p
import pybullet_data
import time
import random
import math

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)
p.resetDebugVisualizerCamera(cameraDistance=.5, cameraYaw=0, cameraPitch=-89.9999,
                             cameraTargetPosition=[0, .5, 0.5])
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 1)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 1)
p.setGravity(0, 0, -10)
plane = p.loadURDF("plane.urdf")
textureId = p.loadTexture("forest_ground_04_diff_2k.jpg")
p.changeVisualShape(plane, -1, textureUniqueId=textureId)

offset = 1.5
x = random.uniform(0, offset)
y = random.uniform(-offset, 0)
tree = p.createVisualShape(p.GEOM_MESH, fileName="tree4_trunk.obj")
meshId = p.createMultiBody(
    0, baseVisualShapeIndex=tree, basePosition=[x, 0, 0],
    baseOrientation=p.getQuaternionFromEuler([1.57, 0, y]))
leaves = p.createVisualShape(p.GEOM_MESH, fileName="tree4_leaves.obj")
meshId2 = p.createMultiBody(
    0, baseVisualShapeIndex=leaves, basePosition=[x, 0, 0],
    baseOrientation=p.getQuaternionFromEuler([1.57, 0, y]))

x = random.uniform(0, offset)
y = random.uniform(-offset, 0)
tree = p.createVisualShape(p.GEOM_MESH, fileName="tree4_trunk.obj")
meshId = p.createMultiBody(
    0, baseVisualShapeIndex=tree, basePosition=[y, 0, 0],
    baseOrientation=p.getQuaternionFromEuler([1.57, 0, y]))
leaves = p.createVisualShape(p.GEOM_MESH, fileName="tree4_leaves.obj")
meshId2 = p.createMultiBody(
    0, baseVisualShapeIndex=leaves, basePosition=[y, 0, 0],
    baseOrientation=p.getQuaternionFromEuler([1.57, 0, y]))

x = random.uniform(0, offset)
y = random.uniform(-offset, 0)
tree = p.createVisualShape(p.GEOM_MESH, fileName="tree2_trunk.obj")
meshId = p.createMultiBody(
    0, baseVisualShapeIndex=tree, basePosition=[0, 0, 0],
    baseOrientation=p.getQuaternionFromEuler([1.57, 0, y]))
leaves = p.createVisualShape(p.GEOM_MESH, fileName="tree2_leaves.obj")
meshId2 = p.createMultiBody(
    0, baseVisualShapeIndex=leaves, basePosition=[0, 0, 0],
    baseOrientation=p.getQuaternionFromEuler([1.57, 0, y]))

x = random.uniform(0, offset)
y = random.uniform(-offset, 0)
tree = p.createVisualShape(p.GEOM_MESH, fileName="tree2_trunk.obj")
meshId = p.createMultiBody(
    0, baseVisualShapeIndex=tree, basePosition=[0, y, 0],
    baseOrientation=p.getQuaternionFromEuler([1.57, 0, y]))
leaves = p.createVisualShape(p.GEOM_MESH, fileName="tree2_leaves.obj")
meshId2 = p.createMultiBody(
    0, baseVisualShapeIndex=leaves, basePosition=[0, y, 0],
    baseOrientation=p.getQuaternionFromEuler([1.57, 0, y]))


# offset = 2
# for i in range(5):
#     x = random.uniform(-offset, offset)
#     y = random.uniform(-offset, offset)
#     tree = p.createVisualShape(p.GEOM_MESH, fileName="tree4_trunk.obj")
#     meshId = p.createMultiBody(
#         0, baseVisualShapeIndex=tree, basePosition=[i + .5, 0, 0],
#         baseOrientation=p.getQuaternionFromEuler([1.57, 0, y]))
#     leaves = p.createVisualShape(p.GEOM_MESH, fileName="tree4_leaves.obj")
#     meshId2 = p.createMultiBody(
#         0, baseVisualShapeIndex=leaves, basePosition=[i + .5, 0, 0],
#         baseOrientation=p.getQuaternionFromEuler([1.57, 0, y]))
# print("ERROR CODE", tree)
# print("VERTICES", p.getAABB(leaves))

# offset = 2
# for i in range(5):
#     x = random.uniform(-offset, offset)
#     y = random.uniform(-offset, offset)
#     tree = p.createVisualShape(p.GEOM_MESH, fileName="tree2_trunk.obj")
#     meshId = p.createMultiBody(
#         0, baseVisualShapeIndex=tree, basePosition=[i+1+.5, 1, 0],
#         baseOrientation=p.getQuaternionFromEuler([1.57, 0.01*x, y]))
#     leaves = p.createVisualShape(p.GEOM_MESH, fileName="tree2_leaves.obj")
#     meshId2 = p.createMultiBody(
#         0, baseVisualShapeIndex=leaves, basePosition=[i + 1 + .5, 1, 0],
#         baseOrientation=p.getQuaternionFromEuler([1.57, 0.01 * x, y]))
#     print("UNIQUE ID", meshId2)
# print("VERTICES", p.getAABB(leaves))


width = 1024
height = 1024

fov = 80
aspect = width / height
near = 0.001
far = 5

view_matrix = p.computeViewMatrix([0, 1.25, 1.5], [0, 0, .2], [0, 0, 1])
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
p.addUserDebugPoints([[1.25, 0, 1]], [[0, 255, 0]], pointSize=20)
p.addUserDebugPoints([[0, 0, .5]], [[0, 0, 255]], pointSize=20)
p.addUserDebugLine([1.25, 0, 1], [0, 0, .5], lineColorRGB=[[255, 0, 0]], lineWidth=10)

distance = math.sqrt((1.25)**2 + .5**2)
offset = distance * math.tan(.174)
# p.addUserDebugPoints([[0, 0, .5], [0, 0, .7]], [[0, 255, 0], [0, 255, 0]], pointSize=10)
p.addUserDebugPoints([[0, offset, 1], [0, -offset, .5]], [[0, 255, 0], [0, 255, 0]], pointSize=10)
p.addUserDebugPoints([[offset, 0, .5], [-offset, 0, .5]], [[0, 255, 0], [0, 255, 0]], pointSize=10)
p.addUserDebugPoints([[0, 0, .5 + offset], [0, 0, .5 - offset]], [[0, 255, 0], [0, 255, 0]], pointSize=10)
# Get depth values using the OpenGL renderer


offset = .3
for i in range(10):
    # x = random.uniform(-offset, offset)
    # y = random.uniform(-offset, offset)
    # z = random.uniform(.5 - offset, .5 + offset)

    x = random.uniform(0, 0)
    y = random.uniform(.75, 1.25)
    z = random.uniform(1, 1.5)

    x2 = random.uniform(-offset, offset)
    y2 = random.uniform(-offset, offset)
    z2 = random.uniform(-offset, offset)

    l = p.createVisualShape(p.GEOM_MESH, fileName="apple2_red.obj")
    meshId2 = p.createMultiBody(
        0, baseVisualShapeIndex=l, basePosition=[x+x2, y+y2, z+z2],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))
    print("VERTICES", p.getAABB(l))


images = p.getCameraImage(width, height, view_matrix, projection_matrix,
                          renderer=p.ER_BULLET_HARDWARE_OPENGL, lightDirection=[-100, -100, 2], lightDistance=5)
print(images)


while True:
    # images = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    p.stepSimulation()
    time.sleep(1/240.)
p.disconnect()
