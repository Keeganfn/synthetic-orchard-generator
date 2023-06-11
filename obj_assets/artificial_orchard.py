import pybullet as p
import pybullet_data
import time
import random
import math
import numpy as np
from copy import deepcopy
from PIL import Image


def setup_sim(gui=True):
    if gui:
        physicsClient = p.connect(p.GUI)
    else:
        physicsClient = p.connect(p.DIRECT)

    # load in ground plane with dirt textures
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = p.loadURDF("plane.urdf")
    textureId = p.loadTexture("forest_ground_04_diff_2k.jpg")
    p.changeVisualShape(plane, -1, textureUniqueId=textureId)

    # Configure the debug camera visualizer (NO EFFECT ON ACTUAL CAMERA)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 1)
    p.resetDebugVisualizerCamera(cameraDistance=.5, cameraYaw=0, cameraPitch=-89.9999,
                                 cameraTargetPosition=[0, .5, 0.5])


def spawn_trees_preset_random(offset=1.5):
    tree_info = {"trunk_ids": [], "leaf_ids": [], "trunk_poses": [], "leaf_poses": []}
    x = random.uniform(0, offset)
    y = random.uniform(-offset, 0)
    tree1 = p.createVisualShape(p.GEOM_MESH, fileName="tree4_trunk.obj")
    tree1_id = p.createMultiBody(
        0, baseVisualShapeIndex=tree1, basePosition=[x, 0, 0],
        baseOrientation=p.getQuaternionFromEuler([1.57, 0, y]))
    leaves1 = p.createVisualShape(p.GEOM_MESH, fileName="tree4_leaves.obj")
    leaves1_id = p.createMultiBody(
        0, baseVisualShapeIndex=leaves1, basePosition=[x, 0, 0],
        baseOrientation=p.getQuaternionFromEuler([1.57, 0, y]))
    tree_info["trunk_ids"].append(tree1_id)
    tree_info["leaf_ids"].append(leaves1_id)
    tree_info["trunk_poses"].append(p.getBasePositionAndOrientation(tree1_id))
    tree_info["leaf_poses"].append(p.getBasePositionAndOrientation(leaves1_id))

    x = random.uniform(0, offset)
    y = random.uniform(-offset, 0)
    tree2 = p.createVisualShape(p.GEOM_MESH, fileName="tree4_trunk.obj")
    tree2_id = p.createMultiBody(
        0, baseVisualShapeIndex=tree2, basePosition=[y, 0, 0],
        baseOrientation=p.getQuaternionFromEuler([1.57, 0, y]))
    leaves2 = p.createVisualShape(p.GEOM_MESH, fileName="tree4_leaves.obj")
    leaves2_id = p.createMultiBody(
        0, baseVisualShapeIndex=leaves2, basePosition=[y, 0, 0],
        baseOrientation=p.getQuaternionFromEuler([1.57, 0, y]))
    tree_info["trunk_ids"].append(tree2_id)
    tree_info["leaf_ids"].append(leaves2_id)
    tree_info["trunk_poses"].append(p.getBasePositionAndOrientation(tree2_id))
    tree_info["leaf_poses"].append(p.getBasePositionAndOrientation(leaves2_id))

    x = random.uniform(0, offset)
    y = random.uniform(-offset, 0)
    tree3 = p.createVisualShape(p.GEOM_MESH, fileName="tree2_trunk.obj")
    tree3_id = p.createMultiBody(
        0, baseVisualShapeIndex=tree3, basePosition=[0, 0, 0],
        baseOrientation=p.getQuaternionFromEuler([1.57, 0, y]))
    leaves3 = p.createVisualShape(p.GEOM_MESH, fileName="tree2_leaves.obj")
    leaves3_id = p.createMultiBody(
        0, baseVisualShapeIndex=leaves3, basePosition=[0, 0, 0],
        baseOrientation=p.getQuaternionFromEuler([1.57, 0, y]))
    tree_info["trunk_ids"].append(tree3_id)
    tree_info["leaf_ids"].append(leaves3_id)
    tree_info["trunk_poses"].append(p.getBasePositionAndOrientation(tree3_id))
    tree_info["leaf_poses"].append(p.getBasePositionAndOrientation(leaves3_id))

    x = random.uniform(0, offset)
    y = random.uniform(-offset, 0)
    tree4 = p.createVisualShape(p.GEOM_MESH, fileName="tree2_trunk.obj")
    tree4_id = p.createMultiBody(
        0, baseVisualShapeIndex=tree4, basePosition=[0, y, 0],
        baseOrientation=p.getQuaternionFromEuler([1.57, 0, y]))
    leaves4 = p.createVisualShape(p.GEOM_MESH, fileName="tree2_leaves.obj")
    leaves4_id = p.createMultiBody(
        0, baseVisualShapeIndex=leaves4, basePosition=[0, y, 0],
        baseOrientation=p.getQuaternionFromEuler([1.57, 0, y]))
    tree_info["trunk_ids"].append(tree4_id)
    tree_info["leaf_ids"].append(leaves4_id)
    tree_info["trunk_poses"].append(p.getBasePositionAndOrientation(tree4_id))
    tree_info["leaf_poses"].append(p.getBasePositionAndOrientation(leaves4_id))

    return tree_info


def spawn_camera(width=1024, height=1024, fov=60, aspect=1, near=.001, far=5, debug=False, target_z=None, camera_z=None):
    if target_z == None or camera_z == None:
        target_z = random.uniform(.2, 2)
        camera_z = random.uniform(.7, 2)
    camera_location = [0, 1.25, camera_z]
    target_location = [0, 0, target_z]
    up_vector = [0, 0, 1]

    view_matrix = p.computeViewMatrix(camera_location, target_location, up_vector)
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

    if debug:
        p.addUserDebugPoints([camera_location], [[0, 255, 0]], pointSize=20)
        p.addUserDebugPoints([target_location], [[0, 0, 255]], pointSize=20)
        p.addUserDebugLine(camera_location, target_location, lineColorRGB=[[255, 0, 0]], lineWidth=10)
        images = p.getCameraImage(width, height, view_matrix, projection_matrix,
                                  renderer=p.ER_BULLET_HARDWARE_OPENGL)

    camera_info = {
        "width": width, "height": height, "view_matrix": view_matrix, "projection_matrix": projection_matrix,
        "camera_location": camera_location, "target_location": target_location, "fov": fov, "aspect": aspect,
        "up_vector": up_vector, "near": near, "far": far}

    return camera_info


def spawn_apples(camera_location, target_location, min_apples=4, max_apples=11, translation_noise=.3,
                 rotational_noise=.7, apple_prob=.15):

    apple_info = {"apple_num": 0, "apple_ids": [], "apple_locations": [], "apple_type": []}
    num_apples = math.ceil(random.uniform(min_apples, max_apples))
    green_prob = random.uniform(0, 1)

    for i in range(num_apples):
        x = random.uniform(0, 0)
        y = random.uniform(min(camera_location[1], target_location[1]), max(target_location[1], camera_location[1]))
        z = random.uniform(min(camera_location[2], target_location[2]), max(target_location[2], camera_location[2]))

        x_noise = random.uniform(-translation_noise, translation_noise)
        y_noise = random.uniform(-translation_noise, translation_noise)
        z_noise = random.uniform(-translation_noise, translation_noise)

        y_rot_noise = random.uniform(-rotational_noise, rotational_noise)
        z_rot_noise = random.uniform(-rotational_noise, rotational_noise)

        if green_prob > apple_prob:
            apple = p.createVisualShape(p.GEOM_MESH, fileName="apple2_red.obj")
            type = "red"
        else:
            apple = p.createVisualShape(p.GEOM_MESH, fileName="apple1_green.obj")
            type = "green"
        apple_id = p.createMultiBody(
            0, baseVisualShapeIndex=apple, basePosition=[x+x_noise, y+y_noise, z+z_noise],
            baseOrientation=p.getQuaternionFromEuler([1.57, y_rot_noise, z_rot_noise]))
        apple_info["apple_ids"].append(apple_id)
        apple_info["apple_locations"].append(p.getBasePositionAndOrientation(apple_id))
        apple_info["apple_type"].append(type)
        apple_info["apple_num"] = num_apples

    return apple_info


def get_image_apple_ids(ids, img):
    visible_apples = []
    for i in ids:
        if i in img:
            visible_apples.append(i)
    return visible_apples


def load_single_apple(position, orientation, type):
    if type == "red":
        obj_file = "apple2_red.obj"
    else:
        obj_file = "apple1_green.obj"
    apple = p.createVisualShape(p.GEOM_MESH, fileName=obj_file)
    apple_id = p.createMultiBody(
        0, baseVisualShapeIndex=apple, basePosition=position,
        baseOrientation=orientation)
    return apple_id


def get_modal_masks(ids, img):
    pass


def get_amodal_masks(ids, apple_info, camera_info):
    amodal_masks = {}

    p.resetSimulation()
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 1)
    spawn_camera(camera_z=camera_info["camera_location"][2], target_z=camera_info["target_location"][2])
    for i in ids:
        apple_ind = apple_info["apple_ids"].index(i)
        new_id = load_single_apple(
            apple_info["apple_locations"][apple_ind][0],
            apple_info["apple_locations"][apple_ind][1],
            apple_info["apple_type"][apple_ind])
        amodal_mask = p.getCameraImage(
            camera_info["width"],
            camera_info["height"],
            camera_info["view_matrix"],
            camera_info["projection_matrix"],
            renderer=p.ER_BULLET_HARDWARE_OPENGL)[4]
        p.removeBody(new_id)
        amodal_masks[i] = deepcopy(amodal_mask)
        p.stepSimulation()
    return amodal_masks


def stall_sim():
    for i in range(10000):
        p.stepSimulation()
        time.sleep(1./240.)


if __name__ == "__main__":
    setup_sim()
    tree_info = spawn_trees_preset_random()
    camera_info = spawn_camera()
    apple_info = spawn_apples(
        camera_location=camera_info["camera_location"],
        target_location=camera_info["target_location"])
    image_info_modal = p.getCameraImage(
        camera_info["width"],
        camera_info["height"],
        camera_info["view_matrix"],
        camera_info["projection_matrix"],
        renderer=p.ER_BULLET_HARDWARE_OPENGL)
    print(image_info_modal[4])
    apples_ids_in_image = get_image_apple_ids(apple_info["apple_ids"], image_info_modal[4])
    amodal_masks = get_amodal_masks(apples_ids_in_image, apple_info, camera_info)

    im = Image.fromarray(image_info_modal[2].astype(np.uint8))
    im.show()
    j = image_info_modal[4]
    for i in apples_ids_in_image:
        j[j == i] = 255
    j[j != 255] = 0
    print(j)
    im = Image.fromarray(j.astype(np.uint8))
    im.show()
    im.save("test.png")
