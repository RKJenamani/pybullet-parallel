# import threading
import multiprocessing 
import time
import numpy as np
from types import SimpleNamespace
import pybullet
import pybullet_data
import pybullet_utils.bullet_client as bc

def setup_exp(p):
    mu = 0.65
    object_ids = []

    local_path = "./"

    state = ([-0.0404755 , -0.00112935 , 0.183326,0.000183599, 0.0274296, -3.12269])
    req = SimpleNamespace(o_x=state[0],o_y=state[1],o_z=state[2],o_r = state[3], o_p = state[4], o_yaw = state[5], type = 8)
    xyz = [req.o_x, req.o_y, req.o_z]
    rpy = [req.o_r, req.o_p, req.o_yaw]
    body_id = p.loadURDF(local_path + "models/024_bowl/model.urdf", xyz, p.getQuaternionFromEuler(rpy))        
    p.changeDynamics(body_id, -1, lateralFriction=mu)
    object_ids.append(body_id)

    state = ([-0.031447 , 0.00245332 , 0.231471,1.11077, 0.116883, 1.88827])
    req = SimpleNamespace(o_x=state[0],o_y=state[1],o_z=state[2],o_r = state[3], o_p = state[4], o_yaw = state[5], type = 8)
    xyz = [req.o_x, req.o_y, req.o_z]
    rpy = [req.o_r, req.o_p, req.o_yaw]
    body_id = p.loadURDF(local_path + "models/037_scissors/model.urdf", xyz, p.getQuaternionFromEuler(rpy))        
    p.changeDynamics(body_id, -1, lateralFriction=mu)
    object_ids.append(body_id)

    state = ([-0.0342039 , -0.0411717 , 0.221495,-1.21418, -0.411491, 1.3281])
    req = SimpleNamespace(o_x=state[0],o_y=state[1],o_z=state[2],o_r = state[3], o_p = state[4], o_yaw = state[5], type = 8)
    xyz = [req.o_x, req.o_y, req.o_z]
    rpy = [req.o_r, req.o_p, req.o_yaw]
    body_id = p.loadURDF(local_path + "models/011_banana/model.urdf", xyz, p.getQuaternionFromEuler(rpy))        
    p.changeDynamics(body_id, -1, lateralFriction=mu)
    object_ids.append(body_id)

    state = ([-0.00109234 , 0.00217348, 0.0940739,0.603044, -1.5319, 2.60045])
    req = SimpleNamespace(o_x=state[0],o_y=state[1],o_z=state[2],o_r = state[3], o_p = state[4], o_yaw = state[5], type = 8)
    xyz = [req.o_x, req.o_y, req.o_z]
    rpy = [req.o_r, req.o_p, req.o_yaw]
    body_id = p.loadURDF(local_path + "models/004_sugar_box/model.urdf", xyz, p.getQuaternionFromEuler(rpy))        
    p.changeDynamics(body_id, -1, lateralFriction=mu)
    object_ids.append(body_id)

    state = ([0.0431414 , -0.0814738, 0.100775,-1.56183, 0.0200383, -0.0332266])
    req = SimpleNamespace(o_x=state[0],o_y=state[1],o_z=state[2],o_r = state[3], o_p = state[4], o_yaw = state[5], type = 8)
    xyz = [req.o_x, req.o_y, req.o_z]
    rpy = [req.o_r, req.o_p, req.o_yaw]
    body_id = p.loadURDF(local_path + "models/010_potted_meat_can/model.urdf", xyz, p.getQuaternionFromEuler(rpy))        
    p.changeDynamics(body_id, -1, lateralFriction=mu)
    object_ids.append(body_id)

    state = ([-0.0996265 , -0.0109982, 0.229266,1.09516, -0.0324135, 1.93206])
    req = SimpleNamespace(o_x=state[0],o_y=state[1],o_z=state[2],o_r = state[3], o_p = state[4], o_yaw = state[5], type = 8)
    xyz = [req.o_x, req.o_y, req.o_z]
    rpy = [req.o_r, req.o_p, req.o_yaw]
    body_id = p.loadURDF(local_path + "models/037_scissors/model.urdf", xyz, p.getQuaternionFromEuler(rpy))        
    p.changeDynamics(body_id, -1, lateralFriction=mu)
    object_ids.append(body_id)

    state = ([0.0391189 , -0.0793964, 0.142291,-3.03823, -0.084093, 1.51302])
    req = SimpleNamespace(o_x=state[0],o_y=state[1],o_z=state[2],o_r = state[3], o_p = state[4], o_yaw = state[5], type = 8)
    xyz = [req.o_x, req.o_y, req.o_z]
    rpy = [req.o_r, req.o_p, req.o_yaw]
    body_id = p.loadURDF(local_path + "models/009_gelatin_box/model.urdf", xyz, p.getQuaternionFromEuler(rpy))        
    p.changeDynamics(body_id, -1, lateralFriction=mu)
    object_ids.append(body_id)

    state = ([-0.039549 , -0.000712143, 0.141748,-0.00616443, 0.0036889, -1.53266])
    req = SimpleNamespace(o_x=state[0],o_y=state[1],o_z=state[2],o_r = state[3], o_p = state[4], o_yaw = state[5], type = 8)
    xyz = [req.o_x, req.o_y, req.o_z]
    rpy = [req.o_r, req.o_p, req.o_yaw]
    body_id = p.loadURDF(local_path + "models/008_pudding_box/model.urdf", xyz, p.getQuaternionFromEuler(rpy))        
    p.changeDynamics(body_id, -1, lateralFriction=mu)
    object_ids.append(body_id)

    state = ([0.0518593 , -0.113949, 0.206423,-2.99895, -0.0929695, 1.27618])
    req = SimpleNamespace(o_x=state[0],o_y=state[1],o_z=state[2],o_r = state[3], o_p = state[4], o_yaw = state[5], type = 8)
    xyz = [req.o_x, req.o_y, req.o_z]
    rpy = [req.o_r, req.o_p, req.o_yaw]
    body_id = p.loadURDF(local_path + "models/007_tuna_fish_can/model.urdf", xyz, p.getQuaternionFromEuler(rpy))        
    p.changeDynamics(body_id, -1, lateralFriction=mu)
    object_ids.append(body_id)

    state = ([0.0575085 , -0.102112, 0.171442,-3.01417, -0.111824, 1.44483])
    req = SimpleNamespace(o_x=state[0],o_y=state[1],o_z=state[2],o_r = state[3], o_p = state[4], o_yaw = state[5], type = 8)
    xyz = [req.o_x, req.o_y, req.o_z]
    rpy = [req.o_r, req.o_p, req.o_yaw]
    body_id = p.loadURDF(local_path + "models/007_tuna_fish_can/model.urdf", xyz, p.getQuaternionFromEuler(rpy))        
    p.changeDynamics(body_id, -1, lateralFriction=mu)
    object_ids.append(body_id)

    state = ([0.000732725 , 0.000258393, 0.0369799,-2.1136, -1.55321, 0.539161])
    req = SimpleNamespace(o_x=state[0],o_y=state[1],o_z=state[2],o_r = state[3], o_p = state[4], o_yaw = state[5], type = 8)
    xyz = [req.o_x, req.o_y, req.o_z]
    rpy = [req.o_r, req.o_p, req.o_yaw]
    body_id = p.loadURDF(local_path + "models/003_cracker_box/model.urdf", xyz, p.getQuaternionFromEuler(rpy))        
    p.changeDynamics(body_id, -1, lateralFriction=mu)
    object_ids.append(body_id)

def run_sim(num_envs):

    all_p = []

    for i in range(num_envs):
        p = bc.BulletClient(connection_mode=pybullet.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-10)
        plane = p.loadURDF("plane.urdf")
        p.setRealTimeSimulation(0)

        all_p.append(p)

    for p in all_p:
        setup_exp(p)

    for _ in range(1000):
        for p in all_p:
            p.stepSimulation()

    return None

for iter in range(1,13):

    num_parallel = iter
    start_time = time.time()

    run_sim(num_parallel)

    end_time = time.time()

    print("Total time taken: {0:.2f} | Parallel Envs: {1} | Time taken per simulation: {2:.2f} ".format((end_time-start_time),num_parallel,(end_time-start_time)/num_parallel))