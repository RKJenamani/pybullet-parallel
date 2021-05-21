import pybullet as p
import pybullet_data as pd
import os

p.connect(p.DIRECT)
name_in = "/home/rajat/catkin_ws/src/TMOMO/models/024_bowl/textured_simple.obj"
name_out = "textured_simple_vhacd.obj"
name_log = "log.txt"
p.vhacd(name_in, name_out, name_log)
