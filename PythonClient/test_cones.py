# from PythonClient import airsimdroneracinglab
from time import time
import random
import time
import airsimdroneracinglab as adrl
from airsimdroneracinglab import Pose, Quaternionr, Vector3r


client = adrl.MultirotorClient()
client.confirmConnection()

client.simLoadLevel("Train2")
all_objects = client.simListSceneObjects()

input("ready camera")

# cones = [obj for obj in all_objects if "Template" in obj]
cones = [obj for obj in all_objects if "Cone" in obj]
cones = list(set((cones)))
print("len cones", len(cones))

# cones = []
cur_x = -10
for idx, cone in enumerate(cones):
    success = client.simSetObjectPose(
        cone, Pose(Vector3r(cur_x, 0, -20), Quaternionr())
    )
    print(success, cone, cur_x)
    cur_x += -2
    time.sleep(1.0)

    # client.simDestroyObject(cone)
    # print("destroy")
    # time.sleep(1.0)

    # client.simSpawnObject(
    #     cone,
    #     cone,
    #     pose=Pose(Vector3r(cur_x, 0, 0), Quaternionr()),
    #     scale=Vector3r(1, 1, 1),
    # )

all_objects = client.simListSceneObjects()
# cones = [obj for obj in all_objects if "Cone" in obj]
cones = [obj for obj in all_objects if "Template" in obj]
cones = list(set((cones)))
print("len cones 2", len(cones))
