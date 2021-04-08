import airsimdroneracinglab as a
import os

c = a.MultirotorClient()
c.confirmConnection()

center = a.Vector3r(0, 0, 0)
output_path = os.path.join(os.getcwd(), "map.sdf")
# c.simCreateVoxelGrid(center, 100, 100, 100, 0.5, output_path)

c.simBuildSDF(center, 100, 100, 100, 0.5)
print(c.simCheckOccupancy(center))

c.simSaveSDF(output_path)

c.simLoadSDF(output_path)
obj_pos = c.simGetObjectPose("OrangeBall").position


print(f"Object is located at {obj_pos}")
print(f"Is point occupied? {c.simCheckOccupancy(obj_pos)}")
print(f"Signed distance at obj center is {c.simGetSignedDistance(obj_pos)}")
print(f"Gradient at obj center is {c.simGetSDFGradient(obj_pos)}")

free_pt = c.simProjectToFreeSpace(obj_pos, 2)

print(f"Signed distance at free pt is {c.simGetSignedDistance(free_pt)}")
print(f"Gradient at free pt is {c.simGetSDFGradient(free_pt)}")
