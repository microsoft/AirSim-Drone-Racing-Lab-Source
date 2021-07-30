import airsimdroneracinglab

c = airsimdroneracinglab.MultirotorClient()

c.confirmConnection()

p = c.simGetVehiclePose().position
print(p)
#print(c.simGetObjectPose("SM_MERGED_OfficeSpaceMesh2_2"))

#while True:
#    print(c.simCheckInVolume(p, "SM_MERGED_OfficeSpaceMesh2_2"))

