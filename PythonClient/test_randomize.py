import airsimdroneracinglab
import time
c = airsimdroneracinglab.MultirotorClient()

c.confirmConnection()

c.simLoadLevel('Soccer_Field_Easy')
time.sleep(1)
# print(c.simListSceneObjects())

c.simSetMeshMaterial('Gate01', '/DroneRacing/DroneGates/Materials/M_DroneGateBase.M_DroneGateBase')
c.simSetMeshMaterialFromTexture('Gate00', 'C://Users//ratneshmadaan//Pictures//1.png')