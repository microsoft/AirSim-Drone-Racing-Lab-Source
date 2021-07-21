import airsimdroneracinglab
import time
c = airsimdroneracinglab.MultirotorClient()

c.confirmConnection()

c.simLoadLevel('Soccer_Field_Easy')
time.sleep(5)
print(c.simListSceneObjects())

c.simSetMeshMaterial('Gate01', '/DroneRacing/DroneGates/Materials/M_DroneGateBase.M_DroneGateBase')
c.simSetMeshMaterialFromTexture('Gate00', 'test.png')