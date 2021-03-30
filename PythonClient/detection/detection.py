import setup_path 
import airsim
import cv2
import numpy as np 
import pprint

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()

# set detection radius in [cm]
client.simSetDetectionFilterRadius(200 * 100) 
# add desired object name to detect in wild card/regex format
client.simAddDetectionFilterMeshName("Cylinder*") 

# set camera name and image type to request images and detections
camera_name = "0"
image_type = airsim.ImageType.Scene

while True:
    rawImage = client.simGetImage(camera_name, image_type)
    if not rawImage:
        continue
    png = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)
    cylinders = client.simGetDetections(camera_name, image_type)
    if cylinders:
        for cylinder in cylinders:
            s = pprint.pformat(cylinder)
            print("Cylinder: %s" % s)

            cv2.rectangle(png,(cylinder.topLeft_x,cylinder.topLeft_y),(cylinder.bottomRight_x,cylinder.bottomRight_y),(255,0,0),2)
            cv2.putText(png, cylinder.name, (cylinder.topLeft_x, cylinder.topLeft_y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36,255,12))

    
    cv2.imshow("AirSim", png)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    elif cv2.waitKey(1) & 0xFF == ord('c'):
        client.simClearDetectionMeshNames()
    elif cv2.waitKey(1) & 0xFF == ord('a'):
        client.simAddDetectionFilterMeshName("Cylinder*")
cv2.destroyAllWindows() 