from __future__ import print_function

from .utils import *
from .types import *
from .baseline import *

import msgpackrpc  # install as admin: pip install msgpack-rpc-python
import numpy as np  # pip install numpy
import msgpack
import time
import math
import logging
from copy import deepcopy


class VehicleClient:
    def __init__(self, ip="", port=41451, timeout_value=3600):
        if ip == "":
            ip = "127.0.0.1"
        self.client = msgpackrpc.Client(
            msgpackrpc.Address(ip, port),
            timeout=timeout_value,
            pack_encoding="utf-8",
            unpack_encoding="utf-8",
        )

    # -----------------------------------  Common vehicle APIs ---------------------------------------------
    def reset(self):
        """
        Reset the vehicle to its original starting state

        Note that you must call `enableApiControl` and `armDisarm` again after the call to reset
        """
        self.client.call("reset")

    def ping(self):
        """
        If connection is established then this call will return true otherwise it will be blocked until timeout

        Returns:
            bool:
        """
        return self.client.call("ping")

    def getClientVersion(self):
        return 1  # sync with C++ client

    def getServerVersion(self):
        return self.client.call("getServerVersion")

    def getMinRequiredServerVersion(self):
        return 1  # sync with C++ client

    def getMinRequiredClientVersion(self):
        return self.client.call("getMinRequiredClientVersion")

    # basic flight control

    def enableApiControl(self, vehicle_name=""):
        """Enables API control for vehicle with name vehicle_name
        
        Args:
            vehicle_name (str, optional): Name of vehicle to enable API control for
        
        Returns:
            bool: Success
        """
        return self.client.call("enableApiControl", True, vehicle_name)

    def disableApiControl(self, vehicle_name=""):
        """Disables API control for vehicle with name vehicle_name
        
        Args:
            vehicle_name (str, optional): Name of vehicle to disable API control for
        
        Returns:
            bool: Success
        """
        return self.client.call("enableApiControl", False, vehicle_name)

    def isApiControlEnabled(self, vehicle_name=""):
        """
        Returns true if API control is established.

        If false (which is default) then API calls would be ignored. After a successful call to `enableApiControl`, `isApiControlEnabled` should return true.

        Args:
            vehicle_name (str, optional): Name of the vehicle

        Returns:
            bool: If API control is enabled
        """
        return self.client.call("isApiControlEnabled", vehicle_name)

    def arm(self, vehicle_name=""):
        """Arms vehicle corresponding to vehicle_name
        
        Args:
            vehicle_name (str): Name of vehicle
        
        Returns:
            bool: Success
        
        """
        return self.client.call("armDisarm", True, vehicle_name)

    def disarm(self, vehicle_name=""):
        """Disarms vehicle corresponding to vehicle_name.
        
        Args:
            vehicle_name (str): Name of vehicle
        
        Returns:
            bool: Success
        
        """
        return self.client.call("armDisarm", False, vehicle_name)

    def simPause(self, is_paused):
        """
        Pauses simulation

        Args:
            is_paused (bool): True to pause the simulation, False to release
        """
        self.client.call("simPause", is_paused)

    def simIsPause(self):
        """
        Returns true if the simulation is paused

        Returns:
            bool: If the simulation is paused
        """
        return self.client.call("simIsPaused")

    def simContinueForTime(self, seconds):
        """
        Continue the simulation for the specified number of seconds

        Args:
            seconds (float): Time to run the simulation for
        """
        self.client.call("simContinueForTime", seconds)

    def getHomeGeoPoint(self, vehicle_name=""):
        """
        Get the Home location of the vehicle

        Args:
            vehicle_name (str, optional): Name of vehicle to get home location of

        Returns:
            GeoPoint: Home location of the vehicle
        """
        return GeoPoint.from_msgpack(self.client.call("getHomeGeoPoint", vehicle_name))

    def confirmConnection(self):
        """
        Checks state of connection every 1 sec and reports it in Console so user can see the progress for connection.
        """
        if self.ping():
            print("Connected!")
        else:
            print("Ping returned false!")
        server_ver = self.getServerVersion()
        client_ver = self.getClientVersion()
        server_min_ver = self.getMinRequiredServerVersion()
        client_min_ver = self.getMinRequiredClientVersion()

        ver_info = (
            "Client Ver:"
            + str(client_ver)
            + " (Min Req: "
            + str(client_min_ver)
            + "), Server Ver:"
            + str(server_ver)
            + " (Min Req: "
            + str(server_min_ver)
            + ")"
        )

        if server_ver < server_min_ver:
            print(ver_info, file=sys.stderr)
            print(
                "AirSim server is of older version and not supported by this client. Please upgrade!"
            )
        elif client_ver < client_min_ver:
            print(ver_info, file=sys.stderr)
            print(
                "AirSim client is of older version and not supported by this server. Please upgrade!"
            )
        else:
            print(ver_info)
        print("")

    def simSwapTextures(self, tags, tex_id=0, component_id=0, material_id=0):
        """
        Runtime Swap Texture API

        See https://microsoft.github.io/AirSim/retexturing/ for details

        Args:
            tags (str): string of "," or ", " delimited tags to identify on which actors to perform the swap
            tex_id (int, optional): indexes the array of textures assigned to each actor undergoing a swap

                                    If out-of-bounds for some object's texture set, it will be taken modulo the number of textures that were available
            component_id (int, optional):
            material_id (int, optional):

        Returns:
            list[str]: List of objects which matched the provided tags and had the texture swap perfomed
        """
        return self.client.call(
            "simSwapTextures", tags, tex_id, component_id, material_id
        )

    def simSetMeshMaterial(self, object_name, material_name):
        """
        Args:
            object_name (str): Name of object (mesh) whose material is to be changed
            material_name (str): Full path to the desired material asset

        Returns:
            success[bool]: True if change succeeded
        """
        return self.client.call(
            "simSetMeshMaterial", object_name, material_name
        )

    def simSetMeshMaterialFromTexture(self, object_name, texture_path):
        """
        Args:
            object_name (str): Name of object (mesh) whose material is to be changed
            material_name (str): Full path to the texture (PNG file) from which material is created

        Returns:
            success[bool]: True if change succeeded
        """
        return self.client.call(
            "simSetMeshMaterialFromTexture", object_name, texture_path
        )


    # time-of-day control
    def simSetTimeOfDay(
        self,
        is_enabled,
        start_datetime="",
        is_start_datetime_dst=False,
        celestial_clock_speed=1,
        update_interval_secs=60,
        move_sun=True,
    ):
        """
        Control the position of Sun in the environment

        Sun's position is computed using the coordinates specified in `OriginGeopoint` in settings for the date-time specified in the argument,
        else if the string is empty, current date & time is used

        Args:
            is_enabled (bool): True to enable time-of-day effect, False to reset the position to original
            start_datetime (str, optional): Date & Time in %Y-%m-%d %H:%M:%S format, e.g. `2018-02-12 15:20:00`
            is_start_datetime_dst (bool, optional): True to adjust for Daylight Savings Time
            celestial_clock_speed (float, optional): Run celestial clock faster or slower than simulation clock
                                                     E.g. Value 100 means for every 1 second of simulation clock, Sun's position is advanced by 100 seconds
                                                     so Sun will move in sky much faster
            update_interval_secs (float, optional): Interval to update the Sun's position
            move_sun (bool, optional): Whether or not to move the Sun
        """
        self.client.call(
            "simSetTimeOfDay",
            is_enabled,
            start_datetime,
            is_start_datetime_dst,
            celestial_clock_speed,
            update_interval_secs,
            move_sun,
        )

    # weather
    def simEnableWeather(self, enable):
        """
        Enable Weather effects. Needs to be called before using `simSetWeatherParameter` API

        Args:
            enable (bool): True to enable, False to disable
        """
        self.client.call("simEnableWeather", enable)

    def simSetWeatherParameter(self, param, val):
        """
        Enable various weather effects

        Args:
            param (WeatherParameter): Weather effect to be enabled
            val (float): Intensity of the effect, Range 0-1
        """
        self.client.call("simSetWeatherParameter", param, val)

    def simSetTextureFromUrl(self, object_name, url):
        """
        Set texture of an object from a URL resource. 

        Args:
            object_name (string): Name of object for which the texture is to be set
            url (string): URL of texture file
        """
        self.client.call("simSetTextureFromUrl", object_name, url)

    # camera control
    # simGetImage returns compressed png in array of bytes
    # image_type uses one of the ImageType members
    def simGetImage(self, camera_name, image_type, vehicle_name=""):
        """
        Get a single image

        Returns bytes of png format image which can be dumped into abinary file to create .png image
        `string_to_uint8_array()` can be used to convert into Numpy unit8 array
        See https://microsoft.github.io/AirSim/image_apis/ for details

        Args:
            camera_name (str): Name of the camera, for backwards compatibility, ID numbers such as 0,1,etc. can also be used
            image_type (ImageType): Type of image required
            vehicle_name (str, optional): Name of the vehicle with the camera

        Returns:
            Binary string literal of compressed png image
        """
        # todo: in future remove below, it's only for compatibility to pre v1.2
        camera_name = str(camera_name)

        # because this method returns std::vector<uint8>, msgpack decides to encode it as a string unfortunately.
        result = self.client.call("simGetImage", camera_name, image_type, vehicle_name)
        if result == "" or result == "\0":
            return None
        return result

    # camera control
    # simGetImage returns compressed png in array of bytes
    # image_type uses one of the ImageType members
    def simGetImages(self, requests, vehicle_name=""):
        """
        Get multiple images

        See https://microsoft.github.io/AirSim/image_apis/ for details and examples

        Args:
            requests (list[ImageRequest]): Images required
            vehicle_name (str, optional): Name of vehicle associated with the camera

        Returns:
            list[ImageResponse]:
        """
        responses_raw = self.client.call("simGetImages", requests, vehicle_name)
        return [
            ImageResponse.from_msgpack(response_raw) for response_raw in responses_raw
        ]

    # gets the static meshes in the unreal scene
    def simGetMeshPositionVertexBuffers(self):
        """
        Returns the static meshes that make up the scene

        See https://microsoft.github.io/AirSim/meshes/ for details and how to use this

        Returns:
            list[MeshPositionVertexBuffersResponse]:
        """
        responses_raw = self.client.call("simGetMeshPositionVertexBuffers")
        return [
            MeshPositionVertexBuffersResponse.from_msgpack(response_raw)
            for response_raw in responses_raw
        ]

    def simGetCollisionInfo(self, vehicle_name=""):
        """
        Args:
            vehicle_name (str, optional): Name of the Vehicle to get the info of

        Returns:
            CollisionInfo:
        """
        return CollisionInfo.from_msgpack(
            self.client.call("simGetCollisionInfo", vehicle_name)
        )

    def simSetVehiclePose(self, pose, ignore_collison, vehicle_name=""):
        """
        Set the pose of the vehicle

        If you don't want to change position (or orientation) then just set components of position (or orientation) to floating point nan values

        Args:
            pose (Pose): Desired Pose pf the vehicle
            ignore_collision (bool): Whether to ignore any collision or not
            vehicle_name (str, optional): Name of the vehicle to move
        """
        self.client.call("simSetVehiclePose", pose, ignore_collison, vehicle_name)

    def simGetVehiclePose(self, vehicle_name=""):
        """
        Args:
            vehicle_name (str, optional): Name of the vehicle to get the Pose of

        Returns:
            Pose:
        """
        pose = self.client.call("simGetVehiclePose", vehicle_name)
        return Pose.from_msgpack(pose)

    def simSetTraceLine(self, color_rgba, thickness=1.0, vehicle_name=""):
        """
        Modify the color and thickness of the line when Tracing is enabled

        Tracing can be enabled by pressing T in the Editor or setting `EnableTrace` to `True` in the Vehicle Settings

        Args:
            color_rgba (list): desired RGBA values from 0.0 to 1.0
            thickness (float, optional): Thickness of the line
            vehicle_name (string, optional): Name of the vehicle to set Trace line values for
        """
        self.client.call("simSetTraceLine", color_rgba, thickness, vehicle_name)

    def simGetObjectPose(self, object_name):
        """
        Returns true pose if simstartrace is not called 
        Returns true pose if simstartrace was called with tier=1
        Returns noisy pose if simstartrace was called with tier=2 or tier=3

        Args:
            object_name (str): Name of the object to get pose for
        
        Returns:
            Pose: pose of desired object, NanPose if object not found
        """
        # if simstartrace was never called, return true pose
        if self.race_tier is None:
            pose = self.client.call("simGetObjectPose", object_name, False)
        # if simstartrace was called with tier 1, return true pose
        elif self.race_tier == 1:
            pose = self.client.call("simGetObjectPose", object_name, False)
        # if simstartrace was called with tier 2 or 3, return noisy pose
        else:
            pose = self.client.call("simGetObjectPose", object_name, True)
        return Pose.from_msgpack(pose)

    def simSetObjectPose(self, object_name, pose, teleport=True):
        """
        Set the pose of the object(actor) in the environment

        The specified actor must have Mobility set to movable, otherwise there will be undefined behaviour.
        See https://www.unrealengine.com/en-US/blog/moving-physical-objects for details on how to set Mobility and the effect of Teleport parameter

        Args:
            object_name (str): Name of the object(actor) to move
            pose (Pose): Desired Pose of the object
            teleport (bool, optional): Whether to move the object immediately without affecting their velocity

        Returns:
            bool: If the move was successful
        """
        return self.client.call("simSetObjectPose", object_name, pose, teleport)

    def simGetNominalGateInnerDimensions(self):
        """
        - Return the dimensions of the drone racing gate cavity, with scale (width=1.0, thickness=1.0, height=1.0)
        - Use this API in conjunction with simGetObjectScale(), simSetObjectScale(), simSetObjectPose() to generate arbitrary sized checkered gates with arbitrary poses.  

        Returns:
            Vector3r: width, thickness, height of the gate inner cavity in meters. 
            See https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/issues/28 for gate coordinate system
        """
        return Vector3r(1.6, 0.2, 1.6)

    def simGetNominalGateOuterDimensions(self):
        """
        - Return the outer dimensions of the drone racing gate, with scale (width=1.0, thickness=1.0, height=1.0)
        - Use this API in conjunction with simGetObjectScale(), simSetObjectScale(), simSetObjectPose() to generate arbitrary sized checkered gates with arbitrary poses.  

        Returns:
            Vector3r: width, thickness, height of the gate inner cavity in meters. 
            See https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/issues/28 for gate coordinate system
        """
        return Vector3r(3.2 / 1.5, 0.2, 3.2 / 1.5)

    def simGetObjectScale(self, object_name):
        """

        Args:
            object_name (str): Name of the object to poll

        Returns:
            Vector3r: scale vector of desired object
        """
        scale = self.client.call("simGetObjectScale", object_name)
        return Vector3r.from_msgpack(scale)

    def simGetObjectScaleInternal(self, object_name):
        scale = self.client.call("simGetObjectScaleInternal", object_name)
        return Vector3r.from_msgpack(scale)

    def simSetObjectScale(self, object_name, scale_vector):
        """

        Args:
            object_name (str): Name of the object to poll
            scale (Vector3r): desired scale of the object
        """
        return self.client.call("simSetObjectScale", object_name, scale_vector)

    def simLoadLevel(self, level_name):
        """Loads desired level
        
        Args:
            level_name (str): Description
        
        Returns:
            TYPE: Description
        """
        self.level_name = level_name
        return self.client.call("simLoadLevel", level_name)

    def simListSceneObjects(self, name_regex=".*"):
        """
        Lists the objects present in the environment

        Default behaviour is to list all objects, regex can be used to return smaller list of matching objects or actors

        Args:
            name_regex (str, optional): String to match actor names against, e.g. "Cylinder.*"

        Returns:
            list[str]: List containing all the names
        """
        return self.client.call("simListSceneObjects", name_regex)

    def simSetSegmentationObjectID(self, mesh_name, object_id, is_name_regex=False):
        """
        Set segmentation ID for specific objects

        See https://microsoft.github.io/AirSim/image_apis/#segmentation for details

        Args:
            mesh_name (str): Name of the mesh to set the ID of (supports regex)
            object_id (int): Object ID to be set, range 0-255

                             RBG values for IDs can be seen at https://microsoft.github.io/AirSim/seg_rgbs.txt
            is_name_regex (bool, optional): Whether the mesh name is a regex

        Returns:
            bool: If the mesh was found
        """
        return self.client.call(
            "simSetSegmentationObjectID", mesh_name, object_id, is_name_regex
        )

    def simGetSegmentationObjectID(self, mesh_name):
        """
        Returns Object ID for the given mesh name

        Mapping of Object IDs to RGB values can be seen at https://microsoft.github.io/AirSim/seg_rgbs.txt

        Args:
            mesh_name (str): Name of the mesh to get the ID of
        """
        return self.client.call("simGetSegmentationObjectID", mesh_name)

    def simPrintLogMessage(self, message, message_param="", severity=0):
        """
        Prints the specified message in the simulator's window.

        If message_param is supplied, then it's printed next to the message and in that case if this API is called with same message value
        but different message_param again then previous line is overwritten with new line (instead of API creating new line on display).

        For example, `simPrintLogMessage("Iteration: ", to_string(i))` keeps updating same line on display when API is called with different values of i.
        The valid values of severity parameter is 0 to 3 inclusive that corresponds to different colors.

        Args:
            message (str): Message to be printed
            message_param (str, optional): Parameter to be printed next to the message
            severity (int, optional): Range 0-3, inclusive, corresponding to the severity of the message
        """
        self.client.call("simPrintLogMessage", message, message_param, severity)

    def simGetCameraInfo(self, camera_name, vehicle_name=""):
        """
        Get details about the camera

        Args:
            camera_name (str): Name of the camera, for backwards compatibility, ID numbers such as 0,1,etc. can also be used
            vehicle_name (str, optional): Vehicle which the camera is associated with

        Returns:
            CameraInfo:
        """
        # TODO: below str() conversion is only needed for legacy reason and should be removed in future
        return CameraInfo.from_msgpack(
            self.client.call("simGetCameraInfo", str(camera_name), vehicle_name)
        )

    def simGetDistortionParams(self, camera_name, vehicle_name=""):
        """
        Get camera distortion parameters
        Args:
            camera_name (str): Name of the camera, for backwards compatibility, ID numbers such as 0,1,etc. can also be used
            vehicle_name (str, optional): Vehicle which the camera is associated with
        Returns:
            List (float): List of distortion parameter values corresponding to K1, K2, K3, P1, P2 respectively.
        """

        return self.client.call(
            "simGetDistortionParams", str(camera_name), vehicle_name
        )

    def simSetDistortionParams(self, camera_name, distortion_params, vehicle_name=""):
        """
        Set camera distortion parameters
        Args:
            camera_name (str): Name of the camera, for backwards compatibility, ID numbers such as 0,1,etc. can also be used
            distortion_params (dict): Dictionary of distortion param names and corresponding values
                                        {"K1": 0.0, "K2": 0.0, "K3": 0.0, "P1": 0.0, "P2": 0.0}
            vehicle_name (str, optional): Vehicle which the camera is associated with
        """

        for param_name, value in distortion_params.items():
            self.client.call(
                "simSetDistortionParam",
                str(camera_name),
                param_name,
                value,
                vehicle_name,
            )

    def simSetDistortionParam(self, camera_name, param_name, value, vehicle_name=""):
        """
        Set single camera distortion parameter
        Args:
            camera_name (str): Name of the camera, for backwards compatibility, ID numbers such as 0,1,etc. can also be used
            param_name (str): Name of distortion parameter
            value (float): Value of distortion parameter
            vehicle_name (str, optional): Vehicle which the camera is associated with
        """
        self.client.call(
            "simSetDistortionParam", str(camera_name), param_name, value, vehicle_name
        )

    def simSetCameraOrientation(self, camera_name, orientation, vehicle_name=""):
        """
        - Control the orientation of a selected camera

        Args:
            camera_name (str): Name of the camera to be controlled
            orientation (Quaternionr): Quaternion representing the desired orientation of the camera
            vehicle_name (str, optional): Name of vehicle which the camera corresponds to
        """
        # TODO: below str() conversion is only needed for legacy reason and should be removed in future
        self.client.call(
            "simSetCameraOrientation", str(camera_name), orientation, vehicle_name
        )

    def simSetCameraFov(self, camera_name, fov_degrees, vehicle_name=""):
        """
        - Control the field of view of a selected camera

        Args:
            camera_name (str): Name of the camera to be controlled
            fov_degrees (float): Value of field of view in degrees
            vehicle_name (str, optional): Name of vehicle which the camera corresponds to
        """
        # TODO: below str() conversion is only needed for legacy reason and should be removed in future
        self.client.call("simSetCameraFov", str(camera_name), fov_degrees, vehicle_name)

    def simGetGroundTruthKinematics(self, vehicle_name=""):
        """
        Get Ground truth kinematics of the vehicle

        Args:
            vehicle_name (str, optional): Name of the vehicle

        Returns:
            KinematicsState: Ground truth of the vehicle
        """
        kinematics_state = self.client.call("simGetGroundTruthKinematics", vehicle_name)
        return KinematicsState.from_msgpack(kinematics_state)

    simGetGroundTruthKinematics.__annotations__ = {"return": KinematicsState}

    def simGetGroundTruthEnvironment(self, vehicle_name=""):
        """
        Get ground truth environment state

        Args:
            vehicle_name (str, optional): Name of the vehicle

        Returns:
            EnvironmentState: Ground truth environment state
        """
        env_state = self.client.call("simGetGroundTruthEnvironment", vehicle_name)
        return EnvironmentState.from_msgpack(env_state)

    simGetGroundTruthEnvironment.__annotations__ = {"return": EnvironmentState}

    # sensor APIs
    def getImuData(self, imu_name="", vehicle_name=""):
        """
        Args:
            imu_name (str, optional): Name of IMU to get data from, specified in settings.json
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to

        Returns:
            ImuData:
        """
        return ImuData.from_msgpack(
            self.client.call("getImuData", imu_name, vehicle_name)
        )

    def getBarometerData(self, barometer_name="", vehicle_name=""):
        """
        Args:
            barometer_name (str, optional): Name of Barometer to get data from, specified in settings.json
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to

        Returns:
            BarometerData:
        """
        return BarometerData.from_msgpack(
            self.client.call("getBarometerData", barometer_name, vehicle_name)
        )

    def getMagnetometerData(self, magnetometer_name="", vehicle_name=""):
        """
        Args:
            magnetometer_name (str, optional): Name of Magnetometer to get data from, specified in settings.json
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to

        Returns:
            MagnetometerData:
        """
        return MagnetometerData.from_msgpack(
            self.client.call("getMagnetometerData", magnetometer_name, vehicle_name)
        )

    def getGpsData(self, gps_name="", vehicle_name=""):
        """
        Args:
            gps_name (str, optional): Name of GPS to get data from, specified in settings.json
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to

        Returns:
            GpsData:
        """
        return GpsData.from_msgpack(
            self.client.call("getGpsData", gps_name, vehicle_name)
        )

    def getDistanceSensorData(self, distance_sensor_name="", vehicle_name=""):
        """
        Args:
            distance_sensor_name (str, optional): Name of Distance Sensor to get data from, specified in settings.json
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to

        Returns:
            DistanceSensorData:
        """
        return DistanceSensorData.from_msgpack(
            self.client.call(
                "getDistanceSensorData", distance_sensor_name, vehicle_name
            )
        )

    def getLidarData(self, lidar_name="", vehicle_name=""):
        """
        Args:
            lidar_name (str, optional): Name of Lidar to get data from, specified in settings.json
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to

        Returns:
            LidarData:
        """
        return LidarData.from_msgpack(
            self.client.call("getLidarData", lidar_name, vehicle_name)
        )

    def simGetLidarSegmentation(self, lidar_name="", vehicle_name=""):
        """
        Returns Segmentation ID of each point's collided object in the last Lidar update

        Args:
            lidar_name (str, optional): Name of Lidar sensor
            vehicle_name (str, optional): Name of the vehicle wth the sensor

        Returns:
            list[int]: Segmentation IDs of the objects
        """
        return self.client.call("simGetLidarSegmentation", lidar_name, vehicle_name)

    #  Plotting APIs
    def simFlushPersistentMarkers(self):
        """
        Clear any persistent markers - those plotted with setting `is_persistent=True` in the APIs below
        """
        self.client.call("simFlushPersistentMarkers")

    def simPlotPoints(
        self,
        points,
        color_rgba=[1.0, 0.0, 0.0, 1.0],
        size=10.0,
        duration=-1.0,
        is_persistent=False,
    ):
        """
        Plot a list of 3D points in World NED frame

        Args:
            points (list[Vector3r]): List of Vector3r objects
            color_rgba (list, optional): desired RGBA values from 0.0 to 1.0
            size (float, optional): Size of plotted point
            duration (float, optional): Duration (seconds) to plot for
            is_persistent (bool, optional): If set to True, the desired object will be plotted for infinite time.
        """
        self.client.call(
            "simPlotPoints", points, color_rgba, size, duration, is_persistent
        )

    def simPlotLineStrip(
        self,
        points,
        color_rgba=[1.0, 0.0, 0.0, 1.0],
        thickness=5.0,
        duration=-1.0,
        is_persistent=False,
    ):
        """
        Plots a line strip in World NED frame, defined from points[0] to points[1], points[1] to points[2], ... , points[n-2] to points[n-1]

        Args:
            points (list[Vector3r]): List of 3D locations of line start and end points, specified as Vector3r objects
            color_rgba (list, optional): desired RGBA values from 0.0 to 1.0
            thickness (float, optional): Thickness of line
            duration (float, optional): Duration (seconds) to plot for
            is_persistent (bool, optional): If set to True, the desired object will be plotted for infinite time.
        """
        self.client.call(
            "simPlotLineStrip", points, color_rgba, thickness, duration, is_persistent
        )

    def simPlotLineList(
        self,
        points,
        color_rgba=[1.0, 0.0, 0.0, 1.0],
        thickness=5.0,
        duration=-1.0,
        is_persistent=False,
    ):
        """
        Plots a line strip in World NED frame, defined from points[0] to points[1], points[2] to points[3], ... , points[n-2] to points[n-1]

        Args:
            points (list[Vector3r]): List of 3D locations of line start and end points, specified as Vector3r objects. Must be even
            color_rgba (list, optional): desired RGBA values from 0.0 to 1.0
            thickness (float, optional): Thickness of line
            duration (float, optional): Duration (seconds) to plot for
            is_persistent (bool, optional): If set to True, the desired object will be plotted for infinite time.
        """
        self.client.call(
            "simPlotLineList", points, color_rgba, thickness, duration, is_persistent
        )

    def simPlotArrows(
        self,
        points_start,
        points_end,
        color_rgba=[1.0, 0.0, 0.0, 1.0],
        thickness=5.0,
        arrow_size=2.0,
        duration=-1.0,
        is_persistent=False,
    ):
        """
        Plots a list of arrows in World NED frame, defined from points_start[0] to points_end[0], points_start[1] to points_end[1], ... , points_start[n-1] to points_end[n-1]

        Args:
            points_start (list[Vector3r]): List of 3D start positions of arrow start positions, specified as Vector3r objects
            points_end (list[Vector3r]): List of 3D end positions of arrow start positions, specified as Vector3r objects
            color_rgba (list, optional): desired RGBA values from 0.0 to 1.0
            thickness (float, optional): Thickness of line
            arrow_size (float, optional): Size of arrow head
            duration (float, optional): Duration (seconds) to plot for
            is_persistent (bool, optional): If set to True, the desired object will be plotted for infinite time.
        """
        self.client.call(
            "simPlotArrows",
            points_start,
            points_end,
            color_rgba,
            thickness,
            arrow_size,
            duration,
            is_persistent,
        )

    def simPlotStrings(
        self,
        strings,
        positions,
        scale=5,
        color_rgba=[1.0, 0.0, 0.0, 1.0],
        duration=-1.0,
    ):
        """
        Plots a list of strings at desired positions in World NED frame.

        Args:
            strings (list[String], optional): List of strings to plot
            positions (list[Vector3r]): List of positions where the strings should be plotted. Should be in one-to-one correspondence with the strings' list
            scale (float, optional): Font scale of transform name
            color_rgba (list, optional): desired RGBA values from 0.0 to 1.0
            duration (float, optional): Duration (seconds) to plot for
        """
        self.client.call(
            "simPlotStrings", strings, positions, scale, color_rgba, duration
        )

    def simPlotTransforms(
        self, poses, scale=5.0, thickness=5.0, duration=-1.0, is_persistent=False
    ):
        """
        Plots a list of transforms in World NED frame.

        Args:
            poses (list[Pose]): List of Pose objects representing the transforms to plot
            scale (float, optional): Length of transforms' axes
            thickness (float, optional): Thickness of transforms' axes
            duration (float, optional): Duration (seconds) to plot for
            is_persistent (bool, optional): If set to True, the desired object will be plotted for infinite time.
        """
        self.client.call(
            "simPlotTransforms", poses, scale, thickness, duration, is_persistent
        )

    def simPlotTransformsWithNames(
        self,
        poses,
        names,
        tf_scale=5.0,
        tf_thickness=5.0,
        text_scale=10.0,
        text_color_rgba=[1.0, 0.0, 0.0, 1.0],
        duration=-1.0,
    ):
        """
        Plots a list of transforms with their names in World NED frame.

        Args:
            poses (list[Pose]): List of Pose objects representing the transforms to plot
            names (list[string]): List of strings with one-to-one correspondence to list of poses
            tf_scale (float, optional): Length of transforms' axes
            tf_thickness (float, optional): Thickness of transforms' axes
            text_scale (float, optional): Font scale of transform name
            text_color_rgba (list, optional): desired RGBA values from 0.0 to 1.0 for the transform name
            duration (float, optional): Duration (seconds) to plot for
        """
        self.client.call(
            "simPlotTransformsWithNames",
            poses,
            names,
            tf_scale,
            tf_thickness,
            text_scale,
            text_color_rgba,
            duration,
        )

    def cancelLastTask(self, vehicle_name=""):
        """
        Cancel previous Async task

        Args:
            vehicle_name (str, optional): Name of the vehicle
        """
        self.client.call("cancelLastTask", vehicle_name)

    def waitOnLastTask(self, timeout_sec=float("nan")):
        """
        Wait for the last Async task to complete

        Args:
            timeout_sec (float, optional): Time for the task to complete

        Returns:
            bool: Result of the last task

                  True if the task completed without cancellation or timeout
        """
        return self.client.call("waitOnLastTask", timeout_sec)

    def simCreateVoxelGrid(self, position, x, y, z, res, of):
        """
        Construct and save a binvox-formatted voxel grid of environment. Voxel grid
        stores the binary occupancy of the defined space as boolean values. 

        Stored as a binvox to allow easy conversion to octomaps. 

        Args:
            position (Vector3r): Position around which voxel grid is centered in m
            x, y, z (float): Size of each voxel grid dimension in m
            res (float): Resolution of voxel grid in m
            of (str): Name of output file to save voxel grid as
        """
        return self.client.call("simCreateVoxelGrid", position, x, y, z, res, of)

    def simBuildSDF(self, position, x, y, z, res):
        """
        Construct a signed distance field of the environment centered at position, 
        and with dimensions (x, y, z). Internally, the SDF is stored as a special 
        case of a voxel grid with floating point distances instead of boolean occupancy.

        Args:
            position (Vector3r): Global position around which field is centered in m
            x, y, z (float): Size of distance field dimensions in m
            res (float): Resolution of distance field in m
        """
        return self.client.call("simBuildSDF", position, x, y, z, res)

    def simCheckOccupancy(self, position):
        """
        Check and return occupancy of a point. Requires signed distance field to be 
        built beforehand. 

        Args:
            position (Vector3r): Global position at which occupancy is to be checked (m)
        """
        return self.client.call("simCheckOccupancy", position)

    def simGetSignedDistance(self, position):
        """
        Get signed distance of a point (distance to the closest 'object surface') 
        in the environment. Requires signed distance field to be built beforehand. 

        Distance is positive if the point is in free space, and negative if the point is 
        inside an object. 

        Args:
            position (Vector3r): Global position at which distance is to be computed (m)

        Returns:
            dist (float)
        """
        return self.client.call("simGetSignedDistance", position)

    def simGetSignedDistances(self, positions):
        """
        Get signed distance of a list of points (distance to the closest 'object surface') 
        in the environment. Requires signed distance field to be built beforehand. 

        Distance is positive if the point is in free space, and negative if the point is 
        inside an object. 

        Args:
            positions (list): List of global positions at which distance is to be computed (m)

        Returns:
            dists (list)
        """
        return self.client.call("simGetSignedDistances", positions)

    def simGetSDFGradient(self, position):
        """
        Get the SDF gradient at a point (vector pointing away from the closest
        'object surface') in the environment. Requires signed distance field to be built 
        beforehand. 

        Args:
            position (Vector3r): Global position at which gradient is to be computed (m)

        Returns:
            gradient (Vector3r): SDF gradient at the position
        """
        return self.client.call("simGetSDFGradient", position)

    def simProjectToFreeSpace(self, position, mindist):
        """
        Project a given point into free space using the SDF, with a specified minimum clearance 
        from existing objects. Returns the same point if the point is already free, else follows 
        the SDF gradient to find a free point that satisfies the minimum distance constraint.

        Args:
            position (Vector3r): Global position to project (m)
            mindist (float): Minimum distance from objects to satisfy when finding the free point

        Returns:
            free_pt (Vector3r): Projected position in free space
        """
        return self.client.call("simProjectToFreeSpace", position, mindist)

    def simSaveSDF(self, filepath):
        """
        Save the constructed signed distance field to a file.

        Args:
            filepath (str): Filename to save the SDF to
        """
        return self.client.call("simSaveSDF", filepath)

    def simLoadSDF(self, filepath):
        """
        Load a saved signed distance field.

        Args:
            filepath (str): Filename to load the SDF from
        """
        return self.client.call("simLoadSDF", filepath)

    def simSetWind(self, wind):
        """
        Set simulated wind, in World frame, NED direction, m/s
        Args:
            wind (Vector3r): Wind, in World frame, NED direction, in m/s 
        """
        self.client.call("simSetWind", wind)


# -----------------------------------  Multirotor APIs ---------------------------------------------
class MultirotorClient(VehicleClient, object):
    def __init__(self, ip="", port=41451, timeout_value=3600):
        super(MultirotorClient, self).__init__(ip, port, timeout_value)

        self.race_tier: int = None
        self.level_name: str
        self.MAX_NUMBER_OF_GETOBJECTPOSE_TRIALS = 10

    def takeoffAsync(self, timeout_sec=20, vehicle_name=""):
        """
        Takeoff vehicle to 3m above ground. Vehicle should not be moving when this API is used

        Args:
            timeout_sec (int, optional): Timeout for the vehicle to reach desired altitude
            vehicle_name (str, optional): Name of the vehicle to send this command to

        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async("takeoff", timeout_sec, vehicle_name)

    def landAsync(self, timeout_sec=60, vehicle_name=""):
        """
        Land the vehicle

        Args:
            timeout_sec (int, optional): Timeout for the vehicle to land
            vehicle_name (str, optional): Name of the vehicle to send this command to

        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async("land", timeout_sec, vehicle_name)

    def goHomeAsync(self, timeout_sec=3e38, vehicle_name=""):
        """
        Return vehicle to Home i.e. Launch location

        Args:
            timeout_sec (int, optional): Timeout for the vehicle to reach desired altitude
            vehicle_name (str, optional): Name of the vehicle to send this command to

        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async("goHome", timeout_sec, vehicle_name)

    # APIs for control
    def moveByAngleZAsync(self, pitch, roll, z, yaw, duration, vehicle_name=""):
        return self.client.call_async(
            "moveByAngleZ", pitch, roll, z, yaw, duration, vehicle_name
        )

    def moveByAngleThrottleAsync(
        self, pitch, roll, throttle, yaw_rate, duration, vehicle_name=""
    ):
        return self.client.call_async(
            "moveByAngleThrottle",
            pitch,
            roll,
            throttle,
            yaw_rate,
            duration,
            vehicle_name,
        )

    def moveByVelocityAsync(
        self,
        vx,
        vy,
        vz,
        duration,
        drivetrain=DrivetrainType.MaxDegreeOfFreedom,
        yaw_mode=YawMode(),
        vehicle_name="",
    ):
        """
        Args:
            vx (float): desired velocity in world (NED) X axis
            vy (float): desired velocity in world (NED) Y axis
            vz (float): desired velocity in world (NED) Z axis
            duration (float): Desired amount of time (seconds), to send this command for
            drivetrain (DrivetrainType, optional):
            yaw_mode (YawMode, optional):
            vehicle_name (str, optional): Name of the multirotor to send this command to

        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async(
            "moveByVelocity", vx, vy, vz, duration, drivetrain, yaw_mode, vehicle_name
        )

    def moveByVelocityZAsync(
        self,
        vx,
        vy,
        z,
        duration,
        drivetrain=DrivetrainType.MaxDegreeOfFreedom,
        yaw_mode=YawMode(),
        vehicle_name="",
    ):
        """
        
        Args:
            vx (float): desired velocity in world (NED) X axis 
            vy (float): desired velocity in world (NED) Y axis
            z (float): desired altitude in world (NED) Z axis
            duration (float): Desired amount of time (seconds), to send this command for
            drivetrain (DrivetrainType, optional): Description
            yaw_mode (YawMode, optional): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async(
            "moveByVelocityZ", vx, vy, z, duration, drivetrain, yaw_mode, vehicle_name
        )

    def moveOnPathAsync(
        self,
        path,
        velocity,
        timeout_sec=3e38,
        drivetrain=DrivetrainType.MaxDegreeOfFreedom,
        yaw_mode=YawMode(),
        lookahead=-1,
        adaptive_lookahead=1,
        vehicle_name="",
    ):
        """
        
        Args:
            path (TYPE): Description
            velocity (TYPE): Description
            timeout_sec (float, optional): Description
            drivetrain (TYPE, optional): Description
            yaw_mode (TYPE, optional): Description
            lookahead (TYPE, optional): Description
            adaptive_lookahead (int, optional): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            TYPE: Description
        """
        return self.client.call_async(
            "moveOnPath",
            path,
            velocity,
            timeout_sec,
            drivetrain,
            yaw_mode,
            lookahead,
            adaptive_lookahead,
            vehicle_name,
        )

    def moveToPositionAsync(
        self,
        x,
        y,
        z,
        velocity,
        timeout_sec=3e38,
        drivetrain=DrivetrainType.MaxDegreeOfFreedom,
        yaw_mode=YawMode(),
        lookahead=-1,
        adaptive_lookahead=1,
        vehicle_name="",
    ):
        """
        
        Args:
            x (float): Description
            y (float): Description
            z (float): Description
            velocity (float): Description
            timeout_sec (float, optional): Description
            drivetrain (DrivetrainType, optional): Description
            yaw_mode (YawMode, optional): Description
            lookahead (float, optional): Description
            adaptive_lookahead (int, optional): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            bool: Success
        """
        return self.client.call_async(
            "moveToPosition",
            x,
            y,
            z,
            velocity,
            timeout_sec,
            drivetrain,
            yaw_mode,
            lookahead,
            adaptive_lookahead,
            vehicle_name,
        )

    def moveToZAsync(
        self,
        z,
        velocity,
        timeout_sec=3e38,
        yaw_mode=YawMode(),
        lookahead=-1,
        adaptive_lookahead=1,
        vehicle_name="",
    ):
        """
        
        Args:
            z (float): Description
            velocity (float): Description
            timeout_sec (float, optional): Description
            yaw_mode (YawMode, optional): Description
            lookahead (float, optional): Description
            adaptive_lookahead (int, optional): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            bool: Success
        """
        return self.client.call_async(
            "moveToZ",
            z,
            velocity,
            timeout_sec,
            yaw_mode,
            lookahead,
            adaptive_lookahead,
            vehicle_name,
        )

    def moveToYawAsync(self, yaw, timeout_sec=3e38, margin=5, vehicle_name=""):
        """
        
        Args:
            yaw_rate (float): Desired yaw angle, in **degrees per second**.
            timeout_sec (float, optional): Description
            margin (int, optional): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            bool: Success
        """
        return self.client.call_async(
            "rotateToYaw", yaw, timeout_sec, margin, vehicle_name
        )

    def moveByYawRateAsync(self, yaw_rate, duration, vehicle_name=""):
        """
        
        Args:
            yaw_rate (float): Desired yaw rate, in **degrees per second**.
            duration (float): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            bool: Success
        """
        return self.client.call_async(
            "rotateByYawRate", yaw_rate, duration, vehicle_name
        )

    def moveOnSplineAsync(
        self,
        waypoints,
        vel_max=15.0,
        acc_max=7.5,
        add_position_constraint=True,
        add_velocity_constraint=True,
        add_acceleration_constraint=False,
        viz_traj=True,
        viz_traj_color_rgba=[1.0, 0.0, 0.0, 1.0],
        replan_from_lookahead=True,
        replan_lookahead_sec=1.0,
        vehicle_name="",
    ):
        """
        - Fits a minimum jerk trajectory to the list of given 3D waypoints (specified by the waypoints parameter).
        - Uses ETHZ-ASL's `mav_trajectory_generation <https://github.com/ethz-asl/mav_trajectory_generation>`_ as the trajectory planning backend.
        - Tracks the references positions and velocities using a pure pursuit tracking controller. 
        - The gains of the pure pursuit tracking controller are set by setTrajectoryTrackerGains.
        - Reference yaws are allocated along the tangent of the trajectory. Hence the drone will always look at the direction along which it is flying, behaving like a 3D car.
        - Note: setTrajectoryTrackerGains() must be called once before calling moveOnSpline()

        Args:
            waypoints (list[Vector3r]): 
                - List of 3D waypoints, defined in local NED frame of the vehicle to track.
            vel_max (float, optional): 
                - Maximum magnitude of velocity along trajectory
            acc_max (float, optional):
                - Maximum magnitude of acceleration along trajectory
            add_position_constraint (bool, optional): 
                - Add a start constraint at current position, so that the planned trajectory is smooth if the drone is already moving. 
                - If replan_from_lookahead is False, and add_position_constraint is False, trajectory starts from the first element of the "waypoints" list param. 
                - If replan_from_lookahead is False, and add_position_constraint is True, a position constraint is added at current odometry, and so the trajectory starts from current position. 
                - If replan_from_lookahead is True, a position constraint trajectory is always added at look-ahead point regardless of the value of "add_position_constraint", and so the trajectory starts from the lookahead point. 
                - See below for the definition of "look-ahead point".   
            add_velocity_constraint (bool, optional): 
                - Should only be set to True if add_position_constraint is True. 
                - If replan_from_lookahead is True, a velocity constraint is added at current odometry. 
                - If replan_from_lookahead is True, a velocity constraint is added at lookahead point. 
            add_acceleration_constraint (bool, optional): 
                - Should only be set to True if add_velocity_constraint (and therefore, add_position_constraint) is True. 
                - If replan_from_lookahead is True, an acceleration constraint is added at current odometry. 
                - If replan_from_lookahead is True, an acceleration constraint is added at lookahead point. 
            viz_traj (bool, optional): 
                - set this to True to visualize trajectory in unreal window. 
                - Note that visualization would appear in the FPV image, so this should only be used for debugging. 
            viz_traj_color_rgba (list, optional): 
                - list of 4 floats from 0.0 to 1.0 that determines RGBA value of trajectory visualization. 
                - Example: viz_traj_color_rgba=[1.0, 0.0, 0.0, 1.0] corresponds to Red
            replan_from_lookahead(bool, optional):
                - If this is set to true, the trajectory will start from the "look-ahead point" associated with the trajectory the drone is currently following. 
                - The lookahead point is defined by the value of the replan_lookahead_sec paramater sent in the *previous* call to moveOnSpline. 
            replan_lookahead_sec(float, optional): 
                - Defines the lookahead point by sampling the current trajectory replan_lookahead_sec number of seconds ahead. 
                - If replan_from_lookahead is passed as True in the *next* call to moveOnSpline, the *next* call's trajectory will start from the lookahead point defined by the *current* call's replan_lookahead_sec
            vehicle_name (str, optional): 
                - Name of the multirotor to send this command to 
        
        Returns:
            bool: Success
        """
        return self.client.call_async(
            "moveOnSpline",
            waypoints,
            add_position_constraint,
            add_velocity_constraint,
            add_acceleration_constraint,
            vel_max,
            acc_max,
            viz_traj,
            viz_traj_color_rgba,
            replan_from_lookahead,
            replan_lookahead_sec,
            vehicle_name,
        )

    def moveOnSplineVelConstraintsAsync(
        self,
        waypoints,
        velocity_constraints,
        vel_max=15.0,
        acc_max=7.5,
        add_position_constraint=True,
        add_velocity_constraint=True,
        add_acceleration_constraint=False,
        viz_traj=True,
        viz_traj_color_rgba=[1.0, 0.0, 0.0, 0.4],
        replan_from_lookahead=True,
        replan_lookahead_sec=1.0,
        vehicle_name="",
    ):
        """
        - Fits a minimum jerk trajectory to the list of given 3D waypoints (specified by the waypoints parameter).
        - Also adds corresponding 3D velocity vector constraints (specified by the velocity_constraints parameter).
        - Uses ETHZ-ASL's `mav_trajectory_generation <https://github.com/ethz-asl/mav_trajectory_generation>`_ as the trajectory planning backend.
        - Tracks the references positions and velocities using a pure pursuit tracking controller. 
        - The gains of the pure pursuit tracking controller are set by setTrajectoryTrackerGains.
        - Reference yaws are allocated along the tangent of the trajectory. Hence the drone will always look at the direction along which it is flying, behaving like a 3D car.
        - Reference yaws are allocated along the tangent of the trajectory. 
        - Note: setTrajectoryTrackerGains() must be called once before calling moveOnSpline()
        
        Args:
            waypoints (list[Vector3r]): 
                - List of 3D waypoints, defined in local NED frame of the vehicle to track.
            velocity_constraints (list[Vector3r]): 
                - List of 3D velocity vector constraints, defined in local NED frame of the vehicle to track.
            vel_max (float, optional):
                - Maximum magnitude of velocity along trajectory
            acc_max (float, optional): 
            - Maximum magnitude of acceleration along trajectory
            add_position_constraint (bool, optional):
                - Add a start constraint at current position, so that the planned trajectory is smooth if the drone is already moving. 
                - If replan_from_lookahead is False, and add_position_constraint is False, trajectory starts from the first element of the "waypoints" list param. 
                - If replan_from_lookahead is False, and add_position_constraint is True, a position constraint is added at current odometry, and so the trajectory starts from current position. 
                - If replan_from_lookahead is True, a position constraint trajectory is always added at look-ahead point regardless of the value of "add_position_constraint", and so the trajectory starts from the lookahead point. 
                - See below for the definition of "look-ahead point".   
            add_velocity_constraint (bool, optional):
                - Should only be set to True if add_position_constraint is True. 
                - If replan_from_lookahead is True, a velocity constraint is added at current odometry. 
                - If replan_from_lookahead is True, a velocity constraint is added at lookahead point. 
            add_acceleration_constraint (bool, optional): 
                - Should only be set to True if add_velocity_constraint (and therefore, add_position_constraint) is True. 
                - If replan_from_lookahead is True, an acceleration constraint is added at current odometry. 
                - If replan_from_lookahead is True, an acceleration constraint is added at lookahead point. 
            viz_traj (bool, optional): 
                - set this to True to visualize trajectory in unreal window. 
                - Note that visualization would appear in the FPV image, so this should only be used for debugging. 
            viz_traj_color_rgba (list, optional): 
                - list of 4 floats from 0.0 to 1.0 that determines RGBA value of trajectory visualization. 
                - Example: viz_traj_color_rgba=[1.0, 0.0, 0.0, 1.0] corresponds to Red
            replan_from_lookahead(bool, optional):
                - If this is set to true, the trajectory will start from the "look-ahead point" associated with the trajectory the drone is currently following. 
                - The lookahead point is defined by the value of the replan_lookahead_sec paramater sent in the *previous* call to moveOnSpline. 
            replan_lookahead_sec(float, optional): 
                - Defines the lookahead point by sampling the current trajectory replan_lookahead_sec number of seconds ahead. 
                - If replan_from_lookahead is passed as True in the *next* call to moveOnSpline, the *next* call's trajectory will start from the lookahead point defined by the *current* call's replan_lookahead_sec
            vehicle_name (str, optional): 
                - Name of the multirotor to send this command to 

        Returns:
            bool: Success
        """
        return self.client.call_async(
            "moveOnSplineVelConstraints",
            waypoints,
            velocity_constraints,
            add_position_constraint,
            add_velocity_constraint,
            add_acceleration_constraint,
            vel_max,
            acc_max,
            viz_traj,
            viz_traj_color_rgba,
            replan_from_lookahead,
            replan_lookahead_sec,
            vehicle_name,
        )

    def clearTrajectory(self, vehicle_name=""):
        """
        Clears, and stops following the current trajectory (see moveOnSpline() or moveOnSplineVelConstraintsAsyn,c if any. 
        
        Args:
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            TYPE: Description
        """

        self.client.call("clearTrajectory", vehicle_name)

    def setTrajectoryTrackerGains(
        self, gains=TrajectoryTrackerGains(), vehicle_name=""
    ):
        """
        - Sets trajectory tracker gains for moveOnSplineAsync, moveOnSplineVelConstraintsAsync. 
        - Must be called once before either of the moveOnSpline*() APIs is called
        
        Args:
            gains (TrajectoryTrackerGains): Pass TrajectoryTrackerGains() to set default values. Look at TrajectoryTrackerGains to set custom gains
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        """
        self.client.call(
            "setTrajectoryTrackerGains", *((gains.to_list(),) + (vehicle_name,))
        )

    def moveByManualAsync(
        self,
        vx_max,
        vy_max,
        z_min,
        duration,
        drivetrain=DrivetrainType.MaxDegreeOfFreedom,
        yaw_mode=YawMode(),
        vehicle_name="",
    ):
        """
        - Read current RC state and use it to control the vehicles.

        Parameters sets up the constraints on velocity and minimum altitude while flying. If RC state is detected to violate these constraints
        then that RC state would be ignored.

        Args:
            vx_max (float): max velocity allowed in x direction
            vy_max (float): max velocity allowed in y direction
            vz_max (float): max velocity allowed in z direction
            z_min (float): min z allowed for vehicle position
            duration (float): after this duration vehicle would switch back to non-manual mode
            drivetrain (DrivetrainType): when ForwardOnly, vehicle rotates itself so that its front is always facing the direction of travel. If MaxDegreeOfFreedom then it doesn't do that (crab-like movement)
            yaw_mode (YawMode): Specifies if vehicle should face at given angle (is_rate=False) or should be rotating around its axis at given rate (is_rate=True)
            vehicle_name (str, optional): Name of the multirotor to send this command to
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async(
            "moveByManual",
            vx_max,
            vy_max,
            z_min,
            duration,
            drivetrain,
            yaw_mode,
            vehicle_name,
        )

    def moveToPositionAsync(
        self,
        x,
        y,
        z,
        velocity,
        timeout_sec=3e38,
        drivetrain=DrivetrainType.MaxDegreeOfFreedom,
        yaw_mode=YawMode(),
        lookahead=-1,
        adaptive_lookahead=1,
        vehicle_name="",
    ):
        """
        
        Args:
            x (float): Description
            y (float): Description
            z (float): Description
            velocity (float): Description
            timeout_sec (float, optional): Description
            drivetrain (DrivetrainType, optional): Description
            yaw_mode (YawMode, optional): Description
            lookahead (float, optional): Description
            adaptive_lookahead (int, optional): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            bool: Success
        """
        return self.client.call_async(
            "moveToPosition",
            x,
            y,
            z,
            velocity,
            timeout_sec,
            drivetrain,
            yaw_mode,
            lookahead,
            adaptive_lookahead,
            vehicle_name,
        )

    def moveToZAsync(
        self,
        z,
        velocity,
        timeout_sec=3e38,
        yaw_mode=YawMode(),
        lookahead=-1,
        adaptive_lookahead=1,
        vehicle_name="",
    ):
        """
        
        Args:
            z (float): Description
            velocity (float): Description
            timeout_sec (float, optional): Description
            yaw_mode (YawMode, optional): Description
            lookahead (float, optional): Description
            adaptive_lookahead (int, optional): Description
            vehicle_name (str, optional): Name of the multirotor to send this command to 
        
        Returns:
            bool: Success
        """
        return self.client.call_async(
            "moveToZ",
            z,
            velocity,
            timeout_sec,
            yaw_mode,
            lookahead,
            adaptive_lookahead,
            vehicle_name,
        )

    def hoverAsync(self, vehicle_name=""):
        return self.client.call_async("hover", vehicle_name)

    def moveByRC(self, rcdata=RCData(), vehicle_name=""):
        return self.client.call("moveByRC", rcdata, vehicle_name)

    # low-level control API
    def moveByMotorPWMsAsync(
        self,
        front_right_pwm,
        rear_left_pwm,
        front_left_pwm,
        rear_right_pwm,
        duration,
        vehicle_name="",
    ):
        """
        - Directly control the motors using PWM values

        Args:
            front_right_pwm (float): PWM value for the front right motor (between 0.0 to 1.0)
            rear_left_pwm (float): PWM value for the rear left motor (between 0.0 to 1.0)
            front_left_pwm (float): PWM value for the front left motor (between 0.0 to 1.0)
            rear_right_pwm (float): PWM value for the rear right motor (between 0.0 to 1.0)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async(
            "moveByMotorPWMs",
            front_right_pwm,
            rear_left_pwm,
            front_left_pwm,
            rear_right_pwm,
            duration,
            vehicle_name,
        )

    def moveByRollPitchYawZAsync(self, roll, pitch, yaw, z, duration, vehicle_name=""):
        """
        - z is given in local NED frame of the vehicle.
        - Roll angle, pitch angle, and yaw angle set points are given in **radians**, in the body frame.
        - The body frame follows the Front Left Up (FLU) convention, and right-handedness.

        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor.

            | Clockwise rotation about this axis defines a positive **roll** angle.
            | Hence, rolling with a positive angle is equivalent to translating in the **right** direction, w.r.t. our FLU body frame.

            - Y axis is along the **Left** direction of the quadrotor.

            | Clockwise rotation about this axis defines a positive **pitch** angle.
            | Hence, pitching with a positive angle is equivalent to translating in the **front** direction, w.r.t. our FLU body frame.

            - Z axis is along the **Up** direction.

            | Clockwise rotation about this axis defines a positive **yaw** angle.
            | Hence, yawing with a positive angle is equivalent to rotated towards the **left** direction wrt our FLU body frame. Or in an anticlockwise fashion in the body XY / FL plane.

        Args:
            roll (float): Desired roll angle, in radians.
            pitch (float): Desired pitch angle, in radians.
            yaw (float): Desired yaw angle, in radians.
            z (float): Desired Z value (in local NED frame of the vehicle)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to

        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async(
            "moveByRollPitchYawZ", roll, -pitch, -yaw, z, duration, vehicle_name
        )

    def moveByRollPitchYawThrottleAsync(
        self, roll, pitch, yaw, throttle, duration, vehicle_name=""
    ):
        """
        - Desired throttle is between 0.0 to 1.0
        - Roll angle, pitch angle, and yaw angle are given in **radians**, in the body frame.
        - The body frame follows the Front Left Up (FLU) convention, and right-handedness.

        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor.

            | Clockwise rotation about this axis defines a positive **roll** angle.
            | Hence, rolling with a positive angle is equivalent to translating in the **right** direction, w.r.t. our FLU body frame.

            - Y axis is along the **Left** direction of the quadrotor.

            | Clockwise rotation about this axis defines a positive **pitch** angle.
            | Hence, pitching with a positive angle is equivalent to translating in the **front** direction, w.r.t. our FLU body frame.

            - Z axis is along the **Up** direction.

            | Clockwise rotation about this axis defines a positive **yaw** angle.
            | Hence, yawing with a positive angle is equivalent to rotated towards the **left** direction wrt our FLU body frame. Or in an anticlockwise fashion in the body XY / FL plane.

        Args:
            roll (float): Desired roll angle, in radians.
            pitch (float): Desired pitch angle, in radians.
            yaw (float): Desired yaw angle, in radians.
            throttle (float): Desired throttle (between 0.0 to 1.0)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to

        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async(
            "moveByRollPitchYawThrottle",
            roll,
            -pitch,
            -yaw,
            throttle,
            duration,
            vehicle_name,
        )

    def moveByRollPitchYawrateThrottleAsync(
        self, roll, pitch, yaw_rate, throttle, duration, vehicle_name=""
    ):
        """
        - Desired throttle is between 0.0 to 1.0
        - Roll angle, pitch angle, and yaw rate set points are given in **radians**, in the body frame.
        - The body frame follows the Front Left Up (FLU) convention, and right-handedness.

        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor.

            | Clockwise rotation about this axis defines a positive **roll** angle.
            | Hence, rolling with a positive angle is equivalent to translating in the **right** direction, w.r.t. our FLU body frame.

            - Y axis is along the **Left** direction of the quadrotor.

            | Clockwise rotation about this axis defines a positive **pitch** angle.
            | Hence, pitching with a positive angle is equivalent to translating in the **front** direction, w.r.t. our FLU body frame.

            - Z axis is along the **Up** direction.

            | Clockwise rotation about this axis defines a positive **yaw** angle.
            | Hence, yawing with a positive angle is equivalent to rotated towards the **left** direction wrt our FLU body frame. Or in an anticlockwise fashion in the body XY / FL plane.

        Args:
            roll (float): Desired roll angle, in radians.
            pitch (float): Desired pitch angle, in radians.
            yaw_rate (float): Desired yaw rate, in radian per second.
            throttle (float): Desired throttle (between 0.0 to 1.0)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to

        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async(
            "moveByRollPitchYawrateThrottle",
            roll,
            -pitch,
            -yaw_rate,
            throttle,
            duration,
            vehicle_name,
        )

    def moveByRollPitchYawrateZAsync(
        self, roll, pitch, yaw_rate, z, duration, vehicle_name=""
    ):
        """
        - z is given in local NED frame of the vehicle.
        - Roll angle, pitch angle, and yaw rate set points are given in **radians**, in the body frame.
        - The body frame follows the Front Left Up (FLU) convention, and right-handedness.

        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor.

            | Clockwise rotation about this axis defines a positive **roll** angle.
            | Hence, rolling with a positive angle is equivalent to translating in the **right** direction, w.r.t. our FLU body frame.

            - Y axis is along the **Left** direction of the quadrotor.

            | Clockwise rotation about this axis defines a positive **pitch** angle.
            | Hence, pitching with a positive angle is equivalent to translating in the **front** direction, w.r.t. our FLU body frame.

            - Z axis is along the **Up** direction.

            | Clockwise rotation about this axis defines a positive **yaw** angle.
            | Hence, yawing with a positive angle is equivalent to rotated towards the **left** direction wrt our FLU body frame. Or in an anticlockwise fashion in the body XY / FL plane.

        Args:
            roll (float): Desired roll angle, in radians.
            pitch (float): Desired pitch angle, in radians.
            yaw_rate (float): Desired yaw rate, in radian per second.
            z (float): Desired Z value (in local NED frame of the vehicle)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to

        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async(
            "moveByRollPitchYawrateZ",
            roll,
            -pitch,
            -yaw_rate,
            z,
            duration,
            vehicle_name,
        )

    def moveByAngleRatesZAsync(
        self, roll_rate, pitch_rate, yaw_rate, z, duration, vehicle_name=""
    ):
        """
        - z is given in local NED frame of the vehicle.
        - Roll rate, pitch rate, and yaw rate set points are given in **radians**, in the body frame.
        - The body frame follows the Front Left Up (FLU) convention, and right-handedness.

        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor.

            | Clockwise rotation about this axis defines a positive **roll** angle.
            | Hence, rolling with a positive angle is equivalent to translating in the **right** direction, w.r.t. our FLU body frame.

            - Y axis is along the **Left** direction of the quadrotor.

            | Clockwise rotation about this axis defines a positive **pitch** angle.
            | Hence, pitching with a positive angle is equivalent to translating in the **front** direction, w.r.t. our FLU body frame.

            - Z axis is along the **Up** direction.

            | Clockwise rotation about this axis defines a positive **yaw** angle.
            | Hence, yawing with a positive angle is equivalent to rotated towards the **left** direction wrt our FLU body frame. Or in an anticlockwise fashion in the body XY / FL plane.

        Args:
            roll_rate (float): Desired roll rate, in radians / second
            pitch_rate (float): Desired pitch rate, in radians / second
            yaw_rate (float): Desired yaw rate, in radians / second
            z (float): Desired Z value (in local NED frame of the vehicle)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to

        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async(
            "moveByAngleRatesZ",
            roll_rate,
            -pitch_rate,
            -yaw_rate,
            z,
            duration,
            vehicle_name,
        )

    def moveByAngleRatesThrottleAsync(
        self, roll_rate, pitch_rate, yaw_rate, throttle, duration, vehicle_name=""
    ):
        """
        - Desired throttle is between 0.0 to 1.0
        - Roll rate, pitch rate, and yaw rate set points are given in **radians**, in the body frame.
        - The body frame follows the Front Left Up (FLU) convention, and right-handedness.

        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor.

            | Clockwise rotation about this axis defines a positive **roll** angle.
            | Hence, rolling with a positive angle is equivalent to translating in the **right** direction, w.r.t. our FLU body frame.

            - Y axis is along the **Left** direction of the quadrotor.

            | Clockwise rotation about this axis defines a positive **pitch** angle.
            | Hence, pitching with a positive angle is equivalent to translating in the **front** direction, w.r.t. our FLU body frame.

            - Z axis is along the **Up** direction.

            | Clockwise rotation about this axis defines a positive **yaw** angle.
            | Hence, yawing with a positive angle is equivalent to rotated towards the **left** direction wrt our FLU body frame. Or in an anticlockwise fashion in the body XY / FL plane.

        Args:
            roll_rate (float): Desired roll rate, in radians / second
            pitch_rate (float): Desired pitch rate, in radians / second
            yaw_rate (float): Desired yaw rate, in radians / second
            throttle (float): Desired throttle (between 0.0 to 1.0)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to

        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async(
            "moveByAngleRatesThrottle",
            roll_rate,
            -pitch_rate,
            -yaw_rate,
            throttle,
            duration,
            vehicle_name,
        )

    def setAngleRateControllerGains(
        self, angle_rate_gains=AngleRateControllerGains(), vehicle_name=""
    ):
        """
        - Modifying these gains will have an affect on *ALL* move*() APIs.
            This is because any velocity setpoint is converted to an angle level setpoint which is tracked with an angle level controllers.
            That angle level setpoint is itself tracked with and angle rate controller.
        - This function should only be called if the default angle rate control PID gains need to be modified.

        Args:
            angle_rate_gains (AngleRateControllerGains):
                - Correspond to the roll, pitch, yaw axes, defined in the body frame.
                - Pass AngleRateControllerGains() to reset gains to default recommended values.
            vehicle_name (str, optional): Name of the multirotor to send this command to
        """
        self.client.call(
            "setAngleRateControllerGains",
            *(angle_rate_gains.to_lists() + (vehicle_name,)),
        )

    def setAngleLevelControllerGains(
        self, angle_level_gains=AngleLevelControllerGains(), vehicle_name=""
    ):
        """
        - Sets angle level controller gains (used by any API setting angle references - for ex: moveByRollPitchYawZAsync(), moveByRollPitchYawThrottleAsync(), etc)
        - Modifying these gains will also affect the behaviour of moveByVelocityAsync() API.
            This is because the AirSim flight controller will track velocity setpoints by converting them to angle set points.
        - This function should only be called if the default angle level control PID gains need to be modified.
        - Passing AngleLevelControllerGains() sets gains to default airsim values.

        Args:
            angle_level_gains (AngleLevelControllerGains):
                - Correspond to the roll, pitch, yaw axes, defined in the body frame.
                - Pass AngleLevelControllerGains() to reset gains to default recommended values.
            vehicle_name (str, optional): Name of the multirotor to send this command to
        """
        self.client.call(
            "setAngleLevelControllerGains",
            *(angle_level_gains.to_lists() + (vehicle_name,)),
        )

    def setVelocityControllerGains(
        self, velocity_gains=VelocityControllerGains(), vehicle_name=""
    ):
        """
        - Sets velocity controller gains for moveByVelocityAsync().
        - This function should only be called if the default velocity control PID gains need to be modified.
        - Passing VelocityControllerGains() sets gains to default airsim values.
        - Modifying the velocity controller gains will have an effect on the trajectory tracking behavior of moveOnSpline*() APIs, 
            as moveOnSpline*() APIs use a controller on the lines of the pure-pursuit approach, which is tracking the reference position and
            velocities of the reference trajectories, while minimizing cross-track errors in both position and velocity state, 
            by sending velocity commands (via moveByVelocityAsync()) in the backend.  
            If you change this, it might be worth playing with the gains of moveOnSpline() by using setTrajectoryTrackerGains()

        Args:
            velocity_gains (VelocityControllerGains):
                - Correspond to the world X, Y, Z axes.
                - Pass VelocityControllerGains() to reset gains to default recommended values.
                - Modifying velocity controller gains will have an affect on the behaviour of moveOnSplineAsync() and moveOnSplineVelConstraintsAsync(), as they both use velocity control to track the trajectory.
            vehicle_name (str, optional): Name of the multirotor to send this command to
        """
        self.client.call(
            "setVelocityControllerGains", *(velocity_gains.to_lists() + (vehicle_name,))
        )

    def setPositionControllerGains(
        self, position_gains=PositionControllerGains(), vehicle_name=""
    ):
        """
        Sets position controller gains for moveByPositionAsync.
        This function should only be called if the default position control PID gains need to be modified.

        Args:
            position_gains (PositionControllerGains):
                - Correspond to the X, Y, Z axes.
                - Pass PositionControllerGains() to reset gains to default recommended values.
            vehicle_name (str, optional): Name of the multirotor to send this command to
        """
        self.client.call(
            "setPositionControllerGains", *(position_gains.to_lists() + (vehicle_name,))
        )

    # query vehicle state
    def getMultirotorState(self, vehicle_name=""):
        """
        Args:
            vehicle_name (str, optional): Vehicle to get the state of

        Returns:
            MultirotorState:
        """
        return MultirotorState.from_msgpack(
            self.client.call("getMultirotorState", vehicle_name)
        )

    getMultirotorState.__annotations__ = {"return": MultirotorState}

    def simLogMultirotorState(self, is_enabled, vehicle_name=""):
        """
        Starts or stops high frequency logging of full multirotor state to a text file in Documents/AirSim.
        Values logged: Position, orientation, linear/angular velocity, linear/angular acceleration, 
        body forces and torques, rotor speeds and rotor torques.

        Args:
            is_enabled (bool, required): Start/stop.
            vehicle_name (str, optional): Vehicle to start logging for

        """
        self.client.call("simLogMultirotorState", is_enabled, vehicle_name)

    # Race APIs
    def simStartRace(self, tier=1):
        """ Starts an instance of a race in your given level, if valid."""
        self.race_tier = tier
        if tier == 2:
            self.client.call("simStartRace", tier)
            return
        else:
            client_instance = self.__class__()
            competitor = BaselineRacer(
                client_instance,
                viz_traj=False,
                viz_traj_color_rgba=[1.0, 1.0, 0.0, 1.0],
            )
            competitor.level_name = self.level_name
            competitor.initialize_drone()
            competitor.gate_poses_ground_truth = [
                self.__internalRandomGoalZone(gate)
                for gate in sorted(self.simListSceneObjects(".*[Gg]ate.*"))
            ]
            if self.level_name == "Soccer_Field_Medium":
                curr_pose = deepcopy(competitor.gate_poses_ground_truth[19])
                curr_pose.position.x_val = (
                    competitor.gate_poses_ground_truth[19].position.x_val
                    + competitor.gate_poses_ground_truth[20].position.x_val
                ) / 2
                competitor.gate_poses_ground_truth.insert(20, curr_pose)

            if self.level_name == "Building99_Hard":
                competitor.gate_poses_ground_truth.insert(
                    3,
                    Pose(
                        Vector3r(
                            (-21.49 - 41.89) / 2.0,
                            (-5.44 - 1.84) / 2,
                            (1.51 + 1.82) / 2,
                        )
                    ),
                )

            if self.level_name == "ZhangJiaJie_Medium":
                num_gates = len(competitor.gate_poses_ground_truth)
                last_gate_position = deepcopy(
                    competitor.gate_poses_ground_truth[-1].position
                )
                second_last_gate_position = deepcopy(
                    competitor.gate_poses_ground_truth[-2].position
                )
                competitor.gate_poses_ground_truth.insert(
                    num_gates - 1,
                    Pose(
                        Vector3r(
                            (last_gate_position.x_val + second_last_gate_position.x_val)
                            / 2.0,
                            (last_gate_position.y_val + second_last_gate_position.y_val)
                            / 2.0,
                            (last_gate_position.z_val + second_last_gate_position.z_val)
                            / 2.0,
                        )
                    ),
                )

            self.client.call("simStartRace", tier)
            competitor.run_in_thread()

    def __internalRandomGoalZone(self, gate_name):
        gate_pose = self.__internalGetObjectPose(gate_name)
        x_rand = (2 * np.random.random() - 1) * 0.50
        z_rand = (2 * np.random.random() - 1) * 0.50
        nominal_dims = self.simGetNominalGateInnerDimensions()
        gate_scale = self.simGetObjectScaleInternal(gate_name)
        inner_dims = Vector3r(
            gate_scale.x_val * nominal_dims.x_val,
            0,
            gate_scale.z_val * nominal_dims.z_val,
        )
        useful_zone = [inner_dims.x_val / 2, inner_dims.z_val / 2]
        offset = Quaternionr(x_rand * useful_zone[0], 0, z_rand * useful_zone[1], 0)
        rotated_position = offset.rotate(gate_pose.orientation)
        return Pose(
            gate_pose.position
            + Vector3r(
                rotated_position.x_val, rotated_position.y_val, rotated_position.z_val
            )
        )

    def __internalGetObjectPose(self, object_name):
        pose_msgpack = self.client.call("simGetObjectPose", object_name, False)
        pose = Pose.from_msgpack(pose_msgpack)
        counter = 0
        while (
            math.isnan(pose.position.x_val)
            or math.isnan(pose.position.y_val)
            or math.isnan(pose.position.z_val)
        ) and (counter < self.MAX_NUMBER_OF_GETOBJECTPOSE_TRIALS):
            print(f"DEBUG:  position is nan, retrying...")
            counter += 1
            pose_msgpack = self.client.call("simGetObjectPose", object_name, False)
            pose = Pose.from_msgpack(pose_msgpack)
        assert not math.isnan(
            pose.position.x_val
        ), f"ERROR: pose.position.x_val is still {pose.position.x_val} after {counter} trials"
        assert not math.isnan(
            pose.position.y_val
        ), f"ERROR: pose.position.y_val is still {pose.position.y_val} after {counter} trials"
        assert not math.isnan(
            pose.position.z_val
        ), f"ERROR: pose.position.z_val is still {pose.position.z_val} after {counter} trials"

        return pose

    def simResetRace(self):
        """ Resets a current race: moves players to start positions, timer and penalties reset."""
        self.client.call("simResetRace")

    def simDisableRaceLog(self):
        """ Disables race log """
        self.client.call("simDisableRaceLog")

    def simIsRacerDisqualified(self, vehicle_name):
        """
        Args:
            vehicle_name (str): Name of the multirotor to send this command to 
        
        Returns:
            bool: True if vehicle_name is disqualified. False if not
        """
        return self.client.call("simGetDisqualified", vehicle_name)

    def simGetLastGatePassed(self, vehicle_name):
        """
        
        Args:
            vehicle_name (str): Name of the multirotor to send this command to 
        
        Returns:
            int: index of last gate passed by vehicle_name
        """
        return self.client.call("simGetLastGatePassed", vehicle_name)

    def simSpawnObject(
        self, object_name, asset_name, pose, scale, physics_enabled=False
    ):
        """Spawned selected object in the world
        
        Args:
            object_name (str): Desired name of new object
            asset_name (str): Name of asset(mesh) in the project database
            pose (airsim.Pose): Desired pose of object
            scale (airsim.Vector3r): Desired scale of object
        
        Returns:
            str: Name of spawned object, in case it had to be modified
        """
        return self.client.call(
            "simSpawnObject", object_name, asset_name, pose, scale, physics_enabled
        )

    def simDestroyObject(self, object_name):
        """Removes selected object from the world
        
        Args:
            object_name (str): Name of object to be removed
        
        Returns:
            bool: True if object is queued up for removal
        """
        return self.client.call("simDestroyObject", object_name)

