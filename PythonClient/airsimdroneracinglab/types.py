from __future__ import print_function
import msgpackrpc  # install as admin: pip install msgpack-rpc-python
import numpy as np  # pip install numpy


class MsgpackMixin:

    """
    """

    def __repr__(self):
        """
        
        Returns:
            TYPE: Description
        """
        from pprint import pformat

        return "<" + type(self).__name__ + "> " + pformat(vars(self), indent=4, width=1)

    def to_msgpack(self, *args, **kwargs):
        """
        
        Args:
            *args: Description
            **kwargs: Description
        
        Returns:
            TYPE: Description
        """
        return self.__dict__

    @classmethod
    def from_msgpack(cls, encoded):
        """
        
        Args:
            encoded (TYPE): Description
        
        Returns:
            TYPE: Description
        """
        obj = cls()
        # obj.__dict__ = {k.decode('utf-8'): (from_msgpack(v.__class__, v) if hasattr(v, "__dict__") else v) for k, v in encoded.items()}
        obj.__dict__ = {
            k: (
                v
                if not isinstance(v, dict)
                else getattr(getattr(obj, k).__class__, "from_msgpack")(v)
            )
            for k, v in encoded.items()
        }
        # return cls(**msgpack.unpack(encoded))
        return obj


class ImageType:

    """
    
    Attributes:
        DepthPerspective (int): Description
        DepthPlanner (int): Description
        DepthVis (int): Description
        DisparityNormalized (int): Description
        Infrared (int): Description
        Scene (int): Description
        Segmentation (int): Description
        SurfaceNormals (int): Description
    """

    Scene = 0
    DepthPlanner = 1
    DepthPerspective = 2
    DepthVis = 3
    DisparityNormalized = 4
    Segmentation = 5
    SurfaceNormals = 6
    Infrared = 7


class DrivetrainType:

    """Type of DrivetrainType
    
    Attributes:
        ForwardOnly (int): Fixes yaw along tangent of velocity vector (atan2(vy, vx)) 
        MaxDegreeOfFreedom (int): Description
    """

    MaxDegreeOfFreedom = 0
    ForwardOnly = 1


class LandedState:

    """
    
    Attributes:
        Flying (int): Description
        Landed (int): Description
    """

    Landed = 0
    Flying = 1


class WeatherParameter:
    Rain = 0
    Roadwetness = 1
    Snow = 2
    RoadSnow = 3
    MapleLeaf = 4
    RoadLeaf = 5
    Dust = 6
    Fog = 7
    Enabled = 8


class Vector3r(MsgpackMixin):
    """
    
    Attributes:
        x_val (float): Description
        y_val (float): Description
        z_val (float): Description
    """

    x_val = 0.0
    y_val = 0.0
    z_val = 0.0

    def __init__(self, x_val=0.0, y_val=0.0, z_val=0.0):
        """
        
        Args:
            x_val (float, optional): Description
            y_val (float, optional): Description
            z_val (float, optional): Description
        """
        self.x_val = x_val
        self.y_val = y_val
        self.z_val = z_val

    @staticmethod
    def nanVector3r():
        """
        
        Returns:
            TYPE: Description
        """
        return Vector3r(np.nan, np.nan, np.nan)

    def __add__(self, other):
        """
        
        Args:
            other (TYPE): Description
        
        Returns:
            TYPE: Description
        """
        return Vector3r(
            self.x_val + other.x_val, self.y_val + other.y_val, self.z_val + other.z_val
        )

    def __sub__(self, other):
        """
        
        Args:
            other (TYPE): Description
        
        Returns:
            TYPE: Description
        """
        return Vector3r(
            self.x_val - other.x_val, self.y_val - other.y_val, self.z_val - other.z_val
        )

    def __truediv__(self, other):
        """
        
        Args:
            other (TYPE): Description
        
        Returns:
            TYPE: Description
        
        Raises:
            TypeError: Description
        """
        if (
            type(other)
            in [int, float]
            + np.sctypes["int"]
            + np.sctypes["uint"]
            + np.sctypes["float"]
        ):
            return Vector3r(self.x_val / other, self.y_val / other, self.z_val / other)
        else:
            raise TypeError(
                "unsupported operand type(s) for /: %s and %s"
                % (str(type(self)), str(type(other)))
            )

    def __mul__(self, other):
        """
        
        Args:
            other (TYPE): Description
        
        Returns:
            TYPE: Description
        
        Raises:
            TypeError: Description
        """
        if (
            type(other)
            in [int, float]
            + np.sctypes["int"]
            + np.sctypes["uint"]
            + np.sctypes["float"]
        ):
            return Vector3r(self.x_val * other, self.y_val * other, self.z_val * other)
        else:
            raise TypeError(
                "unsupported operand type(s) for *: %s and %s"
                % (str(type(self)), str(type(other)))
            )

    def dot(self, other):
        """
        
        Args:
            other (TYPE): Description
        
        Returns:
            TYPE: Description
        
        Raises:
            TypeError: Description
        """
        if type(self) == type(other):
            return (
                self.x_val * other.x_val
                + self.y_val * other.y_val
                + self.z_val * other.z_val
            )
        else:
            raise TypeError(
                "unsupported operand type(s) for 'dot': %s and %s"
                % (str(type(self)), str(type(other)))
            )

    def cross(self, other):
        """
        
        Args:
            other (TYPE): Description
        
        Returns:
            TYPE: Description
        
        Raises:
            TypeError: Description
        """
        if type(self) == type(other):
            cross_product = np.cross(self.to_numpy_array(), other.to_numpy_array())
            return Vector3r(cross_product[0], cross_product[1], cross_product[2])
        else:
            raise TypeError(
                "unsupported operand type(s) for 'cross': %s and %s"
                % (str(type(self)), str(type(other)))
            )

    def get_length(self):
        """
        
        Returns:
            TYPE: Description
        """
        return (self.x_val ** 2 + self.y_val ** 2 + self.z_val ** 2) ** 0.5

    def distance_to(self, other):
        """
        
        Args:
            other (TYPE): Description
        
        Returns:
            TYPE: Description
        """
        return (
            (self.x_val - other.x_val) ** 2
            + (self.y_val - other.y_val) ** 2
            + (self.z_val - other.z_val) ** 2
        ) ** 0.5

    def to_Quaternionr(self):
        """
        
        Returns:
            TYPE: Description
        """
        return Quaternionr(self.x_val, self.y_val, self.z_val, 0)

    def to_numpy_array(self):
        """
        
        Returns:
            TYPE: Description
        """
        return np.array([self.x_val, self.y_val, self.z_val], dtype=np.float32)


class Quaternionr(MsgpackMixin):
    """
    
    Attributes:
        w_val (float): Description
        x_val (float): Description
        y_val (float): Description
        z_val (float): Description
    """

    w_val = 0.0
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0

    def __init__(self, x_val=0.0, y_val=0.0, z_val=0.0, w_val=1.0):
        """
        
        Args:
            x_val (float, optional): Description
            y_val (float, optional): Description
            z_val (float, optional): Description
            w_val (float, optional): Description
        """
        self.x_val = x_val
        self.y_val = y_val
        self.z_val = z_val
        self.w_val = w_val

    @staticmethod
    def nanQuaternionr():
        """
        
        Returns:
            TYPE: Description
        """
        return Quaternionr(np.nan, np.nan, np.nan, np.nan)

    def __add__(self, other):
        """
        
        Args:
            other (TYPE): Description
        
        Returns:
            TYPE: Description
        
        Raises:
            TypeError: Description
        """
        if type(self) == type(other):
            return Quaternionr(
                self.x_val + other.x_val,
                self.y_val + other.y_val,
                self.z_val + other.z_val,
                self.w_val + other.w_val,
            )
        else:
            raise TypeError(
                "unsupported operand type(s) for +: %s and %s"
                % (str(type(self)), str(type(other)))
            )

    def __mul__(self, other):
        """
        
        Args:
            other (TYPE): Description
        
        Returns:
            TYPE: Description
        
        Raises:
            TypeError: Description
        """
        if type(self) == type(other):
            t, x, y, z = self.w_val, self.x_val, self.y_val, self.z_val
            a, b, c, d = other.w_val, other.x_val, other.y_val, other.z_val
            return Quaternionr(
                w_val=a * t - b * x - c * y - d * z,
                x_val=b * t + a * x + d * y - c * z,
                y_val=c * t + a * y + b * z - d * x,
                z_val=d * t + z * a + c * x - b * y,
            )
        else:
            raise TypeError(
                "unsupported operand type(s) for *: %s and %s"
                % (str(type(self)), str(type(other)))
            )

    def __truediv__(self, other):
        """
        
        Args:
            other (TYPE): Description
        
        Returns:
            TYPE: Description
        
        Raises:
            TypeError: Description
        """
        if type(other) == type(self):
            return self * other.inverse()
        elif (
            type(other)
            in [int, float]
            + np.sctypes["int"]
            + np.sctypes["uint"]
            + np.sctypes["float"]
        ):
            return Quaternionr(
                self.x_val / other,
                self.y_val / other,
                self.z_val / other,
                self.w_val / other,
            )
        else:
            raise TypeError(
                "unsupported operand type(s) for /: %s and %s"
                % (str(type(self)), str(type(other)))
            )

    def dot(self, other):
        """
        
        Args:
            other (TYPE): Description
        
        Returns:
            TYPE: Description
        
        Raises:
            TypeError: Description
        """
        if type(self) == type(other):
            return (
                self.x_val * other.x_val
                + self.y_val * other.y_val
                + self.z_val * other.z_val
                + self.w_val * other.w_val
            )
        else:
            raise TypeError(
                "unsupported operand type(s) for 'dot': %s and %s"
                % (str(type(self)), str(type(other)))
            )

    def cross(self, other):
        """
        
        Args:
            other (TYPE): Description
        
        Returns:
            TYPE: Description
        
        Raises:
            TypeError: Description
        """
        if type(self) == type(other):
            return (self * other - other * self) / 2
        else:
            raise TypeError(
                "unsupported operand type(s) for 'cross': %s and %s"
                % (str(type(self)), str(type(other)))
            )

    def outer_product(self, other):
        """
        
        Args:
            other (TYPE): Description
        
        Returns:
            TYPE: Description
        
        Raises:
            TypeError: Description
        """
        if type(self) == type(other):
            return (self.inverse() * other - other.inverse() * self) / 2
        else:
            raise TypeError(
                "unsupported operand type(s) for 'outer_product': %s and %s"
                % (str(type(self)), str(type(other)))
            )

    def rotate(self, other):
        """
        
        Args:
            other (TYPE): Description
        
        Returns:
            TYPE: Description
        
        Raises:
            TypeError: Description
            ValueError: Description
        """
        if type(self) == type(other):
            if abs(other.get_length() - 1) <= 0.01:
                return other * self * other.inverse()
            else:
                raise ValueError("length of the other Quaternionr must be 1")
        else:
            raise TypeError(
                "unsupported operand type(s) for 'rotate': %s and %s"
                % (str(type(self)), str(type(other)))
            )

    def conjugate(self):
        """
        
        Returns:
            TYPE: Description
        """
        return Quaternionr(-self.x_val, -self.y_val, -self.z_val, self.w_val)

    def star(self):
        """
        
        Returns:
            TYPE: Description
        """
        return self.conjugate()

    def inverse(self):
        """
        
        Returns:
            TYPE: Description
        """
        return self.star() / self.dot(self)

    def sgn(self):
        """
        
        Returns:
            TYPE: Description
        """
        return self / self.get_length()

    def get_length(self):
        """
        
        Returns:
            float: norm of quaternion
        """
        return (
            self.x_val ** 2 + self.y_val ** 2 + self.z_val ** 2 + self.w_val ** 2
        ) ** 0.5

    def to_numpy_array(self):
        """
        
        Returns:
            np.array: Description
        """
        return np.array(
            [self.x_val, self.y_val, self.z_val, self.w_val], dtype=np.float32
        )


class Pose(MsgpackMixin):
    """
    
    Attributes:
        orientation (Quaternionr): Description
        position (Vector3r): Description
    """

    position = Vector3r()
    orientation = Quaternionr()

    def __init__(self, position_val=Vector3r(), orientation_val=Quaternionr()):
        """
        
        Args:
            position_val (TYPE, optional): Description
            orientation_val (TYPE, optional): Description
        """
        self.position = position_val
        self.orientation = orientation_val

    @staticmethod
    def nanPose():
        """
        
        Returns:
            TYPE: Description
        """
        return Pose(Vector3r.nanVector3r(), Quaternionr.nanQuaternionr())


class CollisionInfo(MsgpackMixin):
    """
    
    Attributes:
        has_collided (bool): Description
        impact_point (TYPE): Description
        normal (TYPE): Description
        object_id (int): Description
        object_name (str): Description
        penetration_depth (float): Description
        position (TYPE): Description
        time_stamp (float): Description
    """

    has_collided = False
    normal = Vector3r()
    impact_point = Vector3r()
    position = Vector3r()
    penetration_depth = 0.0
    time_stamp = 0.0
    object_name = ""
    object_id = -1


class GeoPoint(MsgpackMixin):
    """
    
    Attributes:
        altitude (float): Description
        latitude (float): Description
        longitude (float): Description
    """

    latitude = 0.0
    longitude = 0.0
    altitude = 0.0


class YawMode(MsgpackMixin):
    """
    Struct YawMode with two fields, yaw_or_rate and is_rate. 
    If is_rate field is True then yaw_or_rate field is interpreted as angular velocity in **degrees/sec**,
    which means you want vehicle to rotate continuously around its axis at that angular velocity while moving. 

    If is_rate is False then yaw_or_rate is interpreted as angle in **degrees**, 
    which means you want vehicle to rotate to specific angle (i.e. yaw) and keep that angle while moving.

    When yaw_mode.is_rate == true, the drivetrain parameter shouldn't be set to ForwardOnly because there's a contradiction
    as we're asking for the drone to keep front pointing ahead, but also rotate continuously. 
    However if you have yaw_mode.is_rate = false in ForwardOnly mode then you can do some funky stuff. 
    For example, you can have drone do circles and have yaw_or_rate set to 90 so camera is always pointed to center. 
    In MaxDegreeofFreedom also you can get some funky stuff by setting yaw_mode.is_rate = true and say yaw_mode.yaw_or_rate = 20. 
    This will cause drone to go in its path while rotating which may allow to do 360 scanning.

    In most cases, you just don't want yaw to change which you can do by setting yaw rate of 0. 
    The shorthand for this is airsim.YawMode()

    Attributes:
        is_rate (bool): if True, yaw_or_rate is interpreted as angular velocity in **degrees/sec**,
                        if False, yaw_or_rate is interpreted as angles in **degrees**,
        yaw_or_rate (float): value of desired yaw rate, or desired yaw angle. Interpretation depends upon is_rate
    """

    is_rate = True
    yaw_or_rate = 0.0

    def __init__(self, is_rate=True, yaw_or_rate=0.0):
        """
        
        Args:
            is_rate (bool, optional): Description
            yaw_or_rate (float, optional): Description
        """
        self.is_rate = is_rate
        self.yaw_or_rate = yaw_or_rate


class RCData(MsgpackMixin):

    """
    
    Attributes:
        is_initialized (bool): Description
        is_valid (bool): Description
        pitch (TYPE): Description
        roll (TYPE): Description
        switch1 (TYPE): Description
        switch2 (TYPE): Description
        switch3 (TYPE): Description
        switch4 (TYPE): Description
        switch5 (TYPE): Description
        switch6 (TYPE): Description
        switch7 (TYPE): Description
        switch8 (TYPE): Description
        throttle (TYPE): Description
        timestamp (int): Description
        yaw (TYPE): Description
    """

    timestamp = 0
    pitch, roll, throttle, yaw = (0.0,) * 4  # init 4 variable to 0.0
    switch1, switch2, switch3, switch4 = (0,) * 4
    switch5, switch6, switch7, switch8 = (0,) * 4
    is_initialized = False
    is_valid = False

    def __init__(
        self,
        timestamp=0,
        pitch=0.0,
        roll=0.0,
        throttle=0.0,
        yaw=0.0,
        switch1=0,
        switch2=0,
        switch3=0,
        switch4=0,
        switch5=0,
        switch6=0,
        switch7=0,
        switch8=0,
        is_initialized=False,
        is_valid=False,
    ):
        """
        
        Args:
            timestamp (int, optional): Description
            pitch (float, optional): Description
            roll (float, optional): Description
            throttle (float, optional): Description
            yaw (float, optional): Description
            switch1 (int, optional): Description
            switch2 (int, optional): Description
            switch3 (int, optional): Description
            switch4 (int, optional): Description
            switch5 (int, optional): Description
            switch6 (int, optional): Description
            switch7 (int, optional): Description
            switch8 (int, optional): Description
            is_initialized (bool, optional): Description
            is_valid (bool, optional): Description
        """
        self.timestamp = timestamp
        self.pitch = pitch
        self.roll = roll
        self.throttle = throttle
        self.yaw = yaw
        self.switch1 = switch1
        self.switch2 = switch2
        self.switch3 = switch3
        self.switch4 = switch4
        self.switch5 = switch5
        self.switch6 = switch6
        self.switch7 = switch7
        self.switch8 = switch8
        self.is_initialized = is_initialized
        self.is_valid = is_valid


class ImageRequest(MsgpackMixin):
    """
    
    Attributes:
        camera_name (str): Description
        compress (bool): Description
        image_type (TYPE): Description
        pixels_as_float (bool): Description
    """

    camera_name = "0"
    image_type = ImageType.Scene
    pixels_as_float = False
    compress = False

    def __init__(self, camera_name, image_type, pixels_as_float=False, compress=True):
        """
        
        Args:
            camera_name (TYPE): Description
            image_type (TYPE): Description
            pixels_as_float (bool, optional): Description
            compress (bool, optional): Description
        """
        # todo: in future remove str(), it's only for compatibility to pre v1.2
        self.camera_name = str(camera_name)
        self.image_type = image_type
        self.pixels_as_float = pixels_as_float
        self.compress = compress


class ImageResponse(MsgpackMixin):
    """
    
    Attributes:
        camera_orientation (TYPE): Description
        camera_position (TYPE): Description
        compress (bool): Description
        height (int): Description
        image_data_float (float): Description
        image_data_uint8 (TYPE): Description
        image_type (TYPE): Description
        message (str): Description
        pixels_as_float (float): Description
        time_stamp (TYPE): Description
        width (int): Description
    """

    image_data_uint8 = np.uint8(0)
    image_data_float = 0.0
    camera_position = Vector3r()
    camera_orientation = Quaternionr()
    time_stamp = np.uint64(0)
    message = ""
    pixels_as_float = 0.0
    compress = True
    width = 0
    height = 0
    image_type = ImageType.Scene


class CarControls(MsgpackMixin):
    throttle = 0.0
    steering = 0.0
    brake = 0.0
    handbrake = False
    is_manual_gear = False
    manual_gear = 0
    gear_immediate = True

    def __init__(
        self,
        throttle=0,
        steering=0,
        brake=0,
        handbrake=False,
        is_manual_gear=False,
        manual_gear=0,
        gear_immediate=True,
    ):
        self.throttle = throttle
        self.steering = steering
        self.brake = brake
        self.handbrake = handbrake
        self.is_manual_gear = is_manual_gear
        self.manual_gear = manual_gear
        self.gear_immediate = gear_immediate

    def set_throttle(self, throttle_val, forward):
        if forward:
            self.is_manual_gear = False
            self.manual_gear = 0
            self.throttle = abs(throttle_val)
        else:
            self.is_manual_gear = False
            self.manual_gear = -1
            self.throttle = -abs(throttle_val)


class KinematicsState(MsgpackMixin):
    """
    
    Attributes:
        angular_acceleration (Vector3r): Description
        angular_velocity (Vector3r): Description
        linear_acceleration (Vector3r): Description
        linear_velocity (Vector3r): Description
        orientation (Quaternionr): Description
        position (Vector3r): Description
    """

    position = Vector3r()
    orientation = Quaternionr()
    linear_velocity = Vector3r()
    angular_velocity = Vector3r()
    linear_acceleration = Vector3r()
    angular_acceleration = Vector3r()


class EnvironmentState(MsgpackMixin):
    position = Vector3r()
    geo_point = GeoPoint()
    gravity = Vector3r()
    air_pressure = 0.0
    temperature = 0.0
    air_density = 0.0


class CarState(MsgpackMixin):
    speed = 0.0
    gear = 0
    rpm = 0.0
    maxrpm = 0.0
    handbrake = False
    collision = CollisionInfo()
    kinematics_estimated = KinematicsState()
    timestamp = np.uint64(0)


class MultirotorState(MsgpackMixin):
    """
    
    Attributes:
        collision (CollisionInfo): Description
        gps_location (GeoPoint): Description
        kinematics_estimated (KinematicsState): Description
        landed_state (LandedState.Landed): Description
        rc_data (RCData): Description
        timestamp (np.uint64): Description
    """

    collision = CollisionInfo()
    kinematics_estimated = KinematicsState()
    gps_location = GeoPoint()
    timestamp = np.uint64(0)
    landed_state = LandedState.Landed
    rc_data = RCData()


class ProjectionMatrix(MsgpackMixin):
    """
    
    Attributes:
        matrix (list): Description
    """

    matrix = []


class CameraInfo(MsgpackMixin):
    """
    
    Attributes:
        fov (int): Description
        pose (TYPE): Description
        proj_mat (TYPE): Description
    """

    pose = Pose()
    fov = -1
    proj_mat = ProjectionMatrix()


class LidarData(MsgpackMixin):
    point_cloud = 0.0
    time_stamp = np.uint64(0)
    pose = Pose()


class ImuData(MsgpackMixin):
    time_stamp = np.uint64(0)
    orientation = Quaternionr()
    angular_velocity = Vector3r()
    linear_acceleration = Vector3r()


class BarometerData(MsgpackMixin):
    time_stamp = np.uint64(0)
    altitude = Quaternionr()
    pressure = Vector3r()
    qnh = Vector3r()


class MagnetometerData(MsgpackMixin):
    time_stamp = np.uint64(0)
    magnetic_field_body = Vector3r()
    magnetic_field_covariance = 0.0


class GnssFixType(MsgpackMixin):
    GNSS_FIX_NO_FIX = 0
    GNSS_FIX_TIME_ONLY = 1
    GNSS_FIX_2D_FIX = 2
    GNSS_FIX_3D_FIX = 3


class GnssReport(MsgpackMixin):
    geo_point = GeoPoint()
    eph = 0.0
    epv = 0.0
    velocity = Vector3r()
    fix_type = GnssFixType()
    time_utc = np.uint64(0)


class GpsData(MsgpackMixin):
    time_stamp = np.uint64(0)
    gnss = GnssReport()
    is_valid = False


class DistanceSensorData(MsgpackMixin):
    time_stamp = np.uint64(0)
    distance = Quaternionr()
    min_distance = Quaternionr()
    max_distance = Quaternionr()
    relative_pose = Pose()


class PIDGains:
    """
    Struct to store values of PID gains. Used to transmit controller gain values while instantiating
    AngleLevel/AngleRate/Velocity/PositionControllerGains objects.
    
    Attributes:
        kP (float): Proportional gain
        kI (float): Integrator gain
        kD (float): Derivative gain
    """

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def to_list(self):
        return [self.kp, self.ki, self.kd]


class AngleRateControllerGains:
    """
    Struct to contain controller gains used by angle level PID controller
    
    Attributes:
        roll_gains (PIDGains): kP, kI, kD for roll axis
        pitch_gains (PIDGains): kP, kI, kD for pitch axis
        yaw_gains (PIDGains): kP, kI, kD for yaw axis
    """

    def __init__(
        self,
        roll_gains=PIDGains(0.25, 0, 0),
        pitch_gains=PIDGains(0.25, 0, 0),
        yaw_gains=PIDGains(0.25, 0, 0),
    ):
        self.roll_gains = roll_gains
        self.pitch_gains = pitch_gains
        self.yaw_gains = yaw_gains

    def to_lists(self):
        return (
            [self.roll_gains.kp, self.pitch_gains.kp, self.yaw_gains.kp],
            [self.roll_gains.ki, self.pitch_gains.ki, self.yaw_gains.ki],
            [self.roll_gains.kd, self.pitch_gains.kd, self.yaw_gains.kd],
        )


class AngleLevelControllerGains:
    """
    Struct to contain controller gains used by angle rate PID controller
    
    Attributes:
        roll_gains (PIDGains): kP, kI, kD for roll axis
        pitch_gains (PIDGains): kP, kI, kD for pitch axis
        yaw_gains (PIDGains): kP, kI, kD for yaw axis
    """

    def __init__(
        self,
        roll_gains=PIDGains(2.5, 0, 0),
        pitch_gains=PIDGains(2.5, 0, 0),
        yaw_gains=PIDGains(2.5, 0, 0),
    ):
        self.roll_gains = roll_gains
        self.pitch_gains = pitch_gains
        self.yaw_gains = yaw_gains

    def to_lists(self):
        return (
            [self.roll_gains.kp, self.pitch_gains.kp, self.yaw_gains.kp],
            [self.roll_gains.ki, self.pitch_gains.ki, self.yaw_gains.ki],
            [self.roll_gains.kd, self.pitch_gains.kd, self.yaw_gains.kd],
        )


class VelocityControllerGains:
    """
    Struct to contain controller gains used by velocity PID controller
    
    Attributes:
        x_gains (PIDGains): kP, kI, kD for X axis
        y_gains (PIDGains): kP, kI, kD for Y axis
        z_gains (PIDGains): kP, kI, kD for Z axis
    """

    def __init__(
        self,
        x_gains=PIDGains(0.2, 0, 0),
        y_gains=PIDGains(0.2, 0, 0),
        z_gains=PIDGains(2.0, 2.0, 0),
    ):
        self.x_gains = x_gains
        self.y_gains = y_gains
        self.z_gains = z_gains

    def to_lists(self):
        return (
            [self.x_gains.kp, self.y_gains.kp, self.z_gains.kp],
            [self.x_gains.ki, self.y_gains.ki, self.z_gains.ki],
            [self.x_gains.kd, self.y_gains.kd, self.z_gains.kd],
        )


class PositionControllerGains:
    """
    Struct to contain controller gains used by position PID controller
    
    Attributes:
        x_gains (PIDGains): kP, kI, kD for X axis
        y_gains (PIDGains): kP, kI, kD for Y axis
        z_gains (PIDGains): kP, kI, kD for Z axis
    """

    def __init__(
        self,
        x_gains=PIDGains(0.25, 0, 0),
        y_gains=PIDGains(0.25, 0, 0),
        z_gains=PIDGains(0.25, 0, 0),
    ):
        self.x_gains = x_gains
        self.y_gains = y_gains
        self.z_gains = z_gains

    def to_lists(self):
        return (
            [self.x_gains.kp, self.y_gains.kp, self.z_gains.kp],
            [self.x_gains.ki, self.y_gains.ki, self.z_gains.ki],
            [self.x_gains.kd, self.y_gains.kd, self.z_gains.kd],
        )


class TrajectoryTrackerGains:
    """
    Struct to contain trajectory tracker gains used by the pure pursuit controller for moveBySpline and moveBySplineVelConstraints
    
    Attributes:
        kp_cross_track (float): P gain for position error measured perpendicular to path tangent, or in the "cross track" direction
        kd_cross_track (float): D gain for position error measured perpendicular to path tangent, or in the "cross track" direction
        kp_vel_cross_track (float): P gain for velocity error measured perpendicular to path tangent, or in the "cross track" direction
        kd_vel_cross_track (float): D gain for velocity error measured perpendicular to path tangent, or in the "cross track" direction
        kp_along_track (float): P gain for position error measured along path tangent
        kd_along_track (float): D gain for position error measured along path tangent
        kp_vel_along_track (float): P gain for velocity error measured along path tangent 
        kd_vel_along_track (float): D gain for velocity error measured along path tangent
        kp_z_track (float): P gain for position error measured along world Z axis
        kd_z_track (float): D gain for position error measured along world Z axis
        kp_vel_z (float): P gain for velocity error measured along world Z axis
        kd_vel_z (float): D gain for velocity error measured along world Z axis
        kp_yaw (float): P gain for yaw error
        kd_yaw (float): D gain for yaw error
    """

    def __init__(
        self,
        kp_cross_track=7.5,
        kd_cross_track=0.0,
        kp_vel_cross_track=5.0,
        kd_vel_cross_track=0.0,
        kp_along_track=0.4,
        kd_along_track=0.0,
        kp_vel_along_track=0.04,
        kd_vel_along_track=0.0,
        kp_z_track=2.0,
        kd_z_track=0.0,
        kp_vel_z=0.4,
        kd_vel_z=0.0,
        kp_yaw=3.0,
        kd_yaw=0.1,
    ):
        self.kp_cross_track = kp_cross_track
        self.kd_cross_track = kd_cross_track
        self.kp_vel_cross_track = kp_vel_cross_track
        self.kd_vel_cross_track = kd_vel_cross_track
        self.kp_along_track = kp_along_track
        self.kd_along_track = kd_along_track
        self.kp_vel_along_track = kp_vel_along_track
        self.kd_vel_along_track = kd_vel_along_track
        self.kp_z_track = kp_z_track
        self.kd_z_track = kd_z_track
        self.kp_vel_z = kp_vel_z
        self.kd_vel_z = kd_vel_z
        self.kp_yaw = kp_yaw
        self.kd_yaw = kd_yaw

    def to_list(self):
        """
        Call this before sending to setTrajectoryTrackerGains
        
        Returns:
            list[float]: Description
        """
        return [
            self.kp_cross_track,
            self.kd_cross_track,
            self.kp_vel_cross_track,
            self.kd_vel_cross_track,
            self.kp_along_track,
            self.kd_along_track,
            self.kp_vel_along_track,
            self.kd_vel_along_track,
            self.kp_z_track,
            self.kd_z_track,
            self.kp_vel_z,
            self.kd_vel_z,
            self.kp_yaw,
            self.kd_yaw,
        ]


class MeshPositionVertexBuffersResponse(MsgpackMixin):
    position = Vector3r()
    orientation = Quaternionr()
    vertices = 0.0
    indices = 0.0
    name = ""

