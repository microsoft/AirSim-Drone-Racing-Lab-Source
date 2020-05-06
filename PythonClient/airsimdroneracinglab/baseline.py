import time
from .utils import *
from .types import *
import numpy as np
import math
import threading

# drone_name should match the name in ~/Document/AirSim/settings.json
class BaselineRacer(object):
    """
    Class for drone_2, which flies through gates using moveOnSpline when simStartRace is called.   
    """

    def __init__(self, client, viz_traj=True, viz_traj_color_rgba=[1.0, 0.0, 0.0, 1.0]):
        self.drone_name = "drone_2"
        self.gate_poses_ground_truth = None
        self.viz_traj = viz_traj
        self.viz_traj_color_rgba = viz_traj_color_rgba
        self.airsim_client = client
        self.MAX_NUMBER_OF_GETOBJECTPOSE_TRIALS = 10  # see https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/issues/38

    # arms drone, enable APIs, set default traj tracker gains
    def initialize_drone(self):
        self.airsim_client.enableApiControl(vehicle_name=self.drone_name)
        self.airsim_client.arm(vehicle_name=self.drone_name)

        # set default values for trajectory tracker gains
        traj_tracker_gains = TrajectoryTrackerGains(
            kp_cross_track=5.0,
            kd_cross_track=0.0,
            kp_vel_cross_track=3.0,
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
        )

        self.airsim_client.setTrajectoryTrackerGains(
            traj_tracker_gains, vehicle_name=self.drone_name
        )
        time.sleep(0.2)

    def takeoffAsync(self):
        self.airsim_client.takeoffAsync().join()

    # like takeoffAsync(), but with moveOnSpline()
    def takeoff_with_moveOnSpline(self, takeoff_height=0.1):
        start_position = self.airsim_client.simGetVehiclePose(
            vehicle_name=self.drone_name
        ).position
        takeoff_waypoint = Vector3r(
            start_position.x_val,
            start_position.y_val,
            start_position.z_val - takeoff_height,
        )

        self.airsim_client.moveOnSplineAsync(
            [takeoff_waypoint],
            vel_max=15.0,
            acc_max=5.0,
            add_position_constraint=True,
            add_velocity_constraint=False,
            add_acceleration_constraint=False,
            viz_traj=self.viz_traj,
            viz_traj_color_rgba=self.viz_traj_color_rgba,
            vehicle_name=self.drone_name,
        )

        # calling reset before .join() is finished results in crash stochastically.

        # to get around, I sleep here.
        # this can lead to a freeze instead of a crash..if reset is called at the right (wrong) time, the unreal window freezes
        time.sleep(4.0)

        # curr_position = self.airsim_client.simGetVehiclePose(vehicle_name=self.drone_name).position
        # while (np.abs(curr_position.z_val - (start_position.z_val-takeoff_height)) > 0.15):
        #     curr_position = self.airsim_client.simGetVehiclePose(vehicle_name=self.drone_name).position
        #     # time.sleep(0.0001)
        #     continue

    # stores gate ground truth poses as a list of airsim.Pose() objects in self.gate_poses_ground_truth
    def get_ground_truth_gate_poses(self):
        gate_names_sorted_bad = sorted(self.airsim_client.simListSceneObjects("Gate.*"))
        # gate_names_sorted_bad is of the form `GateN_GARBAGE`. for example:
        # ['Gate0', 'Gate10_21', 'Gate11_23', 'Gate1_3', 'Gate2_5', 'Gate3_7', 'Gate4_9', 'Gate5_11', 'Gate6_13', 'Gate7_15', 'Gate8_17', 'Gate9_19']
        # we sort them by their ibdex of occurence along the race track(N), and ignore the unreal garbage number after the underscore(GARBAGE)
        gate_indices_bad = [
            int(gate_name.split("_")[0][4:]) for gate_name in gate_names_sorted_bad
        ]
        gate_indices_correct = sorted(
            range(len(gate_indices_bad)), key=lambda k: gate_indices_bad[k]
        )
        gate_names_sorted = [
            gate_names_sorted_bad[gate_idx] for gate_idx in gate_indices_correct
        ]
        self.gate_poses_ground_truth = []
        for gate_name in gate_names_sorted:
            curr_pose = self.airsim_client.simGetObjectPose(gate_name)
            counter = 0
            while (
                math.isnan(curr_pose.position.x_val)
                or math.isnan(curr_pose.position.y_val)
                or math.isnan(curr_pose.position.z_val)
            ) and (counter < self.MAX_NUMBER_OF_GETOBJECTPOSE_TRIALS):
                print(f"DEBUG: {gate_name} position is nan, retrying...")
                counter += 1
                curr_pose = self.airsim_client.simGetObjectPose(gate_name)
            assert not math.isnan(
                curr_pose.position.x_val
            ), f"ERROR: {gate_name} curr_pose.position.x_val is still {curr_pose.position.x_val} after {counter} trials"
            assert not math.isnan(
                curr_pose.position.y_val
            ), f"ERROR: {gate_name} curr_pose.position.y_val is still {curr_pose.position.y_val} after {counter} trials"
            assert not math.isnan(
                curr_pose.position.z_val
            ), f"ERROR: {gate_name} curr_pose.position.z_val is still {curr_pose.position.z_val} after {counter} trials"
            self.gate_poses_ground_truth.append(curr_pose)

    def fly_through_all_gates_at_once_with_moveOnSpline(self):
        if self.level_name in ["Soccer_Field_Easy"]:
            vel_max = 30.0
            acc_max = 15.0

        if self.level_name in ["Soccer_Field_Medium"]:
            vel_max = 30.0
            acc_max = 30.0

        if self.level_name in ["Qualifier_Tier_1"]:
            vel_max = 35.0
            acc_max = 15.0

        if self.level_name in [
            "ZhangJiaJie_Medium",
            "Qualifier_Tier_2",
            "Qualifier_Tier_3",
        ]:
            vel_max = 30.0
            acc_max = 17.5

        if self.level_name == "Building99_Hard":
            vel_max = 10.0
            acc_max = 5.0

        return self.airsim_client.moveOnSplineAsync(
            [gate_pose.position for gate_pose in self.gate_poses_ground_truth],
            vel_max=vel_max,
            acc_max=acc_max,
            add_position_constraint=True,
            add_velocity_constraint=False,
            add_acceleration_constraint=False,
            viz_traj=self.viz_traj,
            viz_traj_color_rgba=self.viz_traj_color_rgba,
            vehicle_name=self.drone_name,
        )

    def takeoff_and_fly_through_all_gates_at_once_with_moveOnSpline(self):
        self.takeoff_with_moveOnSpline(1.0)
        self.fly_through_all_gates_at_once_with_moveOnSpline()

    def run_in_thread(self):
        t1 = threading.Thread(
            target=self.takeoff_and_fly_through_all_gates_at_once_with_moveOnSpline
        )
        t1.start()
