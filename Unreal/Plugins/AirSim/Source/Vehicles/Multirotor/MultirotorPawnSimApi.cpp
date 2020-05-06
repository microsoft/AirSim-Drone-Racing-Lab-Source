#include "MultirotorPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "vehicles/multirotor/MultiRotorParamsFactory.hpp"
#include "UnrealSensors/UnrealSensorFactory.h"
#include <exception>

using namespace msr::airlib;

MultirotorPawnSimApi::MultirotorPawnSimApi(const Params& params)
    : PawnSimApi(params), pawn_events_(static_cast<MultirotorPawnEvents*>(params.pawn_events))
{
}

MultirotorPawnSimApi::~MultirotorPawnSimApi()
{
    if (state_log_status_)
        stopStateLogging();
}

void MultirotorPawnSimApi::initialize()
{
    PawnSimApi::initialize();

    //create vehicle API
    std::shared_ptr<UnrealSensorFactory> sensor_factory = std::make_shared<UnrealSensorFactory>(getPawn(), &getNedTransform());
    vehicle_params_ = MultiRotorParamsFactory::createConfig(getVehicleSetting(), sensor_factory);
    vehicle_api_ = vehicle_params_->createMultirotorApi();

    // moveOnSpline viz
    vehicle_api_->viz_traj_ = false; // this is a hack? ideally, should be done with vehicle_params_ --> factory createMultirotorApi() ? 
    vehicle_api_->tf_to_plot_ = false; 
    vehicle_api_->viz_poses_vec_.clear(); 

    //setup physics vehicle
    multirotor_physics_body_ = std::unique_ptr<MultiRotor>(new MultiRotorPhysicsBody(vehicle_params_.get(), vehicle_api_.get(), getKinematics(), getEnvironment()));
    rotor_count_ = multirotor_physics_body_->wrenchVertexCount();
    rotor_actuator_info_.assign(rotor_count_, RotorActuatorInfo());

    vehicle_api_->setSimulatedGroundTruth(getGroundTruthKinematics(), getGroundTruthEnvironment());

    //initialize private vars
    last_phys_pose_ = Pose::nanPose();
    pending_pose_status_ = PendingPoseStatus::NonePending;
    reset_pending_ = false;
    did_reset_ = false;
    rotor_states_.rotors.assign(rotor_count_, RotorParameters());

    //reset roll & pitch of vehicle as multirotors required to be on plain surface at start
    Pose pose = getPose();
    float pitch, roll, yaw;
    VectorMath::toEulerianAngle(pose.orientation, pitch, roll, yaw);
    pose.orientation = VectorMath::toQuaternion(0, 0, yaw);
    setPose(pose, false);

    log_folderpath_ = common_utils::FileSystem::getLogFolderPath(true);    
    state_logger_ = std::unique_ptr<FileLogger>(new FileLogger());
}

void MultirotorPawnSimApi::pawnTick(float dt)
{
    unused(dt);
    //calls to update* are handled by physics engine and in SimModeWorldBase
}

void MultirotorPawnSimApi::updateRenderedState(float dt)
{
    //Utils::log("------Render tick-------");

    //if reset is pending then do it first, no need to do other things until next tick
    if (reset_pending_) {
        reset_task_();
        did_reset_ = true;
        return;
    }

    //move collision info from rendering engine to vehicle
    const CollisionInfo& collision_info = getCollisionInfo();
    multirotor_physics_body_->setCollisionInfo(collision_info);

    if (pending_pose_status_ == PendingPoseStatus::RenderStatePending) {
        multirotor_physics_body_->setPose(pending_phys_pose_);
        pending_pose_status_ = PendingPoseStatus::RenderPending;
    }
    // Neurips reset. todo: could be made?
    else if ((last_phys_pose_ - getPose()).position.norm() > 0.1) {
		getVehicleApi()->clearTrajectory();
		pending_phys_pose_ = getPose();
		multirotor_physics_body_->setPose(pending_phys_pose_);
		pending_pose_status_ = PendingPoseStatus::RenderPending;
		multirotor_physics_body_->reset_everything_minus_position();
	}   

    last_phys_pose_ = multirotor_physics_body_->getPose();

    collision_response = multirotor_physics_body_->getCollisionResponseInfo();

    //update rotor poses
    for (unsigned int i = 0; i < rotor_count_; ++i) {
        const auto& rotor_output = multirotor_physics_body_->getRotorOutput(i);
        // update private rotor variable
        rotor_states_.rotors[i].update(rotor_output.thrust, rotor_output.torque_scaler, rotor_output.speed);
        RotorActuatorInfo* info = &rotor_actuator_info_[i];
        info->rotor_speed = rotor_output.speed;
        info->rotor_direction = static_cast<int>(rotor_output.turning_direction);
        info->rotor_thrust = rotor_output.thrust;
        info->rotor_control_filtered = rotor_output.control_signal_filtered;
    }

    vehicle_api_->getStatusMessages(vehicle_api_messages_);

    if (getRemoteControlID() >= 0)
        vehicle_api_->setRCData(getRCData());
    rotor_states_.timestamp = clock()->nowNanos();
    vehicle_api_->setRotorStates(rotor_states_);
}

void MultirotorPawnSimApi::updateRendering(float dt)
{
    //if we did reset then don't worry about synchronizing states for this tick
    if (reset_pending_) {
        // Continue to wait for reset
        if (!did_reset_) {
            return;
        }
        else {
            reset_pending_ = false;
            did_reset_ = false;
            return;
        }
    }

    if (!VectorMath::hasNan(last_phys_pose_)) {
        if (pending_pose_status_ == PendingPoseStatus::RenderPending) {
            PawnSimApi::setPose(last_phys_pose_, pending_pose_collisions_);
            pending_pose_status_ = PendingPoseStatus::NonePending;
        }
        else
            PawnSimApi::setPose(last_phys_pose_, false);
    }

    //UAirBlueprintLib::LogMessage(TEXT("Collision (raw) Count:"), FString::FromInt(collision_response.collision_count_raw), LogDebugLevel::Unimportant);
    UAirBlueprintLib::LogMessage(TEXT("Collision Count:"),
                                 FString::FromInt(collision_response.collision_count_non_resting),
                                 LogDebugLevel::Informational);

    for (auto i = 0; i < vehicle_api_messages_.size(); ++i) {
        UAirBlueprintLib::LogMessage(FString(vehicle_api_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
    }

    try {
        vehicle_api_->sendTelemetry(dt);
    }
    catch (std::exception& e) {
        UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
    }

    pawn_events_->getActuatorSignal().emit(rotor_actuator_info_);
}

void MultirotorPawnSimApi::setPose(const Pose& pose, bool ignore_collision)
{
    multirotor_physics_body_->lock();
    multirotor_physics_body_->setPose(pose);
    multirotor_physics_body_->setGrounded(false);
    multirotor_physics_body_->unlock();
    pending_pose_collisions_ = ignore_collision;
    pending_pose_status_ = PendingPoseStatus::RenderPending;
}

//*** Start: UpdatableState implementation ***//
void MultirotorPawnSimApi::resetImplementation()
{
    PawnSimApi::resetImplementation();

    vehicle_api_->reset();
    multirotor_physics_body_->reset();
    vehicle_api_messages_.clear();
}

//this is high frequency physics tick, flier gets ticked at rendering frame rate
void MultirotorPawnSimApi::update()
{
    //environment update for current position
    PawnSimApi::update();

    //update forces on vertices
    multirotor_physics_body_->update();

    if (state_log_status_)
        writeStatetoFile();

    //update to controller must be done after kinematics have been updated by physics engine
}

void MultirotorPawnSimApi::reportState(StateReporter& reporter)
{
    PawnSimApi::reportState(reporter);

    multirotor_physics_body_->reportState(reporter);
}

std::string MultirotorPawnSimApi::createStateHeaderLine()
{
    std::stringstream ss;

    ss << "Timestamp,pos_x,pos_y,pos_z,rot_w,rot_x,rot_y,rot_z,lin_vel_x,lin_vel_y,lin_vel_z,"
        << "ang_vel_x,ang_vel_y,ang_vel_z,lin_acc_x,lin_acc_y,lin_acc_z,ang_acc_x,ang_acc_y,ang_acc_z,"
        << "force_x,force_y,force_z,torque_x,torque_y,torque_z,"
        << "rotor1_dir,rotor1_input,rotor1_input_filt,rotor1_speed,rotor1_thrust,rotor1_torque,"
        << "rotor2_dir,rotor2_input,rotor2_input_filt,rotor2_speed,rotor2_thrust,rotor2_torque,"
        << "rotor3_dir,rotor3_input,rotor3_input_filt,rotor3_speed,rotor3_thrust,rotor3_torque,"
        << "rotor4_dir,rotor4_input,rotor4_input_filt,rotor4_speed,rotor4_thrust,rotor4_torque";

    return ss.str();
}

void MultirotorPawnSimApi::writeStatetoFile()
{
    std::stringstream ss;
    uint64_t timestamp_millis = static_cast<uint64_t>(msr::airlib::ClockFactory::get()->nowNanos() / 1.0E6);

    const msr::airlib::Kinematics::State* kinematics = getGroundTruthKinematics();

    ss << std::fixed << std::setprecision(8) << timestamp_millis << ","
        << kinematics->pose.position.x() << "," << kinematics->pose.position.y() << "," << kinematics->pose.position.z() << ","
        << kinematics->pose.orientation.w() << "," << kinematics->pose.orientation.x() << "," << kinematics->pose.orientation.y() << "," << kinematics->pose.orientation.z() << ","
        << kinematics->twist.linear.x() << "," << kinematics->twist.linear.y() << "," << kinematics->twist.linear.z() << ","
        << kinematics->twist.angular.x() << "," << kinematics->twist.angular.y() << "," << kinematics->twist.angular.z() << ","
        << kinematics->accelerations.linear.x() << "," << kinematics->accelerations.linear.y() << "," << kinematics->accelerations.linear.z() << ","
        << kinematics->accelerations.angular.x() << "," << kinematics->accelerations.angular.y() << "," << kinematics->accelerations.angular.z() << ",";

    const auto& multirotor_wrench = multirotor_physics_body_->getWrench();

    ss << std::fixed << std::setprecision(8) << multirotor_wrench.force.x() << "," << multirotor_wrench.force.y() << "," << multirotor_wrench.force.z() << ","
        << multirotor_wrench.torque.x() << "," << multirotor_wrench.torque.y() << "," << multirotor_wrench.torque.z() << ",";

    for (unsigned int i = 0; i < rotor_count_; ++i) {
        const auto& rotor_output = multirotor_physics_body_->getRotorOutput(i);

        ss << static_cast<int>(rotor_output.turning_direction) << ",";
        ss << std::fixed << std::setprecision(8) << rotor_output.control_signal_input  << "," << rotor_output.control_signal_filtered << "," 
            << rotor_output.speed << "," << rotor_output.thrust << "," << rotor_output.torque_scaler;

        if (i < rotor_count_ - 1)
            ss << ",";
    }

    state_logger_->writeString(ss.str() + "\n");
}

void MultirotorPawnSimApi::setStateLogStatus(bool is_enabled)
{
    if (is_enabled != getStateLogStatus()) {
        if (is_enabled)
            startStateLogging();
        else
            stopStateLogging();
    }
}

bool MultirotorPawnSimApi::getStateLogStatus()
{
    return state_log_status_;
}

void MultirotorPawnSimApi::startStateLogging()
{
    std::string log_filepath = common_utils::FileSystem::getLogFileNamePath(log_folderpath_, "multirotor_state_log", "_", ".txt", true);
        
    state_logger_->openFile(log_filepath);
    state_logger_->writeString(createStateHeaderLine() + "\n");

    state_log_status_ = true;

    UAirBlueprintLib::LogMessage(TEXT("High frequency state logging is ON"), TEXT(""), LogDebugLevel::Informational);
}

void MultirotorPawnSimApi::stopStateLogging()
{
    state_logger_->closeFile();
    state_log_status_ = false;
    UAirBlueprintLib::LogMessage(TEXT("High frequency state log saved."), TEXT(""), LogDebugLevel::Success);
}

MultirotorPawnSimApi::UpdatableObject* MultirotorPawnSimApi::getPhysicsBody()
{
    return multirotor_physics_body_->getPhysicsBody();
}
//*** End: UpdatableState implementation ***//
