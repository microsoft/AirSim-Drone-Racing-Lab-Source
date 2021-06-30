#pragma once

#include "CoreMinimal.h"
#include "common/CommonStructs.hpp"
#include "api/WorldSimApiBase.hpp"
#include "SimMode/SimModeBase.h"
#include "Components/StaticMeshComponent.h"
#include "Runtime/Engine/Classes/Engine/StaticMesh.h"
#include "Engine/LevelStreamingDynamic.h"
#include "AirBlueprintLib.h"
#include <map>
#include <string>

class WorldSimApi : public msr::airlib::WorldSimApiBase
{
public:
    typedef msr::airlib::Pose Pose;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::MeshPositionVertexBuffersResponse MeshPositionVertexBuffersResponse;

    WorldSimApi(ASimModeBase* simmode);
    virtual ~WorldSimApi() = default;

    virtual bool loadLevel(const std::string& level_name) override;

    virtual std::string spawnObject(std::string& object_name, const std::string& load_name, const WorldSimApi::Pose& pose, const WorldSimApi::Vector3r& scale, bool physics_enabled) override;
    virtual bool destroyObject(const std::string& object_name) override;

    virtual bool isPaused() const override;
    virtual void reset() override;
    virtual void pause(bool is_paused) override;
    virtual void continueForTime(double seconds) override;
    virtual void continueForFrames(uint32_t frames) override;

    virtual void setTimeOfDay(bool is_enabled, const std::string& start_datetime, bool is_start_datetime_dst,
                              float celestial_clock_speed, float update_interval_secs, bool move_sun);

    virtual void enableWeather(bool enable);
    virtual void setWeatherParameter(WeatherParameter param, float val);

    virtual void setWind(const Vector3r& wind) const override;

    virtual bool setSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex = false) override;
    virtual int getSegmentationObjectID(const std::string& mesh_name) const override;

    virtual bool addVehicle(const std::string& vehicle_name, const std::string& vehicle_type, const Pose& pose, const std::string& pawn_path = "") override;

    virtual void printLogMessage(const std::string& message,
                                 const std::string& message_param = "", unsigned char severity = 0) override;

    virtual std::unique_ptr<std::vector<std::string>> swapTextures(const std::string& tag, int tex_id = 0, int component_id = 0, int material_id = 0) override;
    virtual std::vector<std::string> listSceneObjects(const std::string& name_regex) const override;
    virtual Pose getObjectPose(const std::string& object_name, bool add_noise=true) const override;
    virtual bool setObjectPose(const std::string& object_name, const Pose& pose, bool teleport) override;
    virtual bool runConsoleCommand(const std::string& command) override;
    virtual Vector3r getObjectScale(const std::string& object_name) const override;
    virtual Vector3r getObjectScaleInternal(const std::string& object_name) const override;
    virtual bool setObjectScale(const std::string& object_name, const Vector3r& scale) override;
    virtual bool setTextureFromUrl(std::string& object_name, std::string& url) override;

    //----------- Plotting APIs ----------/
    virtual void simFlushPersistentMarkers() override;
    virtual void simPlotPoints(const std::vector<Vector3r>& points, const std::vector<float>& color_rgba, float size, float duration, bool is_persistent) override;
    virtual void simPlotLineStrip(const std::vector<Vector3r>& points, const std::vector<float>& color_rgba, float thickness, float duration, bool is_persistent) override;
    virtual void simPlotLineList(const std::vector<Vector3r>& points, const std::vector<float>& color_rgba, float thickness, float duration, bool is_persistent) override;
    virtual void simPlotArrows(const std::vector<Vector3r>& points_start, const std::vector<Vector3r>& points_end, const std::vector<float>& color_rgba, float thickness, float arrow_size, float duration, bool is_persistent) override;
    virtual void simPlotStrings(const std::vector<std::string>& strings, const std::vector<Vector3r>& positions, float scale, const std::vector<float>& color_rgba, float duration) override;
    virtual void simPlotTransforms(const std::vector<Pose>& poses, float scale, float thickness, float duration, bool is_persistent) override;
    virtual void simPlotTransformsWithNames(const std::vector<Pose>& poses, const std::vector<std::string>& names, float tf_scale, float tf_thickness, float text_scale, const std::vector<float>& text_color_rgba, float duration) override;
    virtual std::vector<MeshPositionVertexBuffersResponse> getMeshPositionVertexBuffers() const override;

    // Voxel grid/SDF API
    virtual bool createVoxelGrid(const Vector3r& position, const int& x_size, const int& y_size, const int& z_size, const float& res, const std::string& output_file) override;
    virtual bool buildSDF(const Vector3r& position, const double& x_size, const double& y_size, const double& z_size, const float& res) override;
    virtual Vector3r projectToCollisionFree(const Vector3r& position, const double& mindist) override;
    virtual double getSignedDistance(const Vector3r& position) override;
    virtual Vector3r getSDFGradient(const Vector3r& position) override;
    virtual bool isOccupied(const Vector3r& position) override;
    virtual bool saveSDF(const std::string& filepath) override;
    virtual bool loadSDF(const std::string& filepath) override;

	// Race API
	virtual void disableRaceLogging() override;
	virtual void startRace(const int race_tier) override;
	virtual void startBenchmarkRace(const int race_tier) override;
	virtual void resetRace() override;
	virtual bool getDisqualified(const std::string& racer_name) override;
	virtual int getLastGatePassed(const std::string& racer_name) override;

    // Recording APIs
    virtual void startRecording() override;
    virtual void stopRecording() override;
    virtual bool isRecording() const override;

    virtual std::vector<std::string> listVehicles() const override;

    virtual std::string getSettingsString() const override;

private:
    AActor* createNewActor(const FActorSpawnParameters& spawn_params, const FTransform& actor_transform, const Vector3r& scale, UStaticMesh* static_mesh);
    void spawnPlayer();
    Pose generateNoise(float position_magnitude=3.0, float orientation_magnitude=0.01);

private:
    ASimModeBase* simmode_;
    ULevelStreamingDynamic* current_level_;

    VoxelGrid::VoxelGrid<uint8_t> voxel_grid_temp;
    std::vector<bool> voxel_grid_;
    sdf_tools::SignedDistanceField sdf_;

	// Race API
	std::map<FString, Pose> gate_noise_map_;
};
