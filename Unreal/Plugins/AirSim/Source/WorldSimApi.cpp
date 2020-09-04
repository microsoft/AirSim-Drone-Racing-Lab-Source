#include "WorldSimApi.h"
#include "common/common_utils/Utils.hpp"
#include "AirBlueprintLib.h"
#include "TextureShuffleActor.h"
#include "common/common_utils/Utils.hpp"
#include "Weather/WeatherLib.h"
#include "DrawDebugHelpers.h"
#include "Runtime/Engine/Classes/Engine/Engine.h"
#include <cstdlib>
#include <ctime>

WorldSimApi::WorldSimApi(ASimModeBase* simmode)
    : simmode_(simmode) {}

bool WorldSimApi::loadLevel(const std::string& level_name)
{
    bool success;
    using namespace std::chrono_literals;

    // Add loading screen to viewport
    simmode_->toggleLoadingScreen(true);
    std::this_thread::sleep_for(0.1s);
    UAirBlueprintLib::RunCommandOnGameThread([this, level_name]() {
        
        this->current_level_ = UAirBlueprintLib::loadLevel(this->simmode_->GetWorld(), FString(level_name.c_str()));
    }, true);

    if (this->current_level_)
    {
        success = true;
        std::this_thread::sleep_for(1s);
        spawnPlayer();
    }
    else
        success = false;

    //Remove Loading screen from viewport
    UAirBlueprintLib::RunCommandOnGameThread([this, level_name]() {
        this->simmode_->OnLevelLoaded.Broadcast();
    }, true);
    this->simmode_->toggleLoadingScreen(false);

    return success;
}

void WorldSimApi::spawnPlayer()
{
    using namespace std::chrono_literals;
    UE_LOG(LogTemp, Log, TEXT("spawning player"));
    bool success{ false };

    UAirBlueprintLib::RunCommandOnGameThread([&]() {
        success = UAirBlueprintLib::spawnPlayer(this->simmode_->GetWorld());
    }, true);

    if (!success)
    {
        UE_LOG(LogTemp, Error, TEXT("Could not find valid PlayerStart Position"));
    }
    else
    {
        std::this_thread::sleep_for(1s);
        simmode_->reset();
    }
}

bool WorldSimApi::destroyObject(const std::string& object_name)
{
    bool result{ false };
    UAirBlueprintLib::RunCommandOnGameThread([this, &object_name, &result]() {
        AActor* actor = UAirBlueprintLib::FindActor<AActor>(simmode_, FString(object_name.c_str()));
        if (actor)
        {
            actor->Destroy();
            result = actor->IsPendingKill();
        }
        if (result)
            simmode_->scene_object_map.Remove(FString(object_name.c_str()));

        GEngine->ForceGarbageCollection(true);
    }, true);
    return result;
}

std::string WorldSimApi::spawnObject(std::string& object_name, const std::string& load_object, const WorldSimApi::Pose& pose, const WorldSimApi::Vector3r& scale, bool physics_enabled)
{
    // Create struct for Location and Rotation of actor in Unreal
    FTransform actor_transform = simmode_->getGlobalNedTransform().fromGlobalNed(pose);

    bool found_object = false, spawned_object = false;
    UAirBlueprintLib::RunCommandOnGameThread([this, load_object, &object_name, &actor_transform, &found_object, &spawned_object, &scale, &physics_enabled]() {
            FString asset_name = FString(load_object.c_str());
            FAssetData *LoadAsset = simmode_->asset_map.Find(asset_name);
            
            if (LoadAsset)
            {
                found_object  = true;
                UStaticMesh* LoadObject = dynamic_cast<UStaticMesh*>(LoadAsset->GetAsset());
                std::vector<std::string> matching_names = UAirBlueprintLib::ListMatchingActors(simmode_->GetWorld(), ".*"+object_name+".*");
                if (matching_names.size() > 0)
                {
                    size_t greatest_num{ 0 }, result{ 0 };
                    for (auto match : matching_names)
                    {
                        std::string number_extension = match.substr(match.find_last_not_of("0123456789") + 1);
                        if (number_extension != "")
                        {
                            result = std::stoi(number_extension);
                            greatest_num = greatest_num > result ? greatest_num : result;
                        }
                    }
                    object_name += std::to_string(greatest_num + 1);
                }
                FActorSpawnParameters new_actor_spawn_params;
                new_actor_spawn_params.Name = FName(object_name.c_str());
                //new_actor_spawn_params.NameMode = FActorSpawnParameters::ESpawnActorNameMode::Required_ReturnNull;
                AActor* NewActor = this->createNewActor(new_actor_spawn_params, actor_transform, scale, LoadObject);

                if (NewActor)
                {
                    spawned_object = true;
                    simmode_->scene_object_map.Add(FString(object_name.c_str()), NewActor);
                }

                UAirBlueprintLib::setSimulatePhysics(NewActor, physics_enabled);
            }
            else
            {
                found_object = false;
            }
    }, true);

    if (!found_object)
    {
        throw std::invalid_argument(
            "There were no objects with name " + load_object + " found in the Registry");
    }
    if (!spawned_object)
    {
        throw std::invalid_argument(
            "Engine could not spawn " + load_object + " because of a stale reference of same name");
    }
    return object_name;
}

AActor* WorldSimApi::createNewActor(const FActorSpawnParameters& spawn_params, const FTransform& actor_transform, const Vector3r& scale, UStaticMesh* static_mesh)
{
    AActor* NewActor = simmode_->GetWorld()->SpawnActor<AActor>(AActor::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, spawn_params); 

    if (NewActor)
    {
        UStaticMeshComponent* ObjectComponent = NewObject<UStaticMeshComponent>(NewActor);
        ObjectComponent->SetStaticMesh(static_mesh);
        ObjectComponent->SetRelativeLocation(FVector(0, 0, 0));
        ObjectComponent->SetWorldScale3D(FVector(scale[0], scale[1], scale[2]));
        ObjectComponent->SetHiddenInGame(false, true);
        ObjectComponent->RegisterComponent();
        NewActor->SetRootComponent(ObjectComponent);
        NewActor->SetActorLocationAndRotation(actor_transform.GetLocation(), actor_transform.GetRotation(), false, nullptr, ETeleportType::TeleportPhysics);
    }
    return NewActor;
}

bool WorldSimApi::isPaused() const
{
    return simmode_->isPaused();
}

void WorldSimApi::reset()
{
    UAirBlueprintLib::RunCommandOnGameThread([this]() {
        simmode_->reset(); 
        }, true);
}

void WorldSimApi::pause(bool is_paused)
{
    simmode_->pause(is_paused);
}

void WorldSimApi::continueForTime(double seconds)
{
    simmode_->continueForTime(seconds);
}

void WorldSimApi::setTimeOfDay(bool is_enabled, const std::string& start_datetime, bool is_start_datetime_dst,
    float celestial_clock_speed, float update_interval_secs, bool move_sun)
{
    simmode_->setTimeOfDay(is_enabled, start_datetime, is_start_datetime_dst,
        celestial_clock_speed, update_interval_secs, move_sun);
}

bool WorldSimApi::setSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex)
{
    bool success;
    UAirBlueprintLib::RunCommandOnGameThread([mesh_name, object_id, is_name_regex, &success]() {
        success = UAirBlueprintLib::SetMeshStencilID(mesh_name, object_id, is_name_regex);
    }, true);
    return success;
}

int WorldSimApi::getSegmentationObjectID(const std::string& mesh_name) const
{
    int result;
    UAirBlueprintLib::RunCommandOnGameThread([&mesh_name, &result]() {
        result = UAirBlueprintLib::GetMeshStencilID(mesh_name);
    }, true);
    return result;
}

void WorldSimApi::printLogMessage(const std::string& message,
    const std::string& message_param, unsigned char severity)
{
    UAirBlueprintLib::LogMessageString(message, message_param, static_cast<LogDebugLevel>(severity));
}

std::vector<std::string> WorldSimApi::listSceneObjects(const std::string& name_regex) const
{
    std::vector<std::string> result;
    UAirBlueprintLib::RunCommandOnGameThread([this, &name_regex, &result]() {
        result = UAirBlueprintLib::ListMatchingActors(simmode_, name_regex);
    }, true);
    return result;
}

bool WorldSimApi::runConsoleCommand(const std::string& command)
{
    bool succeeded = false;
    UAirBlueprintLib::RunCommandOnGameThread([this, &command, &succeeded]() {
        FString fStringCommand(command.c_str());
        succeeded = UAirBlueprintLib::RunConsoleCommand(simmode_, fStringCommand);
    }, true);
    return succeeded;
}

WorldSimApi::Pose WorldSimApi::getObjectPose(const std::string& object_name) const
{
    Pose result;
    UAirBlueprintLib::RunCommandOnGameThread([this, &object_name, &result]() {
        // AActor* actor = UAirBlueprintLib::FindActor<AActor>(simmode_, FString(object_name.c_str()));
        AActor* actor = simmode_->scene_object_map.FindRef(FString(object_name.c_str()));
        result = actor ? simmode_->getGlobalNedTransform().toGlobalNed(FTransform(actor->GetActorRotation(), actor->GetActorLocation()))
            : Pose::nanPose();
    }, true);

    return result;
}

WorldSimApi::Vector3r WorldSimApi::getObjectScale(const std::string& object_name) const
{
    Vector3r result;
    UAirBlueprintLib::RunCommandOnGameThread([this, &object_name, &result]() {
        // AActor* actor = UAirBlueprintLib::FindActor<AActor>(simmode_, FString(object_name.c_str()));
        AActor* actor = simmode_->scene_object_map.FindRef(FString(object_name.c_str()));
        result = actor ? Vector3r(actor->GetActorScale().X, actor->GetActorScale().Y, actor->GetActorScale().Z)
            : Vector3r::Zero();
    }, true);
    return result;
}

bool WorldSimApi::setObjectPose(const std::string& object_name, const WorldSimApi::Pose& pose, bool teleport)
{
    bool result;
    UAirBlueprintLib::RunCommandOnGameThread([this, &object_name, &pose, teleport, &result]() {
        FTransform actor_transform = simmode_->getGlobalNedTransform().fromGlobalNed(pose);
        // AActor* actor = UAirBlueprintLib::FindActor<AActor>(simmode_, FString(object_name.c_str()));
        AActor* actor = simmode_->scene_object_map.FindRef(FString(object_name.c_str()));
        if (actor) {
            if (teleport) 
                result = actor->SetActorLocationAndRotation(actor_transform.GetLocation(), actor_transform.GetRotation(), false, nullptr, ETeleportType::TeleportPhysics);
            else
                result = actor->SetActorLocationAndRotation(actor_transform.GetLocation(), actor_transform.GetRotation(), true);
        }
        else
            result = false;
    }, true);
    return result;
}

bool WorldSimApi::setObjectScale(const std::string& object_name, const Vector3r& scale)
{
    bool result;
    UAirBlueprintLib::RunCommandOnGameThread([this, &object_name, &scale, &result]() {
        // AActor* actor = UAirBlueprintLib::FindActor<AActor>(simmode_, FString(object_name.c_str()));
        AActor* actor = simmode_->scene_object_map.FindRef(FString(object_name.c_str()));
        if (actor) {
            actor->SetActorScale3D(FVector(scale[0], scale[1], scale[2]));
            result = true;
        }
        else
            result = false;
    }, true);
    return result;
}

void WorldSimApi::enableWeather(bool enable)
{
    UWeatherLib::setWeatherEnabled(simmode_->GetWorld(), enable);
}

void WorldSimApi::setWeatherParameter(WeatherParameter param, float val)
{
    unsigned char param_n = static_cast<unsigned char>(msr::airlib::Utils::toNumeric<WeatherParameter>(param));
    EWeatherParamScalar param_e = msr::airlib::Utils::toEnum<EWeatherParamScalar>(param_n);

    UWeatherLib::setWeatherParamScalar(simmode_->GetWorld(), param_e, val);
}

std::unique_ptr<std::vector<std::string>> WorldSimApi::swapTextures(const std::string& tag, int tex_id, int component_id, int material_id)
{
    auto swappedObjectNames = std::make_unique<std::vector<std::string>>();
    UAirBlueprintLib::RunCommandOnGameThread([this, &tag, tex_id, component_id, material_id, &swappedObjectNames]() {
        //Split the tag string into individual tags.
        TArray<FString> splitTags;
        FString notSplit = FString(tag.c_str());
        FString next = "";
        while (notSplit.Split(",", &next, &notSplit))
        {
            next.TrimStartInline();
            splitTags.Add(next);
        }
        notSplit.TrimStartInline();
        splitTags.Add(notSplit);

        //Texture swap on actors that have all of those tags.
        TArray<AActor*> shuffleables;
        UAirBlueprintLib::FindAllActor<ATextureShuffleActor>(simmode_, shuffleables);
        for (auto *shuffler : shuffleables)
        {
            bool invalidChoice = false;
            for (auto required_tag : splitTags)
            {
                invalidChoice |= !shuffler->ActorHasTag(FName(*required_tag));
                if (invalidChoice)
                    break;
            }
            
            if (invalidChoice)
                continue;
            dynamic_cast<ATextureShuffleActor*>(shuffler)->SwapTexture(tex_id, component_id, material_id);
            swappedObjectNames->push_back(TCHAR_TO_UTF8(*shuffler->GetName()));
        }
    }, true);
    return swappedObjectNames;
}

//----------- Plotting APIs ----------/
void WorldSimApi::simFlushPersistentMarkers()
{
    UAirBlueprintLib::RunCommandOnGameThread([this]() {
        FlushPersistentDebugLines(simmode_->GetWorld());
    }, true);
}

void WorldSimApi::simPlotPoints(const std::vector<Vector3r>& points, const std::vector<float>& color_rgba, float size, float duration, bool is_persistent)
{
    FColor color = FLinearColor{color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3]}.ToFColor(true);

    UAirBlueprintLib::RunCommandOnGameThread([this, &points, &color, size, duration, is_persistent]() {
        for (const auto& point : points) {
            DrawDebugPoint(simmode_->GetWorld(),
                    simmode_->getGlobalNedTransform().fromGlobalNed(point),
                    size, color, is_persistent, duration);
        }
    }, true);
}

// plot line for points 0-1, 1-2, 2-3
void WorldSimApi::simPlotLineStrip(const std::vector<Vector3r>& points, const std::vector<float>& color_rgba, float thickness, float duration, bool is_persistent)
{
    FColor color = FLinearColor{color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3]}.ToFColor(true);

    UAirBlueprintLib::RunCommandOnGameThread([this, &points, &color, thickness, duration, is_persistent]() {
        for (size_t idx = 0; idx != points.size()-1; ++idx) {
            DrawDebugLine(simmode_->GetWorld(), 
                simmode_->getGlobalNedTransform().fromGlobalNed(points[idx]), 
                simmode_->getGlobalNedTransform().fromGlobalNed(points[idx+1]), 
                color, is_persistent, duration, 0, thickness);
        }
    }, true);
}

// plot line for points 0-1, 2-3, 4-5... must be even number of points
void WorldSimApi::simPlotLineList(const std::vector<Vector3r>& points, const std::vector<float>& color_rgba, float thickness, float duration, bool is_persistent)
{
    FColor color = FLinearColor{color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3]}.ToFColor(true);

    UAirBlueprintLib::RunCommandOnGameThread([this, &points, &color, thickness, duration, is_persistent]() {
        for (int idx = 0; idx < points.size()-1; idx += 2) {
            DrawDebugLine(simmode_->GetWorld(),
                simmode_->getGlobalNedTransform().fromGlobalNed(points[idx]),
                simmode_->getGlobalNedTransform().fromGlobalNed(points[idx+1]),
                color, is_persistent, duration, 0, thickness);
        }
    }, true);
}

void WorldSimApi::simPlotArrows(const std::vector<Vector3r>& points_start, const std::vector<Vector3r>& points_end, const std::vector<float>& color_rgba, float thickness, float arrow_size, float duration, bool is_persistent)
{
    // assert points_start.size() == poinst_end.size()
    FColor color = FLinearColor{color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3]}.ToFColor(true);

    UAirBlueprintLib::RunCommandOnGameThread([this, &points_start, &points_end, &color, thickness, arrow_size, duration, is_persistent]() {
        for (int idx = 0; idx < points_start.size(); ++idx) {
            DrawDebugDirectionalArrow(simmode_->GetWorld(),
                simmode_->getGlobalNedTransform().fromGlobalNed(points_start[idx]),
                simmode_->getGlobalNedTransform().fromGlobalNed(points_end[idx]),
                arrow_size, color, is_persistent, duration, 0, thickness);
        }
    }, true);
}

void WorldSimApi::simPlotStrings(const std::vector<std::string>& strings, const std::vector<Vector3r>& positions, float scale, const std::vector<float>& color_rgba, float duration)
{
    // assert positions.size() == strings.size()
    FColor color = FLinearColor{color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3]}.ToFColor(true);

    UAirBlueprintLib::RunCommandOnGameThread([this, &strings, &positions, &color, scale, duration]() {
        for (int idx = 0; idx < positions.size(); ++idx) {
            DrawDebugString(simmode_->GetWorld(),
                simmode_->getGlobalNedTransform().fromGlobalNed(positions[idx]),
                FString(strings[idx].c_str()),
                NULL, color, duration, false, scale);
        }
    }, true);
}

void WorldSimApi::simPlotTransforms(const std::vector<Pose>& poses, float scale, float thickness, float duration, bool is_persistent)
{
    UAirBlueprintLib::RunCommandOnGameThread([this, &poses, scale, thickness, duration, is_persistent]() {
        for (const auto& pose : poses) {
            DrawDebugCoordinateSystem(simmode_->GetWorld(),
                simmode_->getGlobalNedTransform().fromGlobalNed(pose.position),
                simmode_->getGlobalNedTransform().fromNed(pose.orientation).Rotator(),
                scale, is_persistent, duration, 0, thickness);
        }
    }, true);
}

void WorldSimApi::simPlotTransformsWithNames(const std::vector<Pose>& poses, const std::vector<std::string>& names, float tf_scale, float tf_thickness, float text_scale, const std::vector<float>& text_color_rgba, float duration)
{
    // assert poses.size() == names.size()
    FColor color = FLinearColor{text_color_rgba[0], text_color_rgba[1], text_color_rgba[2], text_color_rgba[3]}.ToFColor(true);

    UAirBlueprintLib::RunCommandOnGameThread([this, &poses, &names, &color, tf_scale, tf_thickness, text_scale, duration]() {
        for (int idx = 0; idx < poses.size(); ++idx) {
            DrawDebugCoordinateSystem(simmode_->GetWorld(),
                simmode_->getGlobalNedTransform().fromGlobalNed(poses[idx].position),
                simmode_->getGlobalNedTransform().fromNed(poses[idx].orientation).Rotator(),
                tf_scale, false, duration, 0, tf_thickness);
            DrawDebugString(simmode_->GetWorld(),
                simmode_->getGlobalNedTransform().fromGlobalNed(poses[idx]).GetLocation(),
                FString(names[idx].c_str()), NULL, color, duration, false, text_scale);
        }
    }, true);
}

std::vector<WorldSimApi::MeshPositionVertexBuffersResponse> WorldSimApi::getMeshPositionVertexBuffers() const
{
	std::vector<WorldSimApi::MeshPositionVertexBuffersResponse> responses;
	UAirBlueprintLib::RunCommandOnGameThread([&responses]() {
		responses = UAirBlueprintLib::GetStaticMeshComponents();
	}, true);
	return responses;
}

// Recording APIs
void WorldSimApi::startRecording()
{
    simmode_->startRecording();
}

void WorldSimApi::stopRecording()
{
    simmode_->stopRecording();
}

bool WorldSimApi::isRecording() const
{
    return simmode_->isRecording();
}


void WorldSimApi::setWind(const Vector3r& wind) const
{
    simmode_->setWind(wind);
}
