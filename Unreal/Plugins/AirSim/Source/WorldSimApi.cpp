#include "WorldSimApi.h"
#include "common/common_utils/Utils.hpp"
#include "AirBlueprintLib.h"
#include "TextureShuffleActor.h"
#include "common/common_utils/Utils.hpp"
#include "Weather/WeatherLib.h"
#include "DrawDebugHelpers.h"
#include "Runtime/Engine/Classes/Engine/Engine.h"
#include "ImageUtils.h"
#include <cstdlib>
#include <ctime>

WorldSimApi::WorldSimApi(ASimModeBase* simmode)
    : simmode_(simmode)
{
}

bool WorldSimApi::loadLevel(const std::string& level_name)
{
    bool success;
    using namespace std::chrono_literals;

    // Add loading screen to viewport
    simmode_->toggleLoadingScreen(true);
    std::this_thread::sleep_for(0.1s);

    UAirBlueprintLib::RunCommandOnGameThread([this, level_name]() {
        this->current_level_ = UAirBlueprintLib::loadLevel(this->simmode_->GetWorld(), FString(level_name.c_str()));
    },
                                             true);

    if (this->current_level_) {
        success = true;
        std::this_thread::sleep_for(1s);
        spawnPlayer();
    }
    else
        success = false;

    //Remove Loading screen from viewport
    UAirBlueprintLib::RunCommandOnGameThread([this, level_name]() {
        this->simmode_->OnLevelLoaded.Broadcast();
    },
                                             true);
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
    },
                                             true);

    if (!success) {
        UE_LOG(LogTemp, Error, TEXT("Could not find valid PlayerStart Position"));
    }
    else {
        std::this_thread::sleep_for(1s);
        simmode_->reset();
    }
}

bool WorldSimApi::destroyObject(const std::string& object_name)
{
    bool result{ false };
    UAirBlueprintLib::RunCommandOnGameThread([this, &object_name, &result]() {
        AActor* actor = UAirBlueprintLib::FindActor<AActor>(simmode_, FString(object_name.c_str()));
        if (actor) {
            actor->Destroy();
            result = actor->IsPendingKill();
        }
        if (result)
            simmode_->scene_object_map.Remove(FString(object_name.c_str()));

        GEngine->ForceGarbageCollection(true);
    },
                                             true);
    return result;
}

std::string WorldSimApi::spawnObject(std::string& object_name, const std::string& load_object, const WorldSimApi::Pose& pose, const WorldSimApi::Vector3r& scale, bool physics_enabled)
{
    // Create struct for Location and Rotation of actor in Unreal
    FTransform actor_transform = simmode_->getGlobalNedTransform().fromGlobalNed(pose);

    bool found_object = false, spawned_object = false;
    UAirBlueprintLib::RunCommandOnGameThread([this, load_object, &object_name, &actor_transform, &found_object, &spawned_object, &scale, &physics_enabled]() {
        FString asset_name = FString(load_object.c_str());
        FAssetData* LoadAsset = simmode_->asset_map.Find(asset_name);

        if (LoadAsset) {
            found_object = true;
            UStaticMesh* LoadObject = dynamic_cast<UStaticMesh*>(LoadAsset->GetAsset());
            std::vector<std::string> matching_names = UAirBlueprintLib::ListMatchingActors(simmode_->GetWorld(), ".*" + object_name + ".*");
            if (matching_names.size() > 0) {
                size_t greatest_num{ 0 }, result{ 0 };
                for (auto match : matching_names) {
                    std::string number_extension = match.substr(match.find_last_not_of("0123456789") + 1);
                    if (number_extension != "") {
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

            if (NewActor) {
                spawned_object = true;
                simmode_->scene_object_map.Add(FString(object_name.c_str()), NewActor);
            }

            UAirBlueprintLib::setSimulatePhysics(NewActor, physics_enabled);
        }
        else {
            found_object = false;
        }
    },
                                             true);

    if (!found_object) {
        throw std::invalid_argument(
            "There were no objects with name " + load_object + " found in the Registry");
    }
    if (!spawned_object) {
        throw std::invalid_argument(
            "Engine could not spawn " + load_object + " because of a stale reference of same name");
    }
    return object_name;
}

AActor* WorldSimApi::createNewActor(const FActorSpawnParameters& spawn_params, const FTransform& actor_transform, const Vector3r& scale, UStaticMesh* static_mesh)
{
    AActor* NewActor = simmode_->GetWorld()->SpawnActor<AActor>(AActor::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, spawn_params);

    if (NewActor) {
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

bool WorldSimApi::setObjectMaterialFromTexture(const std::string& object_name, const std::string& texture_path)
{
    bool success = false;
    UAirBlueprintLib::RunCommandOnGameThread([this, &object_name, &texture_path, &success]() {
        if (!IsValid(simmode_->domain_rand_material_)) {
            UAirBlueprintLib::LogMessageString("Cannot find material for domain randomization",
                                               "",
                                               LogDebugLevel::Failure);
        }
        else {
            UTexture2D* texture_desired = FImageUtils::ImportFileAsTexture2D(FString(texture_path.c_str()));
            AActor* actor = UAirBlueprintLib::FindActor<AActor>(simmode_, FString(object_name.c_str()));

            if (IsValid(actor)) {
                TArray<UStaticMeshComponent*> components;
                actor->GetComponents<UStaticMeshComponent>(components);
                for (UStaticMeshComponent* staticMeshComponent : components) {
                    UMaterialInstanceDynamic* dynamic_material = UMaterialInstanceDynamic::Create(simmode_->domain_rand_material_, staticMeshComponent);
                    dynamic_material->SetTextureParameterValue("TextureParameter", texture_desired);
                    staticMeshComponent->SetMaterial(0, dynamic_material);
                }
                success = true;
            }
            else {
                UAirBlueprintLib::LogMessageString("Cannot find specified actor for domain randomization",
                                                   "",
                                                   LogDebugLevel::Failure);
            }
        }
    },
                                             true);

    return success;
}

bool WorldSimApi::setObjectMaterial(const std::string& object_name, const std::string& material_name)
{
    bool success = false;
    UAirBlueprintLib::RunCommandOnGameThread([this, &object_name, &material_name, &success]() {
        AActor* actor = UAirBlueprintLib::FindActor<AActor>(simmode_, FString(object_name.c_str()));
        UMaterial* material = static_cast<UMaterial*>(StaticLoadObject(UMaterial::StaticClass(), nullptr, *FString(material_name.c_str())));

        if (!IsValid(material)) {
            UAirBlueprintLib::LogMessageString("Cannot find specified material for domain randomization",
                                               "",
                                               LogDebugLevel::Failure);
        }
        else {
            if (IsValid(actor)) {
                TArray<UStaticMeshComponent*> components;
                actor->GetComponents<UStaticMeshComponent>(components);
                for (UStaticMeshComponent* staticMeshComponent : components) {
                    staticMeshComponent->SetMaterial(0, material);
                }
                success = true;
            }
            else {
                UAirBlueprintLib::LogMessageString("Cannot find specified actor for domain randomization",
                                                   "",
                                                   LogDebugLevel::Failure);
            }
        }
    },
                                             true);

    return success;
}

bool WorldSimApi::createVoxelGrid(const Vector3r& position, const double& x_size, const double& y_size, const double& z_size, const float& res, const std::string& output_file)
{
    bool success = false;
    int ncells_x = x_size / res;
    int ncells_y = y_size / res;
    int ncells_z = z_size / res;

    voxel_grid_.resize(ncells_x * ncells_y * ncells_z);

    float scale_cm = res * 100;
    FCollisionQueryParams params;
    params.bFindInitialOverlaps = true;
    params.bTraceComplex = false;
    params.TraceTag = "";
    auto position_in_UE_frame = simmode_->getGlobalNedTransform().fromGlobalNed(position);
    for (float i = 0; i < ncells_x; i++) {
        for (float k = 0; k < ncells_z; k++) {
            for (float j = 0; j < ncells_y; j++) {
                int idx = i + ncells_x * (k + ncells_z * j);
                FVector vposition = FVector((i - ncells_x / 2) * scale_cm, (j - ncells_y / 2) * scale_cm, (k - ncells_z / 2) * scale_cm) + position_in_UE_frame;
                voxel_grid_[idx] = simmode_->GetWorld()->OverlapBlockingTestByChannel(vposition, FQuat::Identity, ECollisionChannel::ECC_Pawn, FCollisionShape::MakeBox(FVector(scale_cm / 2)), params);
            }
        }
    }

    std::ofstream output(output_file, std::ios::out | std::ios::binary);
    if (!output.good()) {
        UE_LOG(LogTemp, Error, TEXT("Could not open output file to write voxel grid!"));
        return success;
    }

    // Write the binvox file using run-length encoding
    // where each pair of bytes is of the format (run value, run length)
    output << "#binvox 1\n";
    output << "dim " << ncells_x << " " << ncells_z << " " << ncells_y << "\n";
    output << "translate " << -x_size * 0.5 << " " << -y_size * 0.5 << " " << -z_size * 0.5 << "\n";
    output << "scale " << 1.0f / x_size << "\n";
    output << "data\n";
    bool run_value = voxel_grid_[0];
    unsigned int run_length = 0;
    for (size_t i = 0; i < voxel_grid_.size(); ++i) {
        if (voxel_grid_[i] == run_value) {
            // This is a run (repeated bit value)
            run_length++;
            if (run_length == 255) {
                output << static_cast<char>(run_value);
                output << static_cast<char>(run_length);
                run_length = 0;
            }
        }
        else {
            // End of a run
            output << static_cast<char>(run_value);
            output << static_cast<char>(run_length);
            run_value = voxel_grid_[i];
            run_length = 1;
        }
    }
    if (run_length > 0) {
        output << static_cast<char>(run_value);
        output << static_cast<char>(run_length);
    }
    output.close();
    success = true;
    return success;
}

bool WorldSimApi::buildSDF(const Vector3r& position, const double& x_size, const double& y_size, const double& z_size, const float& res)
{
    Eigen::Translation3d origin_translation(-x_size / 2, -y_size / 2, -z_size / 2);
    Eigen::Quaterniond origin_rotation;
    origin_rotation.setIdentity();

    Eigen::Isometry3d origin_transform = origin_translation * origin_rotation;
    std::string frame_;
    bool success = false;

    float scale_cm = res * 100;
    FCollisionQueryParams params;
    params.bFindInitialOverlaps = true;
    params.bTraceComplex = false;
    params.TraceTag = "";

    voxel_grid_temp = VoxelGrid::VoxelGrid<uint8_t>(origin_transform, res, x_size, y_size, z_size, 0);

    std::function<bool(const VoxelGrid::GRID_INDEX&)> is_filled_fn = [&](const VoxelGrid::GRID_INDEX& index) {
        // Convert SDF indices into a real-world location
        const Eigen::Vector4d location = voxel_grid_temp.GridIndexToLocation(index);

        // use UE function here
        Vector3r grid_position = location.head<3>().cast<float>();
        FVector position_local = simmode_->getGlobalNedTransform().fromGlobalNed(grid_position + position);
        bool occupied = simmode_->GetWorld()->OverlapBlockingTestByChannel(position_local, FQuat::Identity, ECollisionChannel::ECC_Pawn, FCollisionShape::MakeBox(FVector(scale_cm / 2)), params);

        if (occupied) {
            // Mark as filled
            return true;
        }
        else {
            // Mark as free space
            return false;
        }
    };

    sdf_ = sdf_generation::ExtractSignedDistanceField(voxel_grid_temp, is_filled_fn, INFINITY, "local", false).first;

    return success;
}

bool WorldSimApi::isOccupied(const Vector3r& position)
{
    std::pair<double, bool> dist = sdf_.EstimateDistance3d(position.cast<double>());
    return (dist.first < 0.0);
}

double WorldSimApi::getSignedDistance(const Vector3r& position)
{
    std::pair<double, bool> dist = sdf_.EstimateDistance3d(position.cast<double>());
    return dist.first;
}

std::vector<double> WorldSimApi::getSignedDistances(const std::vector<Vector3r>& positions)
{
    std::vector<double> distances;

    for (auto& position : positions) {
        std::pair<double, bool> dist = sdf_.EstimateDistance3d(position.cast<double>());
        distances.push_back(dist.first);
    }
    return distances;
}

Vector3r WorldSimApi::getSDFGradient(const Vector3r& position)
{
    std::vector<double> gradient = sdf_.GetAutoDiffGradient3d(position.cast<double>());
    Eigen::Vector3d sdf_gradient;

    if (gradient.size() == 0) {
        sdf_gradient << 0.0, 0.0, 0.0;
    }
    else {
        sdf_gradient = Eigen::Map<Eigen::Vector3d>(gradient.data());
    }

    return sdf_gradient.cast<float>();
}

bool WorldSimApi::checkInVolume(const Vector3r& position, const std::string& volume_actor_name)
{
    FCollisionQueryParams params;
    params.bFindInitialOverlaps = true;
    params.bIgnoreBlocks = true;
    params.bTraceComplex = true;
    params.TraceTag = "";
    auto position_in_UE_frame = simmode_->getGlobalNedTransform().fromGlobalNed(position);
    bool overlap = simmode_->GetWorld()->OverlapBlockingTestByChannel(position_in_UE_frame, FQuat::Identity, ECollisionChannel::ECC_Pawn, FCollisionShape::MakeBox(FVector(100)), params);

    return overlap;
}

Vector3r WorldSimApi::projectToCollisionFree(const Vector3r& position, const double& mindist)
{
    Eigen::Vector3d free_pt = sdf_.ProjectOutOfCollisionToMinimumDistance3d(position.cast<double>(), mindist);
    return free_pt.cast<float>();
}

bool WorldSimApi::saveSDF(const std::string& filepath)
{
    sdf_.SaveToFile(sdf_, filepath, false);
    return false;
}

bool WorldSimApi::loadSDF(const std::string& filepath)
{
    sdf_ = sdf_.LoadFromFile(filepath);
    return false;
}

bool WorldSimApi::isPaused() const
{
    return simmode_->isPaused();
}

void WorldSimApi::reset()
{
    UAirBlueprintLib::RunCommandOnGameThread([this]() {
        simmode_->reset();
    },
                                             true);
}

void WorldSimApi::pause(bool is_paused)
{
    simmode_->pause(is_paused);
}

void WorldSimApi::continueForTime(double seconds)
{
    simmode_->continueForTime(seconds);
}

void WorldSimApi::continueForFrames(uint32_t frames)
{
    simmode_->continueForFrames(frames);
}

void WorldSimApi::setTimeOfDay(bool is_enabled, const std::string& start_datetime, bool is_start_datetime_dst,
                               float celestial_clock_speed, float update_interval_secs, bool move_sun)
{
    simmode_->setTimeOfDay(is_enabled, start_datetime, is_start_datetime_dst, celestial_clock_speed, update_interval_secs, move_sun);
}

bool WorldSimApi::addVehicle(const std::string& vehicle_name, const std::string& vehicle_type, const Pose& pose, const std::string& pawn_path)
{
    bool result;
    UAirBlueprintLib::RunCommandOnGameThread([&]() {
        result = simmode_->createVehicleAtRuntime(vehicle_name, vehicle_type, pose, pawn_path);
    },
                                             true);

    return result;
}

bool WorldSimApi::setSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex)
{
    bool success;
    UAirBlueprintLib::RunCommandOnGameThread([mesh_name, object_id, is_name_regex, &success]() {
        success = UAirBlueprintLib::SetMeshStencilID(mesh_name, object_id, is_name_regex);
    },
                                             true);
    return success;
}

int WorldSimApi::getSegmentationObjectID(const std::string& mesh_name) const
{
    int result;
    UAirBlueprintLib::RunCommandOnGameThread([&mesh_name, &result]() {
        result = UAirBlueprintLib::GetMeshStencilID(mesh_name);
    },
                                             true);
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
    },
                                             true);
    return result;
}

bool WorldSimApi::runConsoleCommand(const std::string& command)
{
    bool succeeded = false;
    UAirBlueprintLib::RunCommandOnGameThread([this, &command, &succeeded]() {
        FString fStringCommand(command.c_str());
        succeeded = UAirBlueprintLib::RunConsoleCommand(simmode_, fStringCommand);
    },
                                             true);
    return succeeded;
}

bool WorldSimApi::setTextureFromUrl(std::string& object_name, std::string& url)
{
    bool found = false;
    UAirBlueprintLib::RunCommandOnGameThread([this, &object_name, &url, &found]() {
        ATextureShuffleActor* actor = UAirBlueprintLib::FindActor<ATextureShuffleActor>(simmode_, FString(object_name.c_str()));
        if (actor != nullptr)
            actor->SetTextureFromUrl(FString(url.c_str()));
    },
                                             true);
    return found;
}

WorldSimApi::Pose WorldSimApi::getObjectPose(const std::string& object_name, bool add_noise) const
{
    Pose result;
    UAirBlueprintLib::RunCommandOnGameThread([this, &object_name, &result]() {
        AActor* actor = UAirBlueprintLib::FindActor<AActor>(simmode_, FString(object_name.c_str()));
        result = actor ? simmode_->getGlobalNedTransform().toGlobalNed(FTransform(actor->GetActorRotation(), actor->GetActorLocation()))
                       : Pose::nanPose();
    },
                                             true);

    // give noisy pose always if level 2 or 3
    std::regex compiledRegex(".*[Gg]ate.*", std::regex::optimize);
    if (add_noise && std::regex_match(object_name, compiledRegex)) {
        auto gate = gate_noise_map_.find(FString(object_name.c_str()));
        if (gate != gate_noise_map_.end()) {
            result.position += gate->second.position;
            result.orientation = msr::airlib::VectorMath::coordOrientationAdd(gate->second.orientation, result.orientation);
        }
    }

    return result;
}

Vector3r WorldSimApi::getObjectScale(const std::string& object_name) const
{
    Vector3r result;
    UAirBlueprintLib::RunCommandOnGameThread([this, &object_name, &result]() {
        // AActor* actor = UAirBlueprintLib::FindActor<AActor>(simmode_, FString(object_name.c_str()));
        AActor* actor = simmode_->scene_object_map.FindRef(FString(object_name.c_str()));
        result = actor ? Vector3r(actor->GetActorScale().X, actor->GetActorScale().Y, actor->GetActorScale().Z)
                       : Vector3r::Zero();
    },
                                             true);
    return result;
}

WorldSimApi::Vector3r WorldSimApi::getObjectScaleInternal(const std::string& object_name) const
{
    Vector3r result;
    UAirBlueprintLib::RunCommandOnGameThread([this, &object_name, &result]() {
        AActor* actor = UAirBlueprintLib::FindActor<AActor>(simmode_, FString(object_name.c_str()));
        result = actor ? Vector3r(actor->GetActorScale().X, actor->GetActorScale().Y, actor->GetActorScale().Z)
                       : Vector3r::Zero();
    },
                                             true);
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
    },
                                             true);
    return result;
}

bool WorldSimApi::setObjectScale(const std::string& object_name, const WorldSimApi::Vector3r& scale)
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
    },
                                             true);
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

void WorldSimApi::setWind(const Vector3r& wind) const
{
    simmode_->setWind(wind);
}

std::unique_ptr<std::vector<std::string>> WorldSimApi::swapTextures(const std::string& tag, int tex_id, int component_id, int material_id)
{
    auto swappedObjectNames = std::make_unique<std::vector<std::string>>();
    UAirBlueprintLib::RunCommandOnGameThread([this, &tag, tex_id, component_id, material_id, &swappedObjectNames]() {
        //Split the tag string into individual tags.
        TArray<FString> splitTags;
        FString notSplit = FString(tag.c_str());
        FString next = "";
        while (notSplit.Split(",", &next, &notSplit)) {
            next.TrimStartInline();
            splitTags.Add(next);
        }
        notSplit.TrimStartInline();
        splitTags.Add(notSplit);

        //Texture swap on actors that have all of those tags.
        TArray<AActor*> shuffleables;
        UAirBlueprintLib::FindAllActor<ATextureShuffleActor>(simmode_, shuffleables);
        for (auto* shuffler : shuffleables) {
            bool invalidChoice = false;
            for (auto required_tag : splitTags) {
                invalidChoice |= !shuffler->ActorHasTag(FName(*required_tag));
                if (invalidChoice)
                    break;
            }

            if (invalidChoice)
                continue;
            dynamic_cast<ATextureShuffleActor*>(shuffler)->SwapTexture(tex_id, component_id, material_id);
            swappedObjectNames->push_back(TCHAR_TO_UTF8(*shuffler->GetName()));
        }
    },
                                             true);
    return swappedObjectNames;
}

//----------- Plotting APIs ----------/
void WorldSimApi::simFlushPersistentMarkers()
{
    UAirBlueprintLib::RunCommandOnGameThread([this]() {
        FlushPersistentDebugLines(simmode_->GetWorld());
    },
                                             true);
}

void WorldSimApi::simPlotPoints(const std::vector<Vector3r>& points, const std::vector<float>& color_rgba, float size, float duration, bool is_persistent)
{
    FColor color = FLinearColor{ color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3] }.ToFColor(true);

    UAirBlueprintLib::RunCommandOnGameThread([this, &points, &color, size, duration, is_persistent]() {
        for (const auto& point : points) {
            DrawDebugPoint(simmode_->GetWorld(),
                           simmode_->getGlobalNedTransform().fromGlobalNed(point),
                           size,
                           color,
                           is_persistent,
                           duration);
        }
    },
                                             true);
}

// plot line for points 0-1, 1-2, 2-3
void WorldSimApi::simPlotLineStrip(const std::vector<Vector3r>& points, const std::vector<float>& color_rgba, float thickness, float duration, bool is_persistent)
{
    FColor color = FLinearColor{ color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3] }.ToFColor(true);

    UAirBlueprintLib::RunCommandOnGameThread([this, &points, &color, thickness, duration, is_persistent]() {
        for (size_t idx = 0; idx != points.size() - 1; ++idx) {
            DrawDebugLine(simmode_->GetWorld(),
                          simmode_->getGlobalNedTransform().fromGlobalNed(points[idx]),
                          simmode_->getGlobalNedTransform().fromGlobalNed(points[idx + 1]),
                          color,
                          is_persistent,
                          duration,
                          0,
                          thickness);
        }
    },
                                             true);
}

// plot line for points 0-1, 2-3, 4-5... must be even number of points
void WorldSimApi::simPlotLineList(const std::vector<Vector3r>& points, const std::vector<float>& color_rgba, float thickness, float duration, bool is_persistent)
{
    FColor color = FLinearColor{ color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3] }.ToFColor(true);

    UAirBlueprintLib::RunCommandOnGameThread([this, &points, &color, thickness, duration, is_persistent]() {
        for (int idx = 0; idx < points.size() - 1; idx += 2) {
            DrawDebugLine(simmode_->GetWorld(),
                          simmode_->getGlobalNedTransform().fromGlobalNed(points[idx]),
                          simmode_->getGlobalNedTransform().fromGlobalNed(points[idx + 1]),
                          color,
                          is_persistent,
                          duration,
                          0,
                          thickness);
        }
    },
                                             true);
}

void WorldSimApi::simPlotArrows(const std::vector<Vector3r>& points_start, const std::vector<Vector3r>& points_end, const std::vector<float>& color_rgba, float thickness, float arrow_size, float duration, bool is_persistent)
{
    // assert points_start.size() == poinst_end.size()
    FColor color = FLinearColor{ color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3] }.ToFColor(true);

    UAirBlueprintLib::RunCommandOnGameThread([this, &points_start, &points_end, &color, thickness, arrow_size, duration, is_persistent]() {
        for (int idx = 0; idx < points_start.size(); ++idx) {
            DrawDebugDirectionalArrow(simmode_->GetWorld(),
                                      simmode_->getGlobalNedTransform().fromGlobalNed(points_start[idx]),
                                      simmode_->getGlobalNedTransform().fromGlobalNed(points_end[idx]),
                                      arrow_size,
                                      color,
                                      is_persistent,
                                      duration,
                                      0,
                                      thickness);
        }
    },
                                             true);
}

void WorldSimApi::simPlotStrings(const std::vector<std::string>& strings, const std::vector<Vector3r>& positions, float scale, const std::vector<float>& color_rgba, float duration)
{
    // assert positions.size() == strings.size()
    FColor color = FLinearColor{ color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3] }.ToFColor(true);

    UAirBlueprintLib::RunCommandOnGameThread([this, &strings, &positions, &color, scale, duration]() {
        for (int idx = 0; idx < positions.size(); ++idx) {
            DrawDebugString(simmode_->GetWorld(),
                            simmode_->getGlobalNedTransform().fromGlobalNed(positions[idx]),
                            FString(strings[idx].c_str()),
                            NULL,
                            color,
                            duration,
                            false,
                            scale);
        }
    },
                                             true);
}

void WorldSimApi::simPlotTransforms(const std::vector<Pose>& poses, float scale, float thickness, float duration, bool is_persistent)
{
    UAirBlueprintLib::RunCommandOnGameThread([this, &poses, scale, thickness, duration, is_persistent]() {
        for (const auto& pose : poses) {
            DrawDebugCoordinateSystem(simmode_->GetWorld(),
                                      simmode_->getGlobalNedTransform().fromGlobalNed(pose.position),
                                      simmode_->getGlobalNedTransform().fromNed(pose.orientation).Rotator(),
                                      scale,
                                      is_persistent,
                                      duration,
                                      0,
                                      thickness);
        }
    },
                                             true);
}

void WorldSimApi::simPlotTransformsWithNames(const std::vector<Pose>& poses, const std::vector<std::string>& names, float tf_scale, float tf_thickness, float text_scale, const std::vector<float>& text_color_rgba, float duration)
{
    // assert poses.size() == names.size()
    FColor color = FLinearColor{ text_color_rgba[0], text_color_rgba[1], text_color_rgba[2], text_color_rgba[3] }.ToFColor(true);

    UAirBlueprintLib::RunCommandOnGameThread([this, &poses, &names, &color, tf_scale, tf_thickness, text_scale, duration]() {
        for (int idx = 0; idx < poses.size(); ++idx) {
            DrawDebugCoordinateSystem(simmode_->GetWorld(),
                                      simmode_->getGlobalNedTransform().fromGlobalNed(poses[idx].position),
                                      simmode_->getGlobalNedTransform().fromNed(poses[idx].orientation).Rotator(),
                                      tf_scale,
                                      false,
                                      duration,
                                      0,
                                      tf_thickness);
            DrawDebugString(simmode_->GetWorld(),
                            simmode_->getGlobalNedTransform().fromGlobalNed(poses[idx]).GetLocation(),
                            FString(names[idx].c_str()),
                            NULL,
                            color,
                            duration,
                            false,
                            text_scale);
        }
    },
                                             true);
}

std::vector<WorldSimApi::MeshPositionVertexBuffersResponse> WorldSimApi::getMeshPositionVertexBuffers() const
{
    std::vector<WorldSimApi::MeshPositionVertexBuffersResponse> responses;
    UAirBlueprintLib::RunCommandOnGameThread([&responses]() {
        responses = UAirBlueprintLib::GetStaticMeshComponents();
    },
                                             true);
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

std::vector<std::string> WorldSimApi::listVehicles() const
{
    std::vector<std::string> vehicle_names;

    UAirBlueprintLib::RunCommandOnGameThread([this, &vehicle_names]() {
        vehicle_names = (simmode_->getApiProvider()->getVehicleSimApis()).keys();
    },
                                             true);

    // Remove '' from the list, representing default vehicle
    auto position = std::find(vehicle_names.begin(), vehicle_names.end(), "");
    if (position != vehicle_names.end())
        vehicle_names.erase(position);
    return vehicle_names;
}

std::string WorldSimApi::getSettingsString() const
{
    return msr::airlib::AirSimSettings::singleton().settings_text_;
}

// Race API
void WorldSimApi::disableRaceLogging()
{
    this->simmode_->RaceLoggingDisabled = true;
}

void WorldSimApi::startRace(const int race_tier)
{
    gate_noise_map_.clear();
    std::srand(static_cast<unsigned>(std::time(0)));

    UAirBlueprintLib::RunCommandOnGameThread([this, race_tier]() {
        // Get all gates
        TArray<AActor*> matching_actors;
        UGameplayStatics::GetAllActorsWithTag(this->simmode_->GetWorld(), "race_gate", matching_actors);
        for (auto actor : matching_actors) {
            if (race_tier == 1) {
                this->gate_noise_map_.insert(std::pair<FString, Pose>(actor->GetName(), Pose()));
            }
            else {
                this->gate_noise_map_.insert(std::pair<FString, Pose>(actor->GetName(), generateNoise()));
            }
        }

        this->simmode_->OnStartRace.Broadcast(race_tier);
    },
                                             true);
}

void WorldSimApi::startBenchmarkRace(const int race_tier)
{
    int race_tier_internal = race_tier;
    gate_noise_map_.clear();
    std::srand(static_cast<unsigned>(std::time(0)));

    UAirBlueprintLib::RunCommandOnGameThread([this, race_tier_internal]() {
        // Get all gates
        TArray<AActor*> matching_actors;
        UGameplayStatics::GetAllActorsWithTag(this->simmode_->GetWorld(), "race_gate", matching_actors);
        for (auto actor : matching_actors) {
            if (race_tier_internal == 1) {
                this->gate_noise_map_.insert(std::pair<FString, Pose>(actor->GetName(), Pose()));
            }
            else {
                this->gate_noise_map_.insert(std::pair<FString, Pose>(actor->GetName(), generateNoise()));
            }
        }

        this->simmode_->OnStartRace.Broadcast(race_tier_internal);
    },
                                             true);
}

Pose WorldSimApi::generateNoise(float position_magnitude, float orientation_magnitude)
{
    // b/w low = -1 and high = +1
    // low + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX / (high - low)));
    Vector3r position_noise(
        -1 + static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / 2)),
        -1 + static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / 2)),
        -1 + static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / 2)));

    // b/w 0 and +1
    Eigen::Vector4f orientation_noise(
        static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX),
        static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX),
        static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX),
        static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX));

    position_noise.normalize();
    orientation_noise.normalize();
    orientation_noise *= orientation_magnitude;
    return Pose(position_magnitude * position_noise, Quaternionr(orientation_noise[0], orientation_noise[1], orientation_noise[2], orientation_noise[3]));
}

void WorldSimApi::resetRace()
{
    UAirBlueprintLib::RunCommandOnGameThread([this]() {
        this->simmode_->OnResetRace.Broadcast();
    },
                                             true);
}

bool WorldSimApi::getDisqualified(const std::string& racer_name)
{
    auto* prog = simmode_->RacerStates.Find(FString(racer_name.c_str()));
    return prog != nullptr ? prog->Disqualified : false;
}

int WorldSimApi::getLastGatePassed(const std::string& racer_name)
{
    auto* prog = simmode_->RacerStates.Find(FString(racer_name.c_str()));
    return prog != nullptr ? prog->LastGatePassed : -1;
}
