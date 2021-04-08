#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <sdf_tools/sdf.hpp>
#include <sdf_tools/sdf_generation.hpp>
#include <sdf_tools/sdf_builder.hpp>

using namespace sdf_tools;

SDF_Builder::SDF_Builder(const Eigen::Isometry3d& origin_transform, const std::string& frame, const double x_size, const double y_size, const double z_size, const double resolution, const float OOB_value)
{
    origin_transform_ = origin_transform;
    frame_ = frame;
    x_size_ = x_size;
    y_size_ = y_size;
    z_size_ = z_size;
    resolution_ = resolution;
    OOB_value_ = OOB_value;
    initialized_ = true;
    has_cached_sdf_ = false;
    has_cached_collmap_ = false;
    has_planning_scene_ = false;
}

SDF_Builder::SDF_Builder(const std::string& frame, const double x_size, const double y_size, const double z_size, const double resolution, const float OOB_value)
{
    const Eigen::Translation3d origin_translation(-x_size * 0.5, -y_size * 0.5, -z_size * 0.5);
    const Eigen::Quaterniond origin_rotation = Eigen::Quaterniond::Identity();
    origin_transform_ = origin_translation * origin_rotation;
    frame_ = frame;
    x_size_ = x_size;
    y_size_ = y_size;
    z_size_ = z_size;
    resolution_ = resolution;
    OOB_value_ = OOB_value;
    initialized_ = true;
    has_cached_sdf_ = false;
    has_cached_collmap_ = false;
    has_planning_scene_ = false;
}

const SignedDistanceField& SDF_Builder::GetCachedSDF() const
{
    if (has_cached_sdf_)
    {
        return cached_sdf_;
    }
    else
    {
        throw std::invalid_argument("No cached SDF available");
    }
}

const VoxelGrid::VoxelGrid<uint8_t>& SDF_Builder::GetCachedCollisionMap() const
{
    if (has_cached_collmap_)
    {
        return cached_collmap_;
    }
    else
    {
        throw std::invalid_argument("No cached Collision Map available");
    }
}
/*
VoxelGrid::VoxelGrid<uint8_t> SDF_Builder::UpdateCollisionMapFromOctomap()
{
    VoxelGrid::VoxelGrid<uint8_t> collision_field(origin_transform_, resolution_, x_size_, y_size_, z_size_, 0);

    //#pragma omp parallel for collapse(3)
    for (int64_t x_index = 0; x_index < collision_field.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < collision_field.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < collision_field.GetNumZCells(); z_index++)
            {     
                bool occupied = false;
                // Convert SDF indices into a real-world location          
                const Eigen::Vector4d location = collision_field.GridIndexToLocation(x_index, y_index, z_index);
                octomap::OcTreeNode* result = tree_->search(location(0), location(1), location(2));

                if (result != NULL)
                    occupied = tree_->isNodeOccupied(result);
                if (occupied)
                {
                    // Mark as filled
                    //std::cout << "Collision" << std::endl;
                    uint8_t status = 1;
                    collision_field.SetValue(x_index, y_index, z_index, status);
                }
                else
                {
                    // Mark as free space
                    //std::cout << "No collision" << std::endl;
                    uint8_t status = 0;
                    collision_field.SetValue(x_index, y_index, z_index, status);
                }
            }
        }
    }
    // Export the collision map
    cached_collmap_ = collision_field;
    has_cached_collmap_ = true;
    return collision_field;
}

SignedDistanceField SDF_Builder::UpdateSDFFromOctomap()
{
    // Make a temporary grid to perform index<->location mapping
    const VoxelGrid::VoxelGrid<uint8_t> temp_grid(origin_transform_, resolution_, x_size_, y_size_, z_size_, OOB_value_);
    std::function<bool(const VoxelGrid::GRID_INDEX&)> is_filled_fn = [&] (const VoxelGrid::GRID_INDEX& index)
    {
        // Convert SDF indices into a real-world location
        const Eigen::Vector4d location = temp_grid.GridIndexToLocation(index);
        bool occupied = false;
        octomap::OcTreeNode* result = tree_->search(location(0), location(1), location(2));

        if (result != NULL)
            occupied = tree_->isNodeOccupied(result);

        if (occupied)
        {
            // Mark as filled
            return true;
        }
        else
        {
            // Mark as free space
            return false;
        }
    };
    cached_sdf_ = sdf_generation::ExtractSignedDistanceField(temp_grid, is_filled_fn, OOB_value_, frame_, false).first;
    has_cached_sdf_ = true;
    // Export the SDF
    return cached_sdf_;
}
*/