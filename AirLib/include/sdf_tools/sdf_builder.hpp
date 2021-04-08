#include <stdlib.h>
#include <vector>
#include <string>
#include <Eigen/Geometry>
#include "sdf_tools/sdf.hpp"

#ifndef SDF_BUILDER_HPP
#define SDF_BUILDER_HPP

namespace sdf_tools
{
    static const uint8_t USE_CACHED = 0x00;
    static const uint8_t USE_ONLY_OCTOMAP = 0x01;
    static const uint8_t USE_ONLY_COLLISION_OBJECTS = 0x02;
    static const uint8_t USE_FULL_PLANNING_SCENE = 0x03;

    class SDF_Builder
    {
    protected:

        bool initialized_;
        bool has_cached_sdf_;
        bool has_cached_collmap_;
        bool has_planning_scene_;
        Eigen::Isometry3d origin_transform_;
        std::string frame_;
        double x_size_;
        double y_size_;
        double z_size_;
        double resolution_;
        float OOB_value_;
        SignedDistanceField cached_sdf_;
        VoxelGrid::VoxelGrid<uint8_t> cached_collmap_;

        bool BuildInternalPlanningScene();

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SDF_Builder(const Eigen::Isometry3d& origin_transform, const std::string& frame, const double x_size, const double y_size, const double z_size, const double resolution, const float OOB_value);

        SDF_Builder(const std::string& frame, const double x_size, const double y_size, const double z_size, const double resolution, const float OOB_value);

        SDF_Builder()
        {
            initialized_ = false;
            has_cached_sdf_ = false;
            has_cached_collmap_ = false;
            has_planning_scene_ = false;
        }

        SignedDistanceField UpdateSDF(const uint8_t update_mode);

        const SignedDistanceField& GetCachedSDF() const;

        VoxelGrid::VoxelGrid<uint8_t> UpdateCollisionMap(const uint8_t update_mode);
        
        const VoxelGrid::VoxelGrid<uint8_t>& GetCachedCollisionMap() const;

    };


}

#endif // SDF_BUILDER_HPP
