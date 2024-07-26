#pragma once
#include <igcclib/core/igcclib_eigen_def.hpp>

namespace _NS_UTILITY{
    // default voxelization precision ratio, larger value leads to sparser voxelization, should be >=1.0
    const double DEFAULT_VOXEL_PRECISION_RATIO = 5.0;

    /**
     * @brief voxelize mesh to points
     * 
     * @param vertices (n,3) mesh vertices
     * @param faces (n,3) mesh faces
     * @param voxel_unit size of a voxel
     * @param precision precision of voxelization, which expands the voxel size by half of this amount so as to avoid holes. -1 means default.
     * @return fMATRIX (n,3) points, centers of voxels touched by the mesh surface
     */
    fMATRIX voxelize_mesh_to_points(const fMATRIX &vertices, const iMATRIX &faces, double voxel_unit, double precision = -1);

    /**
     * @brief voxelize mesh to 3d grid
     * 
     * @param vertices (n,3) mesh vertices
     * @param faces (n,3) mesh faces
     * @param voxel_unit size of a voxel
     * @param precision precision of voxelization, which expands the voxel size by half of this amount so as to avoid holes. -1 means default.
     * @return fMATRIX (n,3) voxel indices, of voxels that touched by the mesh surface, may have duplicates
     */
    iMATRIX voxelize_mesh_to_grid(const fMATRIX &vertices, const iMATRIX &faces, double voxel_unit, double precision = -1);
}