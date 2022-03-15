/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#ifndef GAZEBO_GPU_LASER_CUBE_FACE_H
#define GAZEBO_GPU_LASER_CUBE_FACE_H

#include <string>
#include <utility>
#include <vector>

#include <ignition/math/Vector2.hh>

#include "gazebo/rendering/ogre_gazebo.h"

namespace gazebo
{

  namespace rendering
  {

    /// \brief Cube map face ID
    enum class GpuLaserCubeFaceId
    {
      CUBE_FRONT_FACE,
      CUBE_LEFT_FACE,
      CUBE_REAR_FACE,
      CUBE_RIGHT_FACE,
      CUBE_TOP_FACE,
      CUBE_BOTTOM_FACE
    };

    /// \brief Stores mapping of a single ray (combination of azimuth and elevation)
    /// First element is ID of the corresponding cube map face
    /// Second element is x/y coordinate of ray intersection with face (in range [0,1]x[0,1])
    typedef std::pair<GpuLaserCubeFaceId, ignition::math::Vector2d> GpuLaserCubeMappingPoint;

    struct GpuLaserCameraSetting
    {
      double azimuthOffset;
      double elevationOffset;
    };

    struct GpuLaserCubeFace
    {
      std::string name;
      std::vector<float> depthImg;
      Ogre::TexturePtr texture;
      GpuLaserCameraSetting cameraSetting;
    };

  }

}

#endif //GAZEBO_GPU_LASER_CUBE_FACE_H
