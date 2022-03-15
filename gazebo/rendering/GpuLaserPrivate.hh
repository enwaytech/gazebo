/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef _GAZEBO_RENDERING_GPULASER_PRIVATE_HH_
#define _GAZEBO_RENDERING_GPULASER_PRIVATE_HH_

#include <string>
#include <vector>

#include "gazebo/rendering/GpuLaserCubeFace.hh"
#include "gazebo/rendering/RenderTypes.hh"

#include "gazebo/common/Event.hh"

namespace Ogre
{
  class Camera;
  class Material;
  class MovableObject;
  class RenderTarget;
  class SceneNode;
  class Texture;
  class Viewport;
}

namespace gazebo
{
  namespace common
  {
    class Mesh;
  }

  namespace rendering
  {
    /// \internal
    /// \brief Private data for the GpuLaser class
    class GpuLaserPrivate
    {
      /// \brief Event triggered when new laser range data are available.
      /// \param[in] _frame New frame containing raw laser data.
      /// \param[in] _width Width of frame.
      /// \param[in] _height Height of frame.
      /// \param[in] _depth Depth of frame.
      /// \param[in] _format Format of frame.
      public: event::EventT<void(const float *_frame, unsigned int _width,
                   unsigned int _height, unsigned int _depth,
                   const std::string &_format)> newLaserFrame;

      /// \brief Raw buffer of laser data.
      public: std::vector<float> laserBuffer;

      /// \brief Outgoing laser data, used by newLaserFrame event.
      public: float *laserScan;

      public: std::map<GpuLaserCubeFaceId, GpuLaserCubeFace> cube_map_faces;

      /// \brief Pointer to Ogre material for the first rendering pass.
      public: Ogre::Material *matFirstPass;

      /// \brief Temporary pointer to the current render target.
      public: Ogre::RenderTarget *currentTarget;

      /// \brief Temporary pointer to the current material.
      public: Ogre::Material *currentMat;

      /// \brief Image width of second pass.
      public: unsigned int w2nd;

      /// \brief Image height of second pass.
      public: unsigned int h2nd;
    };
  }
}
#endif
