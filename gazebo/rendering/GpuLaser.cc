/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <algorithm>
#include <sstream>

#include <ignition/math/Color.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#ifndef _WIN32
  #include <dirent.h>
#else
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
  #include "gazebo/common/win_dirent.h"
#endif

#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/MeshManager.hh"
#include "gazebo/common/Timer.hh"

#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/GpuLaser.hh"
#include "gazebo/rendering/GpuLaserPrivate.hh"

using namespace gazebo;
using namespace rendering;

int GpuLaserPrivate::texCount = 0;

//////////////////////////////////////////////////
GpuLaser::GpuLaser(const std::string &_namePrefix, ScenePtr _scene,
                   const bool _autoRender)
: Camera(_namePrefix, _scene, _autoRender),
  dataPtr(new GpuLaserPrivate)
{
  this->dataPtr->laserScan = NULL;
  this->dataPtr->matFirstPass = NULL;
  this->dataPtr->matSecondPass = NULL;
  this->dataPtr->secondPassTexture = NULL;
  this->dataPtr->orthoCam = NULL;
  this->dataPtr->w2nd = 0;
  this->dataPtr->h2nd = 0;
  this->cameraCount = 0;
  this->dataPtr->textureCount = 0;
  this->dataPtr->visual.reset();
}

//////////////////////////////////////////////////
GpuLaser::~GpuLaser()
{
  this->Fini();
}

//////////////////////////////////////////////////
void GpuLaser::Load(sdf::ElementPtr _sdf)
{
  Camera::Load(_sdf);
}

//////////////////////////////////////////////////
void GpuLaser::Load()
{
  Camera::Load();
}

//////////////////////////////////////////////////
void GpuLaser::Init()
{
  Camera::Init();
  this->dataPtr->visual.reset(new Visual(this->Name()+"second_pass_canvas",
     this->GetScene()->WorldVisual()));
}

//////////////////////////////////////////////////
void GpuLaser::Fini()
{
  for (const auto& face : this->dataPtr->cube_map_faces)
  {
    if (!face.second.texture.isNull())
    {
      Ogre::TextureManager::getSingleton().remove(
          face.second.texture->getName());
    }
  }

  if (this->dataPtr->secondPassTexture)
  {
    Ogre::TextureManager::getSingleton().remove(
        this->dataPtr->secondPassTexture->getName());
    this->dataPtr->secondPassTexture = nullptr;
  }

  if (this->dataPtr->orthoCam)
  {
    this->scene->OgreSceneManager()->destroyCamera(this->dataPtr->orthoCam);
    this->dataPtr->orthoCam = nullptr;
  }

  this->dataPtr->visual.reset();
  this->dataPtr->texIdx.clear();
  this->dataPtr->texCount = 0;

  delete [] this->dataPtr->laserScan;
  this->dataPtr->laserScan = nullptr;

  Camera::Fini();
}

//////////////////////////////////////////////////
void GpuLaser::CreateLaserTexture(const std::string &_textureName)
{

  this->CreateOrthoCam();

  this->dataPtr->textureCount = 6; // TODO smarter

  unsigned int cube_face_tex_index = 0;
  for (auto& face : this->dataPtr->cube_map_faces)
  {
    std::stringstream texName;
    texName << _textureName << "_cube_face_texture_" << cube_face_tex_index;
    face.second.texture = Ogre::TextureManager::getSingleton().createManual(
        texName.str(), "General", Ogre::TEX_TYPE_2D,
        this->ImageWidth(), this->ImageHeight(), 0,
        Ogre::PF_FLOAT32_RGB, Ogre::TU_RENDERTARGET);

    this->Set1stPassTarget(face.second.texture->getBuffer()->getRenderTarget(), face.second); // TODO refactor Set1stPassTarget()

    face.second.render_target->setAutoUpdated(false);

    cube_face_tex_index++;
  }

  this->dataPtr->matFirstPass = (Ogre::Material*)(
  Ogre::MaterialManager::getSingleton().getByName("Gazebo/LaserScan1st").get());

  this->dataPtr->matFirstPass->load();
  this->dataPtr->matFirstPass->setCullingMode(Ogre::CULL_NONE);

  this->dataPtr->matSecondPass = (Ogre::Material*)(
  Ogre::MaterialManager::getSingleton().getByName("Gazebo/LaserScan2nd").get());

  this->dataPtr->matSecondPass->load();

  Ogre::TextureUnitState *texUnit;
  for (const auto& face : this->dataPtr->cube_map_faces)
  {
    unsigned int texIndex = this->dataPtr->texCount++;
    Ogre::Technique *technique = this->dataPtr->matSecondPass->getTechnique(0);
    GZ_ASSERT(technique, "GpuLaser material script error: technique not found");

    Ogre::Pass *pass = technique->getPass(0);
    GZ_ASSERT(pass, "GpuLaser material script error: pass not found");

    if (!pass->getTextureUnitState(face.second.texture->getName()))
    {
      texUnit = pass->createTextureUnitState(face.second.texture->getName(), texIndex);

      this->dataPtr->texIdx.push_back(texIndex);

      texUnit->setTextureFiltering(Ogre::TFO_NONE);
      texUnit->setTextureAddressingMode(Ogre::TextureUnitState::TAM_MIRROR);
    }
  }

  this->CreateCanvas();
}

//////////////////////////////////////////////////
void GpuLaser::PostRender()
{
  for (auto& face : this->dataPtr->cube_map_faces)
  {
    face.second.render_target->swapBuffers();
  }

  if (this->newData && this->captureData)
  {
    const size_t size = this->dataPtr->w2nd * this->dataPtr->h2nd * 3;

    // Blit the depth buffer if needed
    if (this->dataPtr->laserBuffer.empty())
    {
      this->dataPtr->laserBuffer.resize(size);
    }

    for (auto& face : this->dataPtr->cube_map_faces)
    {
      const Ogre::HardwarePixelBufferSharedPtr pixelBuffer = face.second.texture->getBuffer();

      const unsigned int frameWidth = face.second.viewport->getActualWidth();
      const unsigned int frameHeight = face.second.viewport->getActualHeight();
      const size_t frameSize = frameWidth * frameHeight * 3;

      face.second.depth_img.resize(frameSize);

      Ogre::PixelBox dstBox(frameWidth, frameHeight,
                            1, Ogre::PF_FLOAT32_RGB, face.second.depth_img.data());

      pixelBuffer->blitToMemory(dstBox);

      // debug output of depth image
      std::vector<float> raw_img(frameSize);
      Ogre::PixelBox imgBox(frameWidth, frameHeight,
                            1, Ogre::PF_FLOAT32_RGB, raw_img.data());
      pixelBuffer->blitToMemory(imgBox);
      Ogre::Image output_img;
      output_img = output_img.loadDynamicImage(static_cast<unsigned char*>(imgBox.data), imgBox.getWidth(), imgBox.getHeight(), Ogre::PF_FLOAT32_RGB);

      // debug draw mapping
      for (unsigned int azimuth_i = 0; azimuth_i < this->dataPtr->w2nd; azimuth_i++)
      {
        for (unsigned int elevation_i = 0; elevation_i < this->dataPtr->h2nd; elevation_i++)
        {
          const GpuLaserCubeMappingPoint& point = mapping[azimuth_i][elevation_i];

          if (point.first == face.first)
          {
            const auto x = static_cast<unsigned int>(point.second.X() * (this->ImageWidth() - 1));
            const auto y = static_cast<unsigned int>(point.second.Y() * (this->ImageHeight() - 1));

            output_img.setColourAt(Ogre::ColourValue(0., 1., 0.), x, y, 0);
          }
        }
      }
      // debug draw mapping

      output_img.save("Gpu_cam_img_" + face.second.name + ".exr");
      gzmsg << "Saved image to file " << face.second.name << "\n";
      // debug output of depth image
    }

    // read ranges
    GZ_ASSERT(this->dataPtr->w2nd == mapping.size(), "cube face mapping size doesn't match number of horizontal rays");

    for (unsigned int azimuth_i = 0; azimuth_i < this->dataPtr->w2nd; azimuth_i++)
    {
      GZ_ASSERT(this->dataPtr->h2nd == mapping[azimuth_i].size(), "cube face mapping size doesn't match number of vertical rays");

      for (unsigned int elevation_i = 0; elevation_i < this->dataPtr->h2nd; elevation_i++)
      {
        const unsigned int index = (elevation_i * this->dataPtr->w2nd + azimuth_i) * 3;

        const GpuLaserCubeMappingPoint& point = mapping[azimuth_i][elevation_i];

        // pixel coordinates
        const auto x = static_cast<unsigned int>(point.second.X() * (this->ImageWidth() - 1));
        const auto y = static_cast<unsigned int>(point.second.Y() * (this->ImageHeight() - 1));

        const unsigned int frame_index = (y * this->ImageWidth() + x) * 3;

        this->dataPtr->laserBuffer.at(index) = this->dataPtr->cube_map_faces.at(point.first).depth_img.at(frame_index);
        this->dataPtr->laserBuffer.at(index + 1) = this->dataPtr->cube_map_faces.at(point.first).depth_img.at(frame_index + 1);
        this->dataPtr->laserBuffer.at(index + 2) = this->dataPtr->cube_map_faces.at(point.first).depth_img.at(frame_index + 2);
      }
    }

    if (!this->dataPtr->laserScan)
    {
      this->dataPtr->laserScan = new float[size];
    }

    memcpy(this->dataPtr->laserScan, this->dataPtr->laserBuffer.data(),
           size * sizeof(this->dataPtr->laserScan[0]));

    this->dataPtr->newLaserFrame(this->dataPtr->laserScan, this->dataPtr->w2nd,
        this->dataPtr->h2nd, 3, "BLABLA");
  }

  this->newData = false;
}

/////////////////////////////////////////////////
void GpuLaser::UpdateRenderTarget(Ogre::RenderTarget *_target,
                   Ogre::Material *_material, Ogre::Camera *_cam,
                   const bool _updateTex)
{
  Ogre::RenderSystem *renderSys;
  Ogre::Viewport *vp = NULL;
  Ogre::SceneManager *sceneMgr = this->scene->OgreSceneManager();
  Ogre::Pass *pass;

  renderSys = this->scene->OgreSceneManager()->getDestinationRenderSystem();
  // Get pointer to the material pass
  pass = _material->getBestTechnique()->getPass(0);

  // Render the depth texture
  // OgreSceneManager::_render function automatically sets farClip to 0.
  // Which normally equates to infinite distance. We don't want this. So
  // we have to set the distance every time.
  _cam->setFarClipDistance(this->FarClip());

  Ogre::AutoParamDataSource autoParamDataSource;

  vp = _target->getViewport(0);

  // Need this line to render the ground plane. No idea why it's necessary.
  renderSys->_setViewport(vp);
  sceneMgr->_setPass(pass, true, false);
  autoParamDataSource.setCurrentPass(pass);
  autoParamDataSource.setCurrentViewport(vp);
  autoParamDataSource.setCurrentRenderTarget(_target);
  autoParamDataSource.setCurrentSceneManager(sceneMgr);
  autoParamDataSource.setCurrentCamera(_cam, true);

  renderSys->setLightingEnabled(false);
  renderSys->_setFog(Ogre::FOG_NONE);

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
  pass->_updateAutoParamsNoLights(&autoParamDataSource);
#else
  pass->_updateAutoParams(&autoParamDataSource, 1);
#endif

  if (_updateTex)
  {
    pass->getFragmentProgramParameters()->setNamedConstant("tex1",
      this->dataPtr->texIdx[0]);
    if (this->dataPtr->texIdx.size() > 1)
    {
      pass->getFragmentProgramParameters()->setNamedConstant("tex2",
        this->dataPtr->texIdx[1]);
      if (this->dataPtr->texIdx.size() > 2)
        pass->getFragmentProgramParameters()->setNamedConstant("tex3",
          this->dataPtr->texIdx[2]);
    }
  }

  // NOTE: We MUST bind parameters AFTER updating the autos
  if (pass->hasVertexProgram())
  {
    renderSys->bindGpuProgram(
        pass->getVertexProgram()->_getBindingDelegate());

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
    renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
    pass->getVertexProgramParameters());
#else
    renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
      pass->getVertexProgramParameters(), 1);
#endif
  }

  if (pass->hasFragmentProgram())
  {
    renderSys->bindGpuProgram(
    pass->getFragmentProgram()->_getBindingDelegate());

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
    renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
    pass->getFragmentProgramParameters());
#else
      renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
      pass->getFragmentProgramParameters(), 1);
#endif
  }
}

/////////////////////////////////////////////////
void GpuLaser::notifyRenderSingleObject(Ogre::Renderable *_rend,
      const Ogre::Pass* /*pass*/, const Ogre::AutoParamDataSource* /*source*/,
      const Ogre::LightList* /*lights*/, bool /*supp*/)
{
  Ogre::Vector4 retro = Ogre::Vector4(0, 0, 0, 0);
  try
  {
    retro = _rend->getCustomParameter(1);
  }
  catch(Ogre::ItemIdentityException& e)
  {
    _rend->setCustomParameter(1, Ogre::Vector4(0, 0, 0, 0));
  }

  Ogre::Pass *pass = this->dataPtr->currentMat->getBestTechnique()->getPass(0);
  Ogre::RenderSystem *renderSys =
                  this->scene->OgreSceneManager()->getDestinationRenderSystem();

  Ogre::AutoParamDataSource autoParamDataSource;

  Ogre::Viewport *vp = this->dataPtr->currentTarget->getViewport(0);

  renderSys->_setViewport(vp);
  autoParamDataSource.setCurrentRenderable(_rend);
  autoParamDataSource.setCurrentPass(pass);
  autoParamDataSource.setCurrentViewport(vp);
  autoParamDataSource.setCurrentRenderTarget(this->dataPtr->currentTarget);
  autoParamDataSource.setCurrentSceneManager(this->scene->OgreSceneManager());
  autoParamDataSource.setCurrentCamera(this->camera, true);

  pass->_updateAutoParams(&autoParamDataSource,
      Ogre::GPV_GLOBAL | Ogre::GPV_PER_OBJECT);
  pass->getFragmentProgramParameters()->setNamedConstant("retro", retro[0]);
  renderSys->bindGpuProgram(
      pass->getVertexProgram()->_getBindingDelegate());

  renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
      pass->getVertexProgramParameters(),
      Ogre::GPV_GLOBAL | Ogre::GPV_PER_OBJECT);

  renderSys->bindGpuProgram(
      pass->getFragmentProgram()->_getBindingDelegate());

  renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
      pass->getFragmentProgramParameters(),
      Ogre::GPV_GLOBAL | Ogre::GPV_PER_OBJECT);
}

//////////////////////////////////////////////////
void GpuLaser::RenderImpl()
{
  common::Timer firstPassTimer, secondPassTimer;

  firstPassTimer.Start();

  Ogre::SceneManager *sceneMgr = this->scene->OgreSceneManager();

  sceneMgr->_suppressRenderStateChanges(true);
  sceneMgr->addRenderObjectListener(this);

  this->dataPtr->currentMat = this->dataPtr->matFirstPass;

  for (auto& face : this->dataPtr->cube_map_faces)
  {
    this->ApplyCameraSetting(face.second.camera_setting);

    this->dataPtr->currentTarget = face.second.render_target;
    this->UpdateRenderTarget(face.second.render_target, this->dataPtr->matFirstPass, this->camera);
    face.second.render_target->update(false);

    this->RevertCameraSetting(face.second.camera_setting);
  }

  sceneMgr->removeRenderObjectListener(this);

  double firstPassDur = firstPassTimer.GetElapsed().Double();

  sceneMgr->_suppressRenderStateChanges(false);

  double secondPassDur = secondPassTimer.GetElapsed().Double();
  this->dataPtr->lastRenderDuration = firstPassDur + secondPassDur;
}

//////////////////////////////////////////////////
const float* GpuLaser::LaserData() const
{
  return this->dataPtr->laserBuffer.data();
}

//////////////////////////////////////////////////
GpuLaser::DataIter GpuLaser::LaserDataBegin() const
{
  const unsigned int index = 0;
  // Data stuffed into three floats (RGB)
  const unsigned int skip = 3;
  // range data in R channel
  const unsigned int rangeOffset = 0;
  // intensity data in G channel
  const unsigned int intenOffset = 1;
  return DataIter(index, this->dataPtr->laserBuffer.data(), skip, rangeOffset,
      intenOffset, this->dataPtr->w2nd);
}

//////////////////////////////////////////////////
GpuLaser::DataIter GpuLaser::LaserDataEnd() const
{
  const unsigned int index = this->dataPtr->h2nd * this->dataPtr->w2nd;

  // Data stuffed into three floats (RGB)
  const unsigned int skip = 3;
  // range data in R channel
  const unsigned int rangeOffset = 0;
  // intensity data in G channel
  const unsigned int intenOffset = 1;
  return DataIter(index, this->dataPtr->laserBuffer.data(), skip, rangeOffset,
      intenOffset, this->dataPtr->w2nd);
}

/////////////////////////////////////////////////
void GpuLaser::CreateOrthoCam()
{
  this->dataPtr->pitchNodeOrtho =
    this->GetScene()->WorldVisual()->GetSceneNode()->createChildSceneNode();

  this->dataPtr->orthoCam = this->scene->OgreSceneManager()->createCamera(
        this->dataPtr->pitchNodeOrtho->getName() + "_ortho_cam");

  // Use X/Y as horizon, Z up
  this->dataPtr->orthoCam->pitch(Ogre::Degree(90));

  // Don't yaw along variable axis, causes leaning
  this->dataPtr->orthoCam->setFixedYawAxis(true, Ogre::Vector3::UNIT_Z);

  this->dataPtr->orthoCam->setDirection(1, 0, 0);

  this->dataPtr->pitchNodeOrtho->attachObject(this->dataPtr->orthoCam);
  this->dataPtr->orthoCam->setAutoAspectRatio(true);

  if (this->dataPtr->orthoCam)
  {
    this->dataPtr->orthoCam->setNearClipDistance(0.01);
    this->dataPtr->orthoCam->setFarClipDistance(0.02);
    this->dataPtr->orthoCam->setRenderingDistance(0.02);

    this->dataPtr->orthoCam->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
  }
}

/////////////////////////////////////////////////
Ogre::Matrix4 GpuLaser::BuildScaledOrthoMatrix(const float _left,
    const float _right, const float _bottom, const float _top,
    const float _near, const float _far)
{
  float invw = 1 / (_right - _left);
  float invh = 1 / (_top - _bottom);
  float invd = 1 / (_far - _near);

  Ogre::Matrix4 proj = Ogre::Matrix4::ZERO;
  proj[0][0] = 2 * invw;
  proj[0][3] = -(_right + _left) * invw;
  proj[1][1] = 2 * invh;
  proj[1][3] = -(_top + _bottom) * invh;
  proj[2][2] = -2 * invd;
  proj[2][3] = -(_far + _near) * invd;
  proj[3][3] = 1;

  return proj;
}

//////////////////////////////////////////////////
void GpuLaser::Set1stPassTarget(Ogre::RenderTarget *_target,
                                GpuLaserCubeFace& cube_face)
{
  cube_face.render_target = _target;

  if (_target)
  {
    // Setup the viewport to use the texture
    cube_face.viewport = cube_face.render_target->addViewport(this->camera);

    cube_face.viewport->setClearEveryFrame(true);
    cube_face.viewport->setOverlaysEnabled(false);
    cube_face.viewport->setShadowsEnabled(false);
    cube_face.viewport->setSkiesEnabled(false);
    cube_face.viewport->setBackgroundColour(Ogre::ColourValue(this->farClip, 0.0, 1.0));
    cube_face.viewport->setVisibilityMask(
        GZ_VISIBILITY_ALL & ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));
  }

  this->camera->setAspectRatio(this->RayCountRatio());
  this->camera->setFOVy(Ogre::Radian(this->CosVertFOV()));
}

//////////////////////////////////////////////////
void GpuLaser::Set2ndPassTarget(Ogre::RenderTarget *_target)
{
  this->dataPtr->secondPassTarget = _target;

  if (this->dataPtr->secondPassTarget)
  {
    // Setup the viewport to use the texture
    this->dataPtr->secondPassViewport =
        this->dataPtr->secondPassTarget->addViewport(this->dataPtr->orthoCam);
    this->dataPtr->secondPassViewport->setClearEveryFrame(true);
    this->dataPtr->secondPassViewport->setOverlaysEnabled(false);
    this->dataPtr->secondPassViewport->setShadowsEnabled(false);
    this->dataPtr->secondPassViewport->setSkiesEnabled(false);
    this->dataPtr->secondPassViewport->setBackgroundColour(
        Ogre::ColourValue(0.0, 1.0, 0.0));
    this->dataPtr->secondPassViewport->setVisibilityMask(
        GZ_VISIBILITY_ALL & ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));
  }
  Ogre::Matrix4 p = this->BuildScaledOrthoMatrix(
      0, static_cast<float>(this->dataPtr->w2nd / 10.0),
      0, static_cast<float>(this->dataPtr->h2nd / 10.0),
      0.01, 0.02);

  this->dataPtr->orthoCam->setCustomProjectionMatrix(true, p);
}

/////////////////////////////////////////////////
void GpuLaser::SetRangeCount(const unsigned int _w, const unsigned int _h)
{
  this->dataPtr->w2nd = _w;
  this->dataPtr->h2nd = _h;
}

/////////////////////////////////////////////////
void GpuLaser::CreateMesh()
{
  std::string meshName = this->Name() + "_undistortion_mesh";

  common::Mesh *mesh = new common::Mesh();
  mesh->SetName(meshName);

  common::SubMesh *submesh = new common::SubMesh();

  double dx, dy;
  submesh->SetPrimitiveType(common::SubMesh::POINTS);

  if (this->dataPtr->h2nd == 1)
  {
    dy = 0;
  }
  else
  {
    dy = 0.1;
  }

  dx = 0.1;

  // startX ranges from 0 to -(w2nd/10) at dx=0.1 increments
  // startY ranges from h2nd/10 to 0 at dy=0.1 decrements
  // see GpuLaser::Set2ndPassTarget() on how the ortho cam is set up
  double startX = dx;
  double startY = this->dataPtr->h2nd/10.0;

  // half of actual camera vertical FOV without padding
  double phi = this->VertFOV() / 2;
  double phiCamera = phi + std::abs(this->VertHalfAngle());
  double theta = this->CosHorzFOV() / 2;

  if (this->ImageHeight() == 1)
  {
    phi = 0;
  }

  // index of ray
  unsigned int ptsOnLine = 0;

  // total laser hfov
  double thfov = this->dataPtr->textureCount * this->CosHorzFOV();
  double hstep = thfov / (this->dataPtr->w2nd - 1);
  double vstep = 2 * phi / (this->dataPtr->h2nd - 1);

  if (this->dataPtr->h2nd == 1)
  {
    vstep = 0;
  }

  for (unsigned int j = 0; j < this->dataPtr->h2nd; ++j)
  {
    double gamma = 0;
    if (this->dataPtr->h2nd != 1)
    {
      // gamma: current vertical angle w.r.t. camera
      gamma = vstep * j - phi + this->VertHalfAngle();
    }

    for (unsigned int i = 0; i < this->dataPtr->w2nd; ++i)
    {
      // current horizontal angle from start of laser scan
      double delta = hstep * i;

      // index of texture that contains the depth value
      unsigned int texture = delta / this->CosHorzFOV();

      // cap texture index and horizontal angle
      if (texture > this->dataPtr->textureCount-1)
      {
        texture -= 1;
        delta -= hstep;
      }

      startX -= dx;
      if (ptsOnLine == this->dataPtr->w2nd)
      {
        ptsOnLine = 0;
        startX = 0;
        startY -= dy;
      }
      ptsOnLine++;

      // the texture/1000.0 value is used in the laser_2nd_pass.frag shader
      // as a trick to determine which camera texture to use when stitching
      // together the final depth image.
      submesh->AddVertex(texture/1000.0, startX, startY);

      // first compute angle from the start of current camera's horizontal
      // min angle, then set delta to be angle from center of current camera.
      delta = delta - (texture * this->CosHorzFOV());
      delta = delta - theta;

      // adjust uv coordinates of depth texture to match projection of current
      // laser ray the depth image plane.
      double u = 0.5 - tan(delta) / (2.0 * tan(theta));
      double v = 0.5 - (tan(gamma) * cos(theta)) /
          (2.0 * tan(phiCamera) * cos(delta));

      submesh->AddTexCoord(u, v);
      submesh->AddIndex(this->dataPtr->w2nd * j + i);
    }
  }

  mesh->AddSubMesh(submesh);

  this->dataPtr->undistMesh = mesh;

  common::MeshManager::Instance()->AddMesh(this->dataPtr->undistMesh);
}

/////////////////////////////////////////////////
void GpuLaser::CreateCanvas()
{
  this->CreateMesh();

  Ogre::Node *parent = this->dataPtr->visual->GetSceneNode()->getParent();
  parent->removeChild(this->dataPtr->visual->GetSceneNode());

  this->dataPtr->pitchNodeOrtho->addChild(
      this->dataPtr->visual->GetSceneNode());

  this->dataPtr->visual->InsertMesh(this->dataPtr->undistMesh);

  std::ostringstream stream;
  std::string meshName = this->dataPtr->undistMesh->GetName();
  stream << this->dataPtr->visual->GetSceneNode()->getName()
      << "_ENTITY_" << meshName;

  this->dataPtr->object = (Ogre::MovableObject*)
      (this->dataPtr->visual->GetSceneNode()->getCreator()->createEntity(
      stream.str(), meshName));

  this->dataPtr->visual->AttachObject(this->dataPtr->object);
  this->dataPtr->object->setVisibilityFlags(GZ_VISIBILITY_ALL
      & ~GZ_VISIBILITY_SELECTABLE);

  ignition::math::Pose3d pose;
  pose.Pos().Set(0.01, 0, 0);
  pose.Rot().Euler(ignition::math::Vector3d(0, 0, 0));

  this->dataPtr->visual->SetPose(pose);

  this->dataPtr->visual->SetDiffuse(ignition::math::Color(0, 1, 0, 0));
  this->dataPtr->visual->SetAmbient(ignition::math::Color(0, 1, 0, 0));
  this->dataPtr->visual->SetVisible(true);
  this->scene->AddVisual(this->dataPtr->visual);
}

//////////////////////////////////////////////////
void GpuLaser::SetHorzHalfAngle(const double _angle)
{
  this->horzHalfAngle = _angle;
}

//////////////////////////////////////////////////
void GpuLaser::SetVertHalfAngle(const double _angle)
{
  this->vertHalfAngle = _angle;
}

//////////////////////////////////////////////////
double GpuLaser::HorzHalfAngle() const
{
  return this->horzHalfAngle;
}

//////////////////////////////////////////////////
double GpuLaser::VertHalfAngle() const
{
  return this->vertHalfAngle;
}

//////////////////////////////////////////////////
void GpuLaser::SetIsHorizontal(const bool _horizontal)
{
  this->isHorizontal = _horizontal;
}

//////////////////////////////////////////////////
bool GpuLaser::IsHorizontal() const
{
  return this->isHorizontal;
}

//////////////////////////////////////////////////
double GpuLaser::HorzFOV() const
{
  return this->hfov;
}

//////////////////////////////////////////////////
double GpuLaser::VertFOV() const
{
  return this->vfov;
}

//////////////////////////////////////////////////
void GpuLaser::SetHorzFOV(const double _hfov)
{
  this->hfov = _hfov;
}

//////////////////////////////////////////////////
void GpuLaser::SetVertFOV(const double _vfov)
{
  this->vfov = _vfov;
}

//////////////////////////////////////////////////
double GpuLaser::CosHorzFOV() const
{
  return this->chfov;
}

//////////////////////////////////////////////////
void GpuLaser::SetCosHorzFOV(const double _chfov)
{
  this->chfov = _chfov;
}

//////////////////////////////////////////////////
double GpuLaser::CosVertFOV() const
{
  return this->cvfov;
}

//////////////////////////////////////////////////
void GpuLaser::SetCosVertFOV(const double _cvfov)
{
  this->cvfov = _cvfov;
}

//////////////////////////////////////////////////
double GpuLaser::NearClip() const
{
  return this->nearClip;
}

//////////////////////////////////////////////////
double GpuLaser::FarClip() const
{
  return this->farClip;
}

//////////////////////////////////////////////////
void GpuLaser::SetNearClip(const double _near)
{
  this->nearClip = _near;
}

//////////////////////////////////////////////////
void GpuLaser::SetFarClip(const double _far)
{
  this->farClip = _far;
}

//////////////////////////////////////////////////
double GpuLaser::RayCountRatio() const
{
  return this->rayCountRatio;
}

//////////////////////////////////////////////////
void GpuLaser::SetRayCountRatio(const double _rayCountRatio)
{
  this->rayCountRatio = _rayCountRatio;
}

//////////////////////////////////////////////////
event::ConnectionPtr GpuLaser::ConnectNewLaserFrame(
    std::function<void (const float *_frame, unsigned int _width,
    unsigned int _height, unsigned int _depth,
    const std::string &_format)> _subscriber)
{
  return this->dataPtr->newLaserFrame.Connect(_subscriber);
}

//////////////////////////////////////////////////
void GpuLaser::SetCameraSettings(std::array<GpuLaserCameraSetting, 6> settings)
{
  //this->dataPtr->cameraSettings = settings;
  // TODO
}

//////////////////////////////////////////////////
void GpuLaser::ApplyCameraSetting(const GpuLaserCameraSetting &setting)
{
  this->sceneNode->roll(Ogre::Radian(setting.azimuthOffset));
  this->sceneNode->yaw(Ogre::Radian(setting.elevationOffset));
}

//////////////////////////////////////////////////
void GpuLaser::RevertCameraSetting(const GpuLaserCameraSetting &setting)
{
  this->sceneNode->yaw(Ogre::Radian(-setting.elevationOffset));
  this->sceneNode->roll(Ogre::Radian(-setting.azimuthOffset));
}

//////////////////////////////////////////////////
void GpuLaser::InitMapping(const std::set<double>& azimuth_values, const std::set<double>& elevation_values)
{
  mapping.clear();
  mapping.reserve(azimuth_values.size());

  if (azimuth_values.empty())
  {
    return;
  }

  const double min_azimuth = *azimuth_values.begin();
  const double front_face_azimuth = min_azimuth + M_PI_4;

  this->dataPtr->cube_map_faces.insert(
      std::make_pair<GpuLaserCubeFaceId, GpuLaserCubeFace>(GpuLaserCubeFaceId::CUBE_FRONT_FACE, {
          "front", {}, {}, {}, {},
          {front_face_azimuth, 0.}}));
  this->dataPtr->cube_map_faces.insert(
      std::make_pair<GpuLaserCubeFaceId, GpuLaserCubeFace>(GpuLaserCubeFaceId::CUBE_LEFT_FACE, {
          "left", {}, {}, {}, {},
          {front_face_azimuth + M_PI_2, 0.0}}));
  this->dataPtr->cube_map_faces.insert(
      std::make_pair<GpuLaserCubeFaceId, GpuLaserCubeFace>(GpuLaserCubeFaceId::CUBE_REAR_FACE, {
          "rear", {}, {}, {}, {},
          {front_face_azimuth + M_PI, 0.0}}));
  this->dataPtr->cube_map_faces.insert(
      std::make_pair<GpuLaserCubeFaceId, GpuLaserCubeFace>(GpuLaserCubeFaceId::CUBE_RIGHT_FACE, {
          "right", {}, {}, {}, {},
          {front_face_azimuth + M_PI + M_PI_2, 0.0}}));
  this->dataPtr->cube_map_faces.insert(
      std::make_pair<GpuLaserCubeFaceId, GpuLaserCubeFace>(GpuLaserCubeFaceId::CUBE_TOP_FACE, {
          "top", {}, {}, {}, {},
          {front_face_azimuth, -M_PI_2}}));
  this->dataPtr->cube_map_faces.insert(
      std::make_pair<GpuLaserCubeFaceId, GpuLaserCubeFace>(GpuLaserCubeFaceId::CUBE_BOTTOM_FACE, {
          "bottom", {}, {}, {}, {},
          {front_face_azimuth, M_PI_2}}));

  for (const double azimuth : azimuth_values)
  {


    mapping.emplace_back();
    mapping.back().reserve(elevation_values.size());

    for (const double elevation : elevation_values)
    {
      mapping.back().push_back(FindCubeFaceMapping(azimuth - min_azimuth, elevation));
    }
  }
}

//////////////////////////////////////////////////
GpuLaserCubeMappingPoint GpuLaser::FindCubeFaceMapping(const double azimuth, const double elevation)
{
  if (azimuth < 0)
  {
    throw std::runtime_error("Azimuth angle should be relative to minimum angle, i.e. it must not be negative!");
  }

  const GpuLaserCubeFaceId face_id = FindCubeFace(azimuth, elevation);

  // center point of the face plane
  ignition::math::Vector3d plane_point;
  switch (face_id)
  {
    case GpuLaserCubeFaceId::CUBE_FRONT_FACE:
      plane_point = {0.5, 0., 0.};
      break;
    case GpuLaserCubeFaceId::CUBE_LEFT_FACE:
      plane_point = {0., 0.5, 0.};
      break;
    case GpuLaserCubeFaceId::CUBE_REAR_FACE:
      plane_point = {-0.5, 0., 0.};
      break;
    case GpuLaserCubeFaceId::CUBE_RIGHT_FACE:
      plane_point = {0., -0.5, 0.};
      break;
    case GpuLaserCubeFaceId::CUBE_TOP_FACE:
      plane_point = {0., 0., 0.5};
      break;
    case GpuLaserCubeFaceId::CUBE_BOTTOM_FACE:
      plane_point = {0., 0., -0.5};
      break;
    default:
      throw std::runtime_error("Invalid face ID");
  }

  const ignition::math::Vector3d plane_normal = plane_point.Normalized();
  const ignition::math::Vector3d viewing_ray = ViewingRay(azimuth, elevation);

  // calculate intersection of viewing ray with cube face plane
  const double s = (-plane_normal).Dot(-plane_point) / plane_normal.Dot(viewing_ray);

  // offset from face center to intersection
  const ignition::math::Vector3d intersection_offset = s * viewing_ray - plane_point;

  // offset in the 2D image on the face
  ignition::math::Vector2d intersection_image_offset;
  switch (face_id)
  {
    case GpuLaserCubeFaceId::CUBE_FRONT_FACE:
      intersection_image_offset = {-intersection_offset.Y(),
                                   -intersection_offset.Z()};
      break;
    case GpuLaserCubeFaceId::CUBE_LEFT_FACE:
      intersection_image_offset = {intersection_offset.X(),
                                   -intersection_offset.Z()};
      break;
    case GpuLaserCubeFaceId::CUBE_REAR_FACE:
      intersection_image_offset = {intersection_offset.Y(),
                                   -intersection_offset.Z()};
      break;
    case GpuLaserCubeFaceId::CUBE_RIGHT_FACE:
      intersection_image_offset = {-intersection_offset.X(),
                                   -intersection_offset.Z()};
      break;
    case GpuLaserCubeFaceId::CUBE_TOP_FACE:
      intersection_image_offset = {-intersection_offset.Y(),
                                   intersection_offset.X()};
      break;
    case GpuLaserCubeFaceId::CUBE_BOTTOM_FACE:
      intersection_image_offset = {-intersection_offset.Y(),
                                   -intersection_offset.X()};
      break;
    default:
      throw std::runtime_error("Invalid face ID");
  }

  // shift offset from image center to origin
  intersection_image_offset += {0.5, 0.5};

  intersection_image_offset.Set(ignition::math::clamp(intersection_image_offset.X(), 0., 1.),
                                ignition::math::clamp(intersection_image_offset.Y(), 0., 1.));

  return {face_id, intersection_image_offset};
}

GpuLaserCubeFaceId GpuLaser::FindCubeFace(const double azimuth, const double elevation)
{
  const ignition::math::Vector3d v = ViewingRay(azimuth, elevation);

  // find corresponding cube face
  if (std::abs(v.Z()) > std::abs(v.X()) && std::abs(v.Z()) > std::abs(v.Y()))
  {
    if (v.Z() >= 0)
    {
      return GpuLaserCubeFaceId::CUBE_TOP_FACE;
    }
    else
    {
      return GpuLaserCubeFaceId::CUBE_BOTTOM_FACE;
    }
  }
  else if (azimuth < M_PI_2)
  {
    return GpuLaserCubeFaceId::CUBE_FRONT_FACE;
  }
  else if (azimuth >= M_PI_2 && azimuth < M_PI)
  {
    return GpuLaserCubeFaceId::CUBE_LEFT_FACE;
  }
  else if (azimuth >= M_PI && azimuth < 3. * M_PI_2)
  {
    return GpuLaserCubeFaceId::CUBE_REAR_FACE;
  }
  else
  {
    return GpuLaserCubeFaceId::CUBE_RIGHT_FACE;
  }
}

ignition::math::Vector3d GpuLaser::ViewingRay(const double azimuth, const double elevation)
{
  return {std::cos(azimuth - M_PI_4) * std::cos(elevation),
          std::sin(azimuth - M_PI_4) * std::cos(elevation),
          std::sin(elevation)};
}
