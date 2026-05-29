/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * Copyright (c) 2020, TNG Technology Consulting GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rviz_default_plugins/displays/camera/camera_display.hpp"

#include <memory>
#include <string>
#include <sstream>
#include <algorithm>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderSystem.h>
#include <OgreRenderWindow.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <OgreTechnique.h>
#include <OgreCamera.h>

#include "image_transport/camera_common.hpp"
#include "image_transport/image_transport.hpp"

#include "rviz_rendering/material_manager.hpp"
#include "rviz_rendering/objects/axes.hpp"
#include "rviz_rendering/render_window.hpp"

#include "rviz_common/bit_allocator.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/display_group_visibility_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/uniform_string_stream.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"

#include "rviz_default_plugins/displays/image/get_transport_from_topic.hpp"
#include "rviz_default_plugins/displays/image/ros_image_texture.hpp"

namespace rviz_default_plugins
{

namespace displays
{

using rviz_common::properties::StatusLevel;

static const char * const CAM_INFO_STATUS = "Camera Info";
static const char * const TIME_STATUS = "Time";

const QString CameraDisplay::BACKGROUND("background");
const QString CameraDisplay::OVERLAY("overlay");
const QString CameraDisplay::BOTH("background and overlay");

bool validateFloats(const sensor_msgs::msg::CameraInfo & msg)
{
  bool valid = true;
  valid = valid && rviz_common::validateFloats(msg.d);
  valid = valid && rviz_common::validateFloats(msg.k);
  valid = valid && rviz_common::validateFloats(msg.r);
  valid = valid && rviz_common::validateFloats(msg.p);
  return valid;
}

static Ogre::Vector4 calculateScreenCorners(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr info,
  const Ogre::Vector2 & zoom)
{
  float x_corner_start, y_corner_start, x_corner_end, y_corner_end;

  if (info->roi.height != 0 || info->roi.width != 0) {
    // corners are computed according to roi
    x_corner_start = (2.0 * info->roi.x_offset / info->width - 1.0) * zoom.x;
    y_corner_start = (-2.0 * info->roi.y_offset / info->height + 1.0) * zoom.y;
    x_corner_end = x_corner_start + (2.0 * info->roi.width / info->width) * zoom.x;
    y_corner_end = y_corner_start - (2.0 * info->roi.height / info->height) * zoom.y;
  } else {
    x_corner_start = -1.0f * zoom.x;
    y_corner_start = 1.0f * zoom.y;
    x_corner_end = 1.0f * zoom.x;
    y_corner_end = -1.0f * zoom.y;
  }

  return {x_corner_start, y_corner_start, x_corner_end, y_corner_end};
}

CameraDisplay::CameraDisplay()
: tf_filter_(nullptr),
  texture_(std::make_unique<ROSImageTexture>()),
  new_caminfo_(false),
  caminfo_ok_(false),
  force_render_(false),
  camera_info_topic_auto_filled_(false)
{

  camera_info_topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Camera Info Topic", "",
    "sensor_msgs/msg/CameraInfo",
    "The topic to subscribe to for camera info messages. "
    "If empty, the topic is automatically derived from the image topic.",
    this, SLOT(updateCameraInfoTopic()));

  image_position_property_ = new rviz_common::properties::EnumProperty(
    "Image Rendering", BOTH,
    "Render the image behind all other geometry or overlay it on top, or both.",
    this);
  image_position_property_->addOption(BACKGROUND);
  image_position_property_->addOption(OVERLAY);
  image_position_property_->addOption(BOTH);

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Overlay Alpha", 0.5f,
    "The amount of transparency to apply to the camera image when rendered as overlay.",
    this, SLOT(updateAlpha()));
  alpha_property_->setMin(0.0f);
  alpha_property_->setMax(1.0f);

  zoom_property_ = new rviz_common::properties::FloatProperty(
    "Zoom Factor", 1.0f,
    "Set a zoom factor below 1 to see a larger part of the world, above 1 to magnify the image.",
    this);
  zoom_property_->setMin(0.00001f);
  zoom_property_->setMax(100000.0f);

  far_plane_property_ = new rviz_common::properties::FloatProperty(
    "Far Plane Distance", 100.0f,
    "Geometry beyond the camera's far plane will not be rendered.",
    this);
  far_plane_property_->setMin(0.00001f);
  far_plane_property_->setMax(100000.0f);

  publish_image_property_ = new rviz_common::properties::BoolProperty(
    "Publish Rendered Image", false,
    "Publish the rendered camera view (with overlays) as a ROS image topic.",
    this, SLOT(updatePublishImage()));

  publish_topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Publish Topic", "/camera_display/image_raw",
    "sensor_msgs/msg/Image",
    "Topic on which the rendered camera image is published.",
    this, SLOT(updatePublishImage()));
  publish_topic_property_->setHidden(true);

  publish_use_input_res_property_ = new rviz_common::properties::BoolProperty(
    "Use Input Resolution", true,
    "Automatically use the input image resolution for the published image. "
    "Disable to set a custom width and height.",
    this, SLOT(updatePublishUseInputRes()));
  publish_use_input_res_property_->setHidden(true);

  publish_width_property_ = new rviz_common::properties::IntProperty(
    "Publish Width", 640,
    "Width in pixels of the published image. Resizes the camera view panel.",
    this, SLOT(updatePublishSize()));
  publish_width_property_->setMin(1);
  publish_width_property_->setHidden(true);

  publish_height_property_ = new rviz_common::properties::IntProperty(
    "Publish Height", 480,
    "Height in pixels of the published image. Resizes the camera view panel.",
    this, SLOT(updatePublishSize()));
  publish_height_property_->setMin(1);
  publish_height_property_->setHidden(true);

}

CameraDisplay::~CameraDisplay()
{
  if (initialized()) {
    unsubscribe();
    context_->visibilityBits()->freeBits(vis_bit_);
    rviz_rendering::RenderWindowOgreAdapter::removeListener(render_panel_->getRenderWindow(), this);
  }
}

void CameraDisplay::onInitialize()
{
  ITDClass::onInitialize();

  camera_info_topic_property_->initialize(rviz_ros_node_);
  publish_topic_property_->initialize(rviz_ros_node_);

  setupSceneNodes();
  setupRenderPanel();

  auto render_window = render_panel_->getRenderWindow();
  rviz_rendering::RenderWindowOgreAdapter::addListener(render_window, this);

  vis_bit_ = context_->visibilityBits()->allocBit();
  rviz_rendering::RenderWindowOgreAdapter::setVisibilityMask(render_window, vis_bit_);

  visibility_property_ = new rviz_common::properties::DisplayGroupVisibilityProperty(
    vis_bit_, context_->getRootDisplayGroup(), this, "Visibility", true,
    "Changes the visibility of other Displays in the camera view.");

  visibility_property_->setIcon(
    rviz_common::loadPixmap("package://rviz_default_plugins/icons/visibility.svg", true));

  this->addChild(visibility_property_, 0);
}

void CameraDisplay::setupSceneNodes()
{
  background_scene_node_ = scene_node_->createChildSceneNode();
  overlay_scene_node_ = scene_node_->createChildSceneNode();

  static int count = 0;
  rviz_common::UniformStringStream materialName;
  materialName << "CameraDisplayObject" << count++ << "Material";

  Ogre::AxisAlignedBox aabInf;
  aabInf.setInfinite();

  background_material_ = createMaterial(materialName.str());

  background_screen_rect_ = createScreenRectangle(
    aabInf, background_material_, Ogre::RENDER_QUEUE_BACKGROUND);

  background_scene_node_->attachObject(background_screen_rect_.get());
  background_scene_node_->setVisible(false);

  overlay_material_ = background_material_->clone(materialName.str() + "fg");
  overlay_material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

  overlay_screen_rect_ = createScreenRectangle(
    aabInf, overlay_material_, Ogre::RENDER_QUEUE_OVERLAY - 1);

  overlay_scene_node_->attachObject(overlay_screen_rect_.get());
  overlay_scene_node_->setVisible(false);

  updateAlpha();
}

void CameraDisplay::setupRenderPanel()
{
  render_panel_ = std::make_unique<rviz_common::RenderPanel>();
  render_panel_->resize(640, 480);
  render_panel_->initialize(context_, true);
  setAssociatedWidget(render_panel_.get());

  static int count = 0;
  render_panel_->getRenderWindow()->setObjectName(
    "CameraDisplayRenderWindow" + QString::number(count++));
}

std::unique_ptr<Ogre::Rectangle2D> CameraDisplay::createScreenRectangle(
  const Ogre::AxisAlignedBox & bounding_box,
  const Ogre::MaterialPtr & material,
  Ogre::uint8 render_queue_group)
{
  auto screen_rect = std::make_unique<Ogre::Rectangle2D>(true);
  screen_rect->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);
  screen_rect->setRenderQueueGroup(render_queue_group);
  screen_rect->setBoundingBox(bounding_box);
  screen_rect->setMaterial(material);

  return screen_rect;
}

Ogre::MaterialPtr CameraDisplay::createMaterial(std::string name) const
{
  auto material = rviz_rendering::MaterialManager::createMaterialWithNoLighting(name);
  material->setDepthWriteEnabled(false);
  material->setDepthCheckEnabled(false);

  material->setCullingMode(Ogre::CULL_NONE);
  material->setSceneBlending(Ogre::SBT_REPLACE);

  Ogre::TextureUnitState * tu =
    material->getTechnique(0)->getPass(0)->createTextureUnitState();
  tu->setTextureName(texture_->getTexture()->getName());
  tu->setTextureFiltering(Ogre::TFO_NONE);
  tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
  tu->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, 0.0);

  return material;
}

void CameraDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent & evt)
{
  (void) evt;
  QString image_position = image_position_property_->getString();
  background_scene_node_->setVisible(
    caminfo_ok_ && (image_position == BACKGROUND || image_position == BOTH));
  overlay_scene_node_->setVisible(
    caminfo_ok_ && (image_position == OVERLAY || image_position == BOTH));

  // set view flags on all displays
  visibility_property_->update();
}

void CameraDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent & evt)
{
  (void) evt;

  if (image_pub_ && caminfo_ok_) {
    auto render_window = render_panel_->getRenderWindow();
    auto * viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(render_window);
    if (viewport) {
      auto * render_target = viewport->getTarget();
      const uint32_t rt_width = render_target->getWidth();
      const uint32_t rt_height = render_target->getHeight();
      if (rt_width > 0 && rt_height > 0) {
        // Crop to the image region, excluding letterbox/pillarbox padding.
        // zoom components > 1 mean the image overflows the viewport (no padding).
        float zx = std::min(last_zoom_.x, 1.0f);
        float zy = std::min(last_zoom_.y, 1.0f);
        uint32_t x1 = static_cast<uint32_t>((1.0f - zx) * 0.5f * rt_width);
        uint32_t y1 = static_cast<uint32_t>((1.0f - zy) * 0.5f * rt_height);
        uint32_t x2 = rt_width - x1;
        uint32_t y2 = rt_height - y1;
        uint32_t img_width = x2 - x1;
        uint32_t img_height = y2 - y1;
        sensor_msgs::msg::Image msg;
        msg.width = img_width;
        msg.height = img_height;
        msg.encoding = "rgb8";
        msg.step = img_width * 3;
        msg.data.resize(img_width * img_height * 3);
        Ogre::PixelBox pixel_box(img_width, img_height, 1, Ogre::PF_BYTE_RGB, msg.data.data());
        render_target->copyContentsToMemory(
          Ogre::Box(x1, y1, x2, y2), pixel_box, Ogre::RenderTarget::FB_AUTO);
        if (texture_->getImage()) {
          msg.header = texture_->getImage()->header;
        }
        image_pub_->publish(msg);
      }
    }
  }

  background_scene_node_->setVisible(false);
  overlay_scene_node_->setVisible(false);
}

void CameraDisplay::onEnable()
{
  subscribe();

  // Ensures the nodes are not visible in the main window
  background_scene_node_->setVisible(false);
  overlay_scene_node_->setVisible(false);
}

void CameraDisplay::onDisable()
{
  unsubscribe();
  clear();
}

void CameraDisplay::fixedFrameChanged()
{
  if (tf_filter_) {
    tf_filter_->setTargetFrame(fixed_frame_.toStdString());
  }
  reset();
}

void CameraDisplay::subscribe()
{
  if (camera_info_topic_auto_filled_) {
    camera_info_topic_property_->blockSignals(true);
    camera_info_topic_property_->setString("");
    camera_info_topic_property_->blockSignals(false);
    camera_info_topic_auto_filled_ = false;
  }

  try {
    ITDClass::subscribe();
  } catch (std::exception & e) {
    setStatus(StatusLevel::Error, "Topic", QString("Error subscribing: ") + e.what());
    return;
  }

  if (!subscription_ || subscription_->getSubscriber().getTopic().empty()) {
    return;
  }

  // Unregster the callback registered by the ImageTransportDisplay
  // and instead connect it to the TF filter
  subscription_callback_.disconnect();

  tf_filter_ = std::make_shared<
    tf2_ros::MessageFilter<sensor_msgs::msg::Image,
    rviz_common::transformation::FrameTransformer>>(
    *context_->getFrameManager()->getTransformer(),
    fixed_frame_.toStdString(), 10, rviz_ros_node_.lock()->get_raw_node());

  tf_filter_->connectInput(*subscription_);
  tf_filter_->registerCallback(
    [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
      this->incomingMessage(msg);
    });

  if ((!isEnabled()) || (topic_property_->getTopicStd().empty())) {
    return;
  }

  createCameraInfoSubscription();
}

void CameraDisplay::updateCameraInfoTopic()
{
  camera_info_topic_auto_filled_ = false;

  if (initialized() && isEnabled()) {
    // Reset the cached camera info so the new topic's data is used.
    {
      std::unique_lock<std::mutex> lock(caminfo_mutex_);
      current_caminfo_.reset();
      new_caminfo_ = false;
    }
    caminfo_ok_ = false;
    caminfo_sub_.reset();
    createCameraInfoSubscription();
    context_->queueRender();
  }
}

void CameraDisplay::updatePublishImage()
{
  bool enabled = publish_image_property_->getBool();
  publish_topic_property_->setHidden(!enabled);
  publish_use_input_res_property_->setHidden(!enabled);
  image_pub_.reset();
  if (enabled) {
    updatePublishUseInputRes();
    if (!publish_topic_property_->getTopicStd().empty()) {
      auto node = rviz_ros_node_.lock()->get_raw_node();
      image_pub_ = std::make_shared<image_transport::Publisher>(
        image_transport::create_publisher(
          node.get(), publish_topic_property_->getTopicStd(), rclcpp::QoS(1).get_rmw_qos_profile()));
    }
  } else {
    publish_width_property_->setHidden(true);
    publish_height_property_->setHidden(true);
  }
}

void CameraDisplay::updatePublishUseInputRes()
{
  bool use_input = publish_use_input_res_property_->getBool();
  publish_width_property_->setHidden(use_input);
  publish_height_property_->setHidden(use_input);
  if (!use_input) {
    updatePublishSize();
  }
}

void CameraDisplay::updatePublishSize()
{
  if (publish_image_property_->getBool()) {
    render_panel_->resize(
      publish_width_property_->getInt(),
      publish_height_property_->getInt());
  }
}

void CameraDisplay::createCameraInfoSubscription()
{
  try {
    // TODO(anhosi,wjwwood): replace with abstraction for subscriptions one available

    // If the camera info topic property is empty, derive it from the image topic
    // and fill in the property so the user can see which topic is being used.
    std::string camera_info_topic;
    if (camera_info_topic_property_->getTopicStd().empty()) {
      camera_info_topic = image_transport::getCameraInfoTopic(
        getBaseTopicFromTopic(topic_property_->getTopicStd()));
      if (camera_info_topic.empty()) {
        setStatus(
          StatusLevel::Warn, CAM_INFO_STATUS,
          "Could not derive Camera Info topic from image topic.");
        return;
      }
      camera_info_topic_property_->blockSignals(true);
      camera_info_topic_property_->setString(QString::fromStdString(camera_info_topic));
      camera_info_topic_property_->blockSignals(false);
      camera_info_topic_auto_filled_ = true;
    } else {
      camera_info_topic = camera_info_topic_property_->getTopicStd();
      camera_info_topic_auto_filled_ = false;
    }

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.event_callbacks.message_lost_callback =
      [this](rclcpp::QOSMessageLostInfo & info)
      {
        std::ostringstream sstm;
        sstm << "Some messages were lost:\n>\tNumber of new lost messages: " <<
          info.total_count_change << " \n>\tTotal number of messages lost: " <<
          info.total_count;
        setStatus(StatusLevel::Warn, CAM_INFO_STATUS, QString(sstm.str().c_str()));
      };

    caminfo_sub_ = rviz_ros_node_.lock()->get_raw_node()->
      template create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        std::unique_lock<std::mutex> lock(caminfo_mutex_);
        current_caminfo_ = msg;
        new_caminfo_ = true;
      }, sub_opts);

    setStatus(StatusLevel::Ok, CAM_INFO_STATUS, "OK");
  } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
    setStatus(StatusLevel::Error, CAM_INFO_STATUS, QString("Error subscribing: ") + e.what());
  } catch (std::system_error & e) {
    setStatus(StatusLevel::Error, CAM_INFO_STATUS, QString("Error subscribing: ") + e.what());
  } catch (std::exception & e) {
    setStatus(StatusLevel::Error, CAM_INFO_STATUS, QString("Error subscribing: ") + e.what());
  }
}

void CameraDisplay::unsubscribe()
{
  ITDClass::unsubscribe();
  caminfo_sub_.reset();
  tf_filter_.reset();
}

void CameraDisplay::updateAlpha()
{
  float alpha = alpha_property_->getFloat();

  Ogre::Pass * pass = overlay_material_->getTechnique(0)->getPass(0);
  if (pass->getNumTextureUnitStates() > 0) {
    Ogre::TextureUnitState * tex_unit = pass->getTextureUnitState(0);
    tex_unit->setAlphaOperation(Ogre::LBX_MODULATE, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, alpha);
  } else {
    overlay_material_->setAmbient(Ogre::ColourValue(0.0f, 1.0f, 1.0f, alpha));
    overlay_material_->setDiffuse(Ogre::ColourValue(0.0f, 1.0f, 1.0f, alpha));
  }

  force_render_ = true;
  context_->queueRender();
}

void CameraDisplay::clear()
{
  texture_->clear();
  force_render_ = true;
  context_->queueRender();

  new_caminfo_ = false;
  current_caminfo_.reset();

  std::string camera_info_topic =
    image_transport::getCameraInfoTopic(getBaseTopicFromTopic(topic_property_->getTopicStd()));

  setStatus(
    StatusLevel::Warn, CAM_INFO_STATUS,
    "No CameraInfo received on [" + QString::fromStdString(camera_info_topic) + "]. "
    "Topic may not exist.");

  rviz_rendering::RenderWindowOgreAdapter::setOgreCameraPosition(
    render_panel_->getRenderWindow(),
    rviz_common::RenderPanel::default_camera_pose_);

  if (tf_filter_) {
    tf_filter_->clear();
  }
}

void CameraDisplay::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;
  try {
    if (texture_->update() || force_render_) {
      caminfo_ok_ = updateCamera();
      force_render_ = false;
    }
  } catch (UnsupportedImageEncoding & e) {
    setStatus(StatusLevel::Error, "Image", e.what());
  }
}

bool CameraDisplay::updateCamera()
{
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
  sensor_msgs::msg::Image::ConstSharedPtr image;
  {
    std::unique_lock<std::mutex> lock(caminfo_mutex_);

    info = current_caminfo_;
    image = texture_->getImage();
  }

  if (!image) {
    return false;
  }

  if (!info) {
    std::string camera_info_topic;
    if (camera_info_topic_property_->getStdString().empty()) {
      camera_info_topic = image_transport::getCameraInfoTopic(
        getBaseTopicFromTopic(topic_property_->getTopicStd()));
    } else {
      camera_info_topic = camera_info_topic_property_->getStdString();
    }

    setStatus(
      StatusLevel::Warn, CAM_INFO_STATUS,
      "Expecting Camera Info on topic [" + QString::fromStdString(camera_info_topic) + "]. "
      "No CameraInfo received. Topic may not exist.");
    return false;
  }

  if (!validateFloats(*info)) {
    setStatus(
      StatusLevel::Error, CAM_INFO_STATUS,
      "Contains invalid floating point values (nans or infs)");
    return false;
  }

  if (context_->getFrameManager()->getSyncMode() == rviz_common::FrameManagerIface::SyncExact) {
    if (cache_images_ == nullptr) {
      cache_images_ = std::make_shared<message_filters::Cache<sensor_msgs::msg::Image>>(
        *subscription_, 20);
    }
    rclcpp::Time rviz_time = context_->getFrameManager()->getTime();
    std::vector<sensor_msgs::msg::Image::ConstSharedPtr> interval_data = cache_images_->getInterval(
      rviz_time, rviz_time);
    if (!interval_data.empty()) {
      image = interval_data[0];
      if (timeDifferenceInExactSyncMode(image, rviz_time)) {
        setStatus(
          StatusLevel::Warn, TIME_STATUS,
          QString("Time-syncing active and no image at timestamp ") +
          QString::number(rviz_time.nanoseconds()) + ".");
        return false;
      }
    }
  } else {
    if (cache_images_ != nullptr) {
      cache_images_.reset();
    }
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  rclcpp::Time time_stamp(image->header.stamp, RCL_ROS_TIME);
  if (!context_->getFrameManager()->getTransform(
      image->header.frame_id, time_stamp, position, orientation))
  {
    setMissingTransformToFixedFrame(image->header.frame_id);
    return false;
  }
  setTransformOk();

  // convert vision (Z-forward) frame to ogre frame (Z-out)
  orientation = orientation * Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_X);

  auto dimensions = getImageDimensions(info);
  if (dimensions.height == 0.0 || dimensions.width == 0.0) {
    setStatus(
      StatusLevel::Error, CAM_INFO_STATUS,
      "Could not determine width/height of image "
      "due to malformed CameraInfo (either width or height is 0)");
    return false;
  }

  translatePosition(position, info, orientation);
  if (!rviz_common::validateFloats(position)) {
    setStatus(
      StatusLevel::Error, CAM_INFO_STATUS,
      "CameraInfo/P resulted in an invalid position calculation (nans or infs)");
    return false;
  }

  auto render_window = render_panel_->getRenderWindow();
  rviz_rendering::RenderWindowOgreAdapter::setOgreCameraPosition(render_window, position);
  rviz_rendering::RenderWindowOgreAdapter::setOgreCameraOrientation(render_window, orientation);

  Ogre::Vector2 zoom = getZoomFromInfo(info, dimensions);
  Ogre::Matrix4 proj_matrix = calculateProjectionMatrix(info, dimensions, zoom);

  rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(render_window)
  ->setCustomProjectionMatrix(true, proj_matrix);

  setStatus(StatusLevel::Ok, CAM_INFO_STATUS, "OK");

  // adjust the image rectangles to fit the zoom & aspect ratio
  Ogre::Vector4 corners = calculateScreenCorners(info, zoom);
  background_screen_rect_->setCorners(corners.x, corners.y, corners.z, corners.w);
  overlay_screen_rect_->setCorners(corners.x, corners.y, corners.z, corners.w);

  Ogre::AxisAlignedBox aabInf;
  aabInf.setInfinite();
  background_screen_rect_->setBoundingBox(aabInf);
  overlay_screen_rect_->setBoundingBox(aabInf);

  setStatus(StatusLevel::Ok, TIME_STATUS, "ok");
  setStatus(StatusLevel::Ok, CAM_INFO_STATUS, "ok");

  last_zoom_ = zoom;

  return true;
}

bool CameraDisplay::timeDifferenceInExactSyncMode(
  const sensor_msgs::msg::Image::ConstSharedPtr & image, rclcpp::Time & rviz_time) const
{
  return context_->getFrameManager()->getSyncMode() == rviz_common::FrameManagerIface::SyncExact &&
         rviz_time != rclcpp::Time(image->header.stamp, RCL_ROS_TIME);
}

ImageDimensions CameraDisplay::getImageDimensions(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info) const
{
  ImageDimensions dimensions{info->height, info->width};

  // If the image width is 0 due to a malformed caminfo, try to grab the width from the image.
  if (dimensions.width == 0) {
    RVIZ_COMMON_LOG_DEBUG_STREAM(
      "Malformed CameraInfo on camera" << qPrintable(getName()) << ", width = 0");
    dimensions.width = texture_->getWidth();
  }

  if (dimensions.height == 0) {
    RVIZ_COMMON_LOG_DEBUG_STREAM(
      "Malformed CameraInfo on camera" << qPrintable(getName()) << ", height = 0");
    dimensions.height = texture_->getHeight();
  }

  return dimensions;
}

Ogre::Vector2 CameraDisplay::getZoomFromInfo(
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info, ImageDimensions dimensions) const
{
  Ogre::Vector2 zoom;
  zoom.x = zoom_property_->getFloat();
  zoom.y = zoom.x;

  auto fx = static_cast<float>(info->p[0]);
  auto fy = static_cast<float>(info->p[5]);

  float win_width = render_panel_->width();
  float win_height = render_panel_->height();

  // Preserve aspect ratio
  if (win_width != 0 && win_height != 0) {
    float img_aspect = (dimensions.width / fx) / (dimensions.height / fy);
    float win_aspect = win_width / win_height;

    if (img_aspect > win_aspect) {
      zoom.y = zoom.y / img_aspect * win_aspect;
    } else {
      zoom.x = zoom.x / win_aspect * img_aspect;
    }
  }

  return zoom;
}

/// Add the camera's translation relative to the left camera (from P[3]);
void CameraDisplay::translatePosition(
  Ogre::Vector3 & position,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info,
  Ogre::Quaternion orientation)
{
  double fx = info->p[0];
  double fy = info->p[5];

  double tx = -1 * (info->p[3] / fx);
  Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
  position = position + (right * tx);

  double ty = -1 * (info->p[7] / fy);
  Ogre::Vector3 down = orientation * Ogre::Vector3::UNIT_Y;
  position = position + (down * ty);
}

/// calculate the projection matrix
Ogre::Matrix4 CameraDisplay::calculateProjectionMatrix(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr info,
  ImageDimensions dimensions,
  const Ogre::Vector2 & zoom) const
{
  auto cx = static_cast<float>(info->p[2]);
  auto cy = static_cast<float>(info->p[6]);

  auto fx = static_cast<float>(info->p[0]);
  auto fy = static_cast<float>(info->p[5]);

  float far_plane = far_plane_property_->getFloat();
  float near_plane = 0.01f;

  Ogre::Matrix4 proj_matrix;
  proj_matrix = Ogre::Matrix4::ZERO;

  proj_matrix[0][0] = 2.0f * fx / dimensions.width * zoom.x;
  proj_matrix[1][1] = 2.0f * fy / dimensions.height * zoom.y;

  proj_matrix[0][2] = 2.0f * (0.5f - cx / dimensions.width) * zoom.x;
  proj_matrix[1][2] = 2.0f * (cy / dimensions.height - 0.5f) * zoom.y;

  proj_matrix[2][2] = -(far_plane + near_plane) / (far_plane - near_plane);
  proj_matrix[2][3] = -2.0f * far_plane * near_plane / (far_plane - near_plane);

  proj_matrix[3][2] = -1;

  return proj_matrix;
}

void CameraDisplay::processMessage(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  texture_->addMessage(msg);
  if (publish_image_property_->getBool() && publish_use_input_res_property_->getBool()) {
    int w = static_cast<int>(msg->width);
    int h = static_cast<int>(msg->height);
    if (w > 0 && h > 0 &&
      (w != publish_width_property_->getInt() || h != publish_height_property_->getInt()))
    {
      publish_width_property_->blockSignals(true);
      publish_height_property_->blockSignals(true);
      publish_width_property_->setInt(w);
      publish_height_property_->setInt(h);
      publish_width_property_->blockSignals(false);
      publish_height_property_->blockSignals(false);
      render_panel_->resize(w, h);
    }
  }
}

void CameraDisplay::reset()
{
  ITDClass::reset();
  clear();
}

}  // namespace displays

}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::CameraDisplay, rviz_common::Display)
