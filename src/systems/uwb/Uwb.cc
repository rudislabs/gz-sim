/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "Uwb.hh"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <gz/plugin/Register.hh>

#include <sdf/Element.hh>

#include <gz/common/Profiler.hh>

#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/UwbSensor.hh>

#include "gz/sim/World.hh"
#include "gz/sim/components/Uwb.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private Uwb data class.
class gz::sim::systems::UwbPrivate
{
  /// \brief A map of UWB entity to its UWB sensor.
  public: std::unordered_map<Entity,
      std::unique_ptr<sensors::UwbSensor>> entitySensorMap;

  /// \brief gz-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  /// \brief Keep list of sensors that were created during the previous
  /// `PostUpdate`, so that components can be created during the next
  /// `PreUpdate`.
  public: std::unordered_set<Entity> newSensors;

  /// True if the rendering component is initialized
  public: bool initialized = false;
  
  /// \brief Keep track of world ID, which is equivalent to the scene's
  /// root visual.
  /// Defaults to zero, which is considered invalid by Gazebo.
  public: Entity worldEntity = kNullEntity;

  /// \brief Create UWB sensors in gz-sensors
  /// \param[in] _ecm Immutable reference to ECM.
  public: void CreateSensors(const EntityComponentManager &_ecm);

  /// \brief Update UWB sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void Update(const EntityComponentManager &_ecm);

  /// \brief Create sensor
  /// \param[in] _ecm Immutable reference to ECM.
  /// \param[in] _entity Entity of the UWB
  /// \param[in] _uwb UWB component.
  /// \param[in] _parent Parent entity component.
  public: void AddSensor(
    const EntityComponentManager &_ecm,
    const Entity _entity,
    const components::Uwb *_uwb,
    const components::ParentEntity *_parent);

  /// \brief Remove UWB sensors if their entities have been removed from
  /// simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveUwbEntities(const EntityComponentManager &_ecm);
};

//////////////////////////////////////////////////
Uwb::Uwb() : System(), dataPtr(std::make_unique<UwbPrivate>())
{
}

//////////////////////////////////////////////////
Uwb::~Uwb() = default;

//////////////////////////////////////////////////
void Uwb::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("Uwb::PreUpdate");

  // Create components
  for (auto entity : this->dataPtr->newSensors)
  {
    auto it = this->dataPtr->entitySensorMap.find(entity);
    if (it == this->dataPtr->entitySensorMap.end())
    {
      gzerr << "Entity [" << entity
             << "] isn't in sensor map, this shouldn't happen." << std::endl;
      continue;
    }
    // Set topic
    _ecm.CreateComponent(entity, components::SensorTopic(it->second->Topic()));
  }
  this->dataPtr->newSensors.clear();
}

//////////////////////////////////////////////////
void Uwb::PostUpdate(const UpdateInfo &_info,
                     const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Uwb::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  this->dataPtr->CreateSensors(_ecm);

  // Only update and publish if not paused.
  if (!_info.paused)
  {
    // check to see if update is necessary
    // we only update if there is at least one sensor that needs data
    // and that sensor has subscribers.
    // note: gz-sensors does its own throttling. Here the check is mainly
    // to avoid doing work in the UwbPrivate::Update function
    bool needsUpdate = false;
    for (auto &it : this->dataPtr->entitySensorMap)
    {
      if (it.second->NextDataUpdateTime() <= _info.simTime &&
          it.second->HasConnections())
      {
        needsUpdate = true;
        break;
      }
    }
    if (!needsUpdate)
      return;

    this->dataPtr->Update(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Update measurement time
      it.second->Update(_info.simTime, false);
    }
  }

  this->dataPtr->RemoveUwbEntities(_ecm);
}

//////////////////////////////////////////////////
void UwbPrivate::AddSensor(
  const EntityComponentManager &_ecm,
  const Entity _entity,
  const components::Uwb *_uwb,
  const components::ParentEntity *_parent)
{
  // Get the world acceleration (defined in world frame)
/*   auto gravity = _ecm.Component<components::Gravity>(worldEntity);
  if (nullptr == gravity)
  {
    gzerr << "World missing gravity." << std::endl;
    return;
  } */

  // create sensor
  std::string sensorScopedName =
      removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
  sdf::Sensor data = _uwb->Data();
  data.SetName(sensorScopedName);
  // check topic
  if (data.Topic().empty())
  {
    std::string topic = scopedName(_entity, _ecm) + "/uwb";
    data.SetTopic(topic);
  }
  std::unique_ptr<sensors::UwbSensor> sensor =
      this->sensorFactory.CreateSensor<
      sensors::UwbSensor>(data);
  if (nullptr == sensor)
  {
    gzerr << "Failed to create sensor [" << sensorScopedName << "]"
           << std::endl;
    return;
  }

  // set sensor parent
  std::string parentName = _ecm.Component<components::Name>(
      _parent->Data())->Data();
  sensor->SetParent(parentName);

  // set gravity - assume it remains fixed
  // sensor->SetGravity(gravity->Data());

  // Get initial pose of sensor and set the reference z pos
  // The WorldPose component was just created and so it's empty
  // We'll compute the world pose manually here
/*   math::Pose3d p = worldPose(_entity, _ecm);
  sensor->SetOrientationReference(p.Rot()); */

  // Get world frame orientation and heading.
  // If <orientation_reference_frame> includes a named
  // frame like NED, that must be supplied to the UWB sensor,
  // otherwise orientations are reported w.r.t to the initial
  // orientation.
/*   if (data.Element()->HasElement("uwb")) {
    auto uwbElementPtr = data.Element()->GetElement("uwb");
    if (uwbElementPtr->HasElement("orientation_reference_frame")) {
      double heading = 0.0;

      gz::sim::World world(worldEntity);
      if (world.SphericalCoordinates(_ecm))
      {
        auto sphericalCoordinates = world.SphericalCoordinates(_ecm).value();
        heading = sphericalCoordinates.HeadingOffset().Radian();
      }

      sensor->SetWorldFrameOrientation(math::Quaterniond(0, 0, heading),
        gz::sensors::WorldFrameEnumType::ENU);
    }
  } */

  // Set whether orientation is enabled
/*   if (data.UwbSensor())
  {
    sensor->SetOrientationEnabled(
        data.UwbSensor()->OrientationEnabled());
  } */

  this->entitySensorMap.insert(
      std::make_pair(_entity, std::move(sensor)));
  this->newSensors.insert(_entity);
}

//////////////////////////////////////////////////
void UwbPrivate::CreateSensors(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("UwbPrivate::CreateUwbEntities");
  // Get World Entity
  if (kNullEntity == this->worldEntity)
    this->worldEntity = _ecm.EntityByComponents(components::World());
  if (kNullEntity == this->worldEntity)
  {
    gzerr << "Missing world entity." << std::endl;
    return;
  }

  if (!this->initialized)
  {
    // Create UWBs
    _ecm.Each<components::Uwb, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Uwb *_uwb,
          const components::ParentEntity *_parent)->bool
        {
          this->AddSensor(_ecm, _entity, _uwb, _parent);
          return true;
        });
      this->initialized = true;
  }
  else
  {
    // Create UWBs
    _ecm.EachNew<components::Uwb, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Uwb *_uwb,
          const components::ParentEntity *_parent)->bool
        {
          this->AddSensor(_ecm, _entity, _uwb, _parent);
          return true;
      });
  }
}

//////////////////////////////////////////////////
void UwbPrivate::Update(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("UwbPrivate::Update");
  _ecm.Each<components::Uwb,
            components::WorldPose>(
    [&](const Entity &_entity,
        const components::Uwb * /*_uwb*/,
        const components::WorldPose *_worldPose)->bool
      {
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          const auto &uwbWorldPose = _worldPose->Data();
          it->second->SetWorldPose(uwbWorldPose);
         }
        else
        {
          gzerr << "Failed to update UWB: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

//////////////////////////////////////////////////
void UwbPrivate::RemoveUwbEntities(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("UwbPrivate::RemoveUwbEntities");
  _ecm.EachRemoved<components::Uwb>(
    [&](const Entity &_entity,
        const components::Uwb *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          gzerr << "Internal error, missing UWB sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

GZ_ADD_PLUGIN(Uwb, System,
  Uwb::ISystemPreUpdate,
  Uwb::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(Uwb, "gz::sim::systems::Uwb")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(Uwb, "ignition::gazebo::systems::Uwb")
