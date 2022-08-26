/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <string>
#include <vector>

#include <gz/plugin/Register.hh>

#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Transparency.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>

#include <sdf/Visual.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>

#include <gz/common/Material.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>

#include <gz/math/Matrix4.hh>

#include <gz/common/ColladaExporter.hh>

#include "ColladaWorldExporter.hh"

using namespace gz;
using namespace gz::sim;
using namespace systems;


class gz::sim::systems::ColladaWorldExporterPrivate
{
  // Default constructor
  public: ColladaWorldExporterPrivate() = default;

  /// \brief Has the world already been exported?.
  private: bool exported{false};

  /// \brief Exports the world to a mesh.
  /// \param[_ecm] _ecm Mutable reference to the EntityComponentManager.
  public: void Export(const EntityComponentManager &_ecm)
  {
    if (this->exported) return;

    common::Mesh worldMesh;
    std::vector<math::Matrix4d> subMeshMatrix;

    _ecm.Each<components::World, components::Name>(
      [&](const Entity /*& _entity*/,
        const components::World *,
        const components::Name * _name)->bool
    {
      worldMesh.SetName(_name->Data());
      return true;
    });

    _ecm.Each<components::Visual,
            components::Name,
            components::Geometry,
            components::Transparency>(
    [&](const Entity &_entity,
        const components::Visual *,
        const components::Name *_name,
        const components::Geometry *_geom,
        const components::Transparency *_transparency)->bool
    {
      std::string name = _name->Data().empty() ? std::to_string(_entity) :
          _name->Data();

      math::Pose3d worldPose = gz::sim::worldPose(_entity, _ecm);

      common::MaterialPtr mat = std::make_shared<common::Material>();
      auto material = _ecm.Component<components::Material>(_entity);
      if (material != nullptr)
      {
        mat->SetDiffuse(material->Data().Diffuse());
        mat->SetAmbient(material->Data().Ambient());
        mat->SetEmissive(material->Data().Emissive());
        mat->SetSpecular(material->Data().Specular());
      }
      mat->SetTransparency(_transparency->Data());

      const common::Mesh *mesh;
      std::weak_ptr<common::SubMesh> subm;
      math::Vector3d scale;
      math::Matrix4d matrix(worldPose);
      common::MeshManager *meshManager =
          common::MeshManager::Instance();

      auto addSubmeshFunc = [&](int i) {
          subm = worldMesh.AddSubMesh(
              *mesh->SubMeshByIndex(0).lock().get());
          subm.lock()->SetMaterialIndex(i);
          subm.lock()->Scale(scale);
          subMeshMatrix.push_back(matrix);
      };

      if (_geom->Data().Type() == sdf::GeometryType::BOX)
      {
        if (meshManager->HasMesh("unit_box"))
        {
          mesh = meshManager->MeshByName("unit_box");
          scale = _geom->Data().BoxShape()->Size();
          int i = worldMesh.AddMaterial(mat);

          addSubmeshFunc(i);
        }
      }
      else if (_geom->Data().Type() == sdf::GeometryType::CYLINDER)
      {
        if (meshManager->HasMesh("unit_cylinder"))
        {
          mesh = meshManager->MeshByName("unit_cylinder");
          scale.X() = _geom->Data().CylinderShape()->Radius() * 2;
          scale.Y() = scale.X();
          scale.Z() = _geom->Data().CylinderShape()->Length();

          int i = worldMesh.AddMaterial(mat);

          addSubmeshFunc(i);
        }
      }
      else if (_geom->Data().Type() == sdf::GeometryType::PLANE)
      {
        if (meshManager->HasMesh("unit_plane"))
        {
          // Create a rotation for the plane mesh to account
          // for the normal vector.
          mesh = meshManager->MeshByName("unit_plane");

          scale.X() = _geom->Data().PlaneShape()->Size().X();
          scale.Y() = _geom->Data().PlaneShape()->Size().Y();

          // // The rotation is the angle between the +z(0,0,1) vector and the
          // // normal, which are both expressed in the local (Visual) frame.
          math::Vector3d normal = _geom->Data().PlaneShape()->Normal();
          math::Quaterniond normalRot;
          normalRot.From2Axes(math::Vector3d::UnitZ, normal.Normalized());
          worldPose.Rot() = worldPose.Rot() * normalRot;

          matrix = math::Matrix4d(worldPose);

          int i = worldMesh.AddMaterial(mat);
          addSubmeshFunc(i);
        }
      }
      else if (_geom->Data().Type() == sdf::GeometryType::SPHERE)
      {
        if (meshManager->HasMesh("unit_sphere"))
        {
          mesh = meshManager->MeshByName("unit_sphere");

          scale.X() = _geom->Data().SphereShape()->Radius() * 2;
          scale.Y() = scale.X();
          scale.Z() = scale.X();

          int i = worldMesh.AddMaterial(mat);

          addSubmeshFunc(i);
        }
      }
      else if (_geom->Data().Type() == sdf::GeometryType::MESH)
      {
        auto fullPath = asFullPath(_geom->Data().MeshShape()->Uri(),
            _geom->Data().MeshShape()->FilePath());

        if (fullPath.empty())
        {
          ignerr << "Mesh geometry missing uri" << std::endl;
          return true;
        }
        mesh = meshManager->Load(fullPath);

        if (!mesh) {
          ignerr << "mesh not found!" << std::endl;
          return true;
        }

        for (unsigned int k = 0; k < mesh->SubMeshCount(); k++)
        {
          auto subMeshLock = mesh->SubMeshByIndex(k).lock();
          int j = subMeshLock->MaterialIndex();

          int i = 0;
          if (j != -1)
          {
            i = worldMesh.IndexOfMaterial(mesh->MaterialByIndex(j).get());
            if (i < 0)
            {
              i = worldMesh.AddMaterial(mesh->MaterialByIndex(j));
            }
          }
          else
          {
            i = worldMesh.AddMaterial(mat);
          }

          scale = _geom->Data().MeshShape()->Scale();

          addSubmeshFunc(i);
        }
      }
      else
      {
        ignwarn << "Unsupported geometry type" << std::endl;
      }

      return true;
    });

    common::ColladaExporter exporter;
    exporter.Export(&worldMesh, "./" + worldMesh.Name(), true,
                    subMeshMatrix);
    ignmsg << "The world has been exported into the "
           << "./" + worldMesh.Name() << " directory." << std::endl;
    this->exported = true;
  }
};

/////////////////////////////////////////////////
ColladaWorldExporter::ColladaWorldExporter() : System(),
    dataPtr(std::make_unique<ColladaWorldExporterPrivate>())
{
}

/////////////////////////////////////////////////
ColladaWorldExporter::~ColladaWorldExporter() = default;

/////////////////////////////////////////////////
void ColladaWorldExporter::PostUpdate(const UpdateInfo & /*_info*/,
    const EntityComponentManager &_ecm)
{
  this->dataPtr->Export(_ecm);
}

IGNITION_ADD_PLUGIN(ColladaWorldExporter,
                    System,
                    ColladaWorldExporter::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ColladaWorldExporter,
                          "sim::systems::ColladaWorldExporter")
