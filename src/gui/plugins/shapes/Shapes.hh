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

#ifndef GZ_SIM_GUI_SHAPES_HH_
#define GZ_SIM_GUI_SHAPES_HH_

#include <memory>

#include <gz/gui/Plugin.hh>

namespace gz
{
namespace sim
{
  class ShapesPrivate;

  /// \brief Provides buttons for adding a box, sphere, or cylinder
  /// to the scene
  class Shapes : public gz::gui::Plugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: Shapes();

    /// \brief Destructor
    public: ~Shapes() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    /// \brief Callback in Qt thread when mode changes.
    /// \param[in] _mode New transform mode
    public slots: void OnMode(const QString &_mode);

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<ShapesPrivate> dataPtr;
  };
}
}

#endif
