// dynamicvolume_plugin.hpp
#ifndef BLIMP_GAZEBO_PLUGIN_DYNAMICVOLUME_PLUGIN_HPP_
#define BLIMP_GAZEBO_PLUGIN_DYNAMICVOLUME_PLUGIN_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/wrench.pb.h>
#include <gz/msgs/float.pb.h>
#include <gz/plugin/Register.hh>
#include <sdf/sdf.hh>

namespace dynamicvolume_plugin
{
  /// \brief A class for storing volume properties of a link
  class VolumeProperties
  {
    /// \brief Default constructor
    public: VolumeProperties() : volume(0) {}

    /// \brief Center of volume in link frame
    public: gz::math::Vector3d cov;

    /// \brief Volume of this link
    public: double volume;
  };

  /// \brief A plugin that simulates dynamic volume and buoyancy forces
  class DynamicVolumePlugin : public gz::sim::System,
                              public gz::sim::ISystemConfigure,
                              public gz::sim::ISystemPreUpdate
  {
    /// \brief Constructor
    public: DynamicVolumePlugin();

    /// \brief Destructor
    public: ~DynamicVolumePlugin() override;

    /// \brief Configure the plugin
    public: void Configure(const gz::sim::Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          gz::sim::EntityComponentManager &_ecm,
                          gz::sim::EventManager &_eventMgr) override;

    /// \brief Called every PreUpdate cycle
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private implementation
    private: class Implementation;
    private: std::unique_ptr<Implementation> impl;
  };
}

#endif // BLIMP_GAZEBO_PLUGIN_DYNAMICVOLUME_PLUGIN_HPP_