// finliftdrag_plugin.hpp
#ifndef BLIMP_GAZEBO_PLUGIN_FINLIFTDRAG_PLUGIN_HPP_
#define BLIMP_GAZEBO_PLUGIN_FINLIFTDRAG_PLUGIN_HPP_

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
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/wrench.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/float.pb.h>
#include <gz/plugin/Register.hh>
#include <sdf/sdf.hh>

namespace finliftdrag_plugin
{
  /// \brief A plugin that simulates lift and drag forces on fins
  class FinLiftDragPlugin : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemPreUpdate
  {
    /// \brief Constructor
    public: FinLiftDragPlugin();

    /// \brief Destructor
    public: ~FinLiftDragPlugin() override;

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

#endif // BLIMP_GAZEBO_PLUGIN_FINLIFTDRAG_PLUGIN_HPP_