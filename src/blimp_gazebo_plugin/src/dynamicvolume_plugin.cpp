// dynamicvolume_plugin.cpp - Implementation (Fixed const-correctness)
#include "dynamicvolume_plugin.hpp"

#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

using namespace dynamicvolume_plugin;

class DynamicVolumePlugin::Implementation
{
  /// \brief Model entity
  public: gz::sim::Entity modelEntity;

  /// \brief Link entity
  public: gz::sim::Entity linkEntity;

  /// \brief Transport node
  public: gz::transport::Node node;

  /// \brief Dynamic volume publisher
  public: gz::transport::Node::Publisher dynamicVolumePub;

  /// \brief Model name
  public: std::string modelName;

  /// \brief Link name
  public: std::string linkName;

  /// \brief Namespace
  public: std::string ns;

  /// \brief Frame ID
  public: std::string frameId;

  /// \brief Topic names
  public: std::string dynamicVolumeTopic;
  public: std::string heliumMassTopic;

  /// \brief Fluid density (kg/m^3)
  public: double fluidDensity{1000.0};

  /// \brief Reference altitude (m)
  public: double refAlt{0.0};

  /// \brief Current helium mass (kg)
  public: double heliumMassKG{1.0};

  /// \brief Volume properties for each link
  public: std::map<gz::sim::Entity, VolumeProperties> volumePropsMap;

  /// \brief Gravity vector
  public: gz::math::Vector3d gravity{0, 0, -9.81};

  /// \brief Helium mass callback
  public: void HeliumMassCallback(const gz::msgs::Float &_msg);

  /// \brief Calculate volume from collision shapes
  public: double CalculateVolume(const gz::sim::Entity &/*_linkEntity*/,
                                gz::sim::EntityComponentManager &/*_ecm*/);

  /// \brief Calculate center of volume from collision shapes
  public: gz::math::Vector3d CalculateCenterOfVolume(const gz::sim::Entity &/*_linkEntity*/,
                                                    gz::sim::EntityComponentManager &/*_ecm*/);
};

/////////////////////////////////////////////////
DynamicVolumePlugin::DynamicVolumePlugin()
  : impl(std::make_unique<Implementation>())
{
}

/////////////////////////////////////////////////
DynamicVolumePlugin::~DynamicVolumePlugin()
{
}

/////////////////////////////////////////////////
void DynamicVolumePlugin::Configure(const gz::sim::Entity &_entity,
                                    const std::shared_ptr<const sdf::Element> &_sdf,
                                    gz::sim::EntityComponentManager &_ecm,
                                    gz::sim::EventManager &/*_eventMgr*/)
{
  this->impl->modelEntity = _entity;
  this->impl->modelName = gz::sim::scopedName(_entity, _ecm);

  // Get parameters from SDF
  if (_sdf->HasElement("robotNamespace"))
    this->impl->ns = _sdf->Get<std::string>("robotNamespace");

  if (_sdf->HasElement("linkName"))
    this->impl->linkName = _sdf->Get<std::string>("linkName");

  if (_sdf->HasElement("frameId"))
    this->impl->frameId = _sdf->Get<std::string>("frameId");

  if (_sdf->HasElement("dynamicVolumeTopic"))
    this->impl->dynamicVolumeTopic = _sdf->Get<std::string>("dynamicVolumeTopic");
  else
    this->impl->dynamicVolumeTopic = "dynamic_volume";

  if (_sdf->HasElement("heliumMassTopic"))
    this->impl->heliumMassTopic = _sdf->Get<std::string>("heliumMassTopic");
  else
    this->impl->heliumMassTopic = "helium_mass";

  if (_sdf->HasElement("fluidDensity"))
    this->impl->fluidDensity = _sdf->Get<double>("fluidDensity");

  if (_sdf->HasElement("refAlt"))
    this->impl->refAlt = _sdf->Get<double>("refAlt");

  if (_sdf->HasElement("heliumMassKG"))
    this->impl->heliumMassKG = _sdf->Get<double>("heliumMassKG");

  // Find the link entity
  gz::sim::Entity linkEntity = gz::sim::Model(_entity).LinkByName(_ecm, this->impl->linkName);
  if (linkEntity != gz::sim::kNullEntity)
  {
    this->impl->linkEntity = linkEntity;
  }
  else
  {
    gzerr << "Link with name[" << this->impl->linkName << "] not found. "
          << "The DynamicVolumePlugin will not generate forces" << std::endl;
  }

  // Process volume properties from SDF - FIXED const-correctness
  if (_sdf->HasElement("link"))
  {
    // Create a mutable copy of the SDF element to work with
    sdf::ElementPtr mutableSdf = _sdf->Clone();
    auto linkElem = mutableSdf->GetElement("link");
    
    while (linkElem)
    {
      if (linkElem->HasAttribute("name"))
      {
        std::string linkName = linkElem->Get<std::string>("name");
        gz::sim::Entity linkEntity = gz::sim::Model(_entity).LinkByName(_ecm, linkName);
        
        if (linkEntity != gz::sim::kNullEntity)
        {
          VolumeProperties props;
          
          if (linkElem->HasElement("center_of_volume"))
          {
            props.cov = linkElem->Get<gz::math::Vector3d>("center_of_volume");
          }
          else
          {
            props.cov = this->impl->CalculateCenterOfVolume(linkEntity, _ecm);
          }

          if (linkElem->HasElement("volume"))
          {
            props.volume = linkElem->Get<double>("volume");
          }
          else
          {
            props.volume = this->impl->CalculateVolume(linkEntity, _ecm);
          }

          this->impl->volumePropsMap[linkEntity] = props;
        }
        else
        {
          gzwarn << "Link with name[" << linkName << "] not found for volume properties" << std::endl;
        }
      }
      
      linkElem = linkElem->GetNextElement("link");
    }
  }

  // If no volume properties specified, use the main link
  if (this->impl->volumePropsMap.empty() && this->impl->linkEntity != gz::sim::kNullEntity)
  {
    VolumeProperties props;
    props.cov = this->impl->CalculateCenterOfVolume(this->impl->linkEntity, _ecm);
    props.volume = this->impl->CalculateVolume(this->impl->linkEntity, _ecm);
    this->impl->volumePropsMap[this->impl->linkEntity] = props;
  }

  // Setup transport
  std::string topic = this->impl->ns + "/" + this->impl->dynamicVolumeTopic;
  this->impl->dynamicVolumePub = this->impl->node.Advertise<gz::msgs::Float>(topic);

  topic = this->impl->ns + "/" + this->impl->heliumMassTopic;
  this->impl->node.Subscribe(topic, &DynamicVolumePlugin::Implementation::HeliumMassCallback, this->impl.get());
}

/////////////////////////////////////////////////
void DynamicVolumePlugin::PreUpdate(const gz::sim::UpdateInfo &/*_info*/,
                                    gz::sim::EntityComponentManager &_ecm)
{
  // Calculate and apply buoyancy forces for each link
  for (auto &volProps : this->impl->volumePropsMap)
  {
    gz::sim::Entity linkEntity = volProps.first;
    VolumeProperties &props = volProps.second;

    // Get link pose
    auto poseComp = _ecm.Component<gz::sim::components::Pose>(linkEntity);
    if (!poseComp)
      continue;

    gz::math::Pose3d linkPose = poseComp->Data();

    // Calculate dynamic volume based on altitude and helium mass
    double altitude = linkPose.Pos().Z() - this->impl->refAlt;
    double pressureRatio = std::exp(-altitude / 8400.0); // Simplified atmospheric model
    double dynamicVolume = props.volume * (1.0 / pressureRatio) * (this->impl->heliumMassKG / 1.0);

    // Calculate buoyancy force
    double buoyancyMagnitude = this->impl->fluidDensity * dynamicVolume * (-this->impl->gravity.Z());
    gz::math::Vector3d buoyancyForce = -this->impl->gravity.Normalized() * buoyancyMagnitude;

    // Create proper wrench message
    gz::msgs::Wrench wrenchMsg;
    wrenchMsg.mutable_force()->set_x(buoyancyForce.X());
    wrenchMsg.mutable_force()->set_y(buoyancyForce.Y());
    wrenchMsg.mutable_force()->set_z(buoyancyForce.Z());
    wrenchMsg.mutable_torque()->set_x(0.0);
    wrenchMsg.mutable_torque()->set_y(0.0);
    wrenchMsg.mutable_torque()->set_z(0.0);
    
    // Set the external wrench component
    auto wrenchComp = _ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(linkEntity);
    if (!wrenchComp)
    {
      _ecm.CreateComponent(linkEntity, 
                          gz::sim::components::ExternalWorldWrenchCmd(wrenchMsg));
    }
    else
    {
      wrenchComp->Data() = wrenchMsg;
    }

    // Publish dynamic volume
    gz::msgs::Float dynamicVolumeMsg;
    dynamicVolumeMsg.set_data(dynamicVolume);
    this->impl->dynamicVolumePub.Publish(dynamicVolumeMsg);
  }
}

/////////////////////////////////////////////////
void DynamicVolumePlugin::Implementation::HeliumMassCallback(const gz::msgs::Float &_msg)
{
  this->heliumMassKG = _msg.data();
}

/////////////////////////////////////////////////
double DynamicVolumePlugin::Implementation::CalculateVolume(const gz::sim::Entity &/*_linkEntity*/,
                                                           gz::sim::EntityComponentManager &/*_ecm*/)
{
  // Simplified volume calculation
  double volume = 1.0; // Default volume
  
  gzwarn << "Using default volume calculation. "
         << "For accurate simulation, specify volume in SDF." << std::endl;
  
  return volume;
}

/////////////////////////////////////////////////
gz::math::Vector3d DynamicVolumePlugin::Implementation::CalculateCenterOfVolume(const gz::sim::Entity &/*_linkEntity*/,
                                                                               gz::sim::EntityComponentManager &/*_ecm*/)
{
  // Simplified center of volume calculation
  gz::math::Vector3d centerOfVolume(0, 0, 0);
  
  gzwarn << "Using default center of volume calculation. "
         << "For accurate simulation, specify center_of_volume in SDF." << std::endl;
  
  return centerOfVolume;
}

// Plugin registration
GZ_ADD_PLUGIN(DynamicVolumePlugin,
              gz::sim::System,
              DynamicVolumePlugin::ISystemConfigure,
              DynamicVolumePlugin::ISystemPreUpdate)