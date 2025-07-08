// normwind_plugin.cpp - Implementation (Fixed)
#include "normwind_plugin.hpp"

#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

using namespace normwind_plugin;

class NormWindPlugin::Implementation
{
  /// \brief Model entity
  public: gz::sim::Entity modelEntity;

  /// \brief Link entity
  public: gz::sim::Entity linkEntity;

  /// \brief Transport node
  public: gz::transport::Node node;

  /// \brief Wind force publisher
  public: gz::transport::Node::Publisher windForcePub;

  /// \brief Wind speed publisher
  public: gz::transport::Node::Publisher windSpeedPub;

  /// \brief Model name
  public: std::string modelName;

  /// \brief Link name
  public: std::string linkName;

  /// \brief Namespace
  public: std::string ns;

  /// \brief Frame ID
  public: std::string frameId;

  /// \brief Wind force topic
  public: std::string windForceTopic;

  /// \brief Wind speed topic
  public: std::string windSpeedTopic;

  /// \brief Wind parameters
  public: double windSpeedMean{0.0};
  public: int windTurbulenceLevel{0};
  public: gz::math::Vector3d windDirectionMean{1.0, 0.0, 0.0};

  /// \brief Random number generation
  public: std::default_random_engine randomGen;
  public: std::normal_distribution<double> gaussianRandomNumber;

  /// \brief Custom wind field parameters
  public: bool useCustomStaticWindField{false};
  public: float minX{0.0}, minY{0.0};
  public: int nX{0}, nY{0};
  public: float resX{1.0}, resY{1.0};
  public: std::vector<float> verticalSpacingFactors;
  public: std::vector<float> bottomZ, topZ;
  public: std::vector<float> u, v, w;

  /// \brief Read custom wind field from file
  public: void ReadCustomWindField(const std::string &customWindFieldPath);
};

/////////////////////////////////////////////////
NormWindPlugin::NormWindPlugin()
  : impl(std::make_unique<Implementation>())
{
}

/////////////////////////////////////////////////
NormWindPlugin::~NormWindPlugin()
{
}

/////////////////////////////////////////////////
void NormWindPlugin::Configure(const gz::sim::Entity &_entity,
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

  if (_sdf->HasElement("windForcePubTopic"))
    this->impl->windForceTopic = _sdf->Get<std::string>("windForcePubTopic");
  else
    this->impl->windForceTopic = "wind_force";

  if (_sdf->HasElement("windSpeedPubTopic"))
    this->impl->windSpeedTopic = _sdf->Get<std::string>("windSpeedPubTopic");
  else
    this->impl->windSpeedTopic = "wind_speed";

  if (_sdf->HasElement("windSpeedMean"))
    this->impl->windSpeedMean = _sdf->Get<double>("windSpeedMean");

  if (_sdf->HasElement("windTurbulenceLevel"))
    this->impl->windTurbulenceLevel = _sdf->Get<int>("windTurbulenceLevel");

  if (_sdf->HasElement("windDirectionMean"))
    this->impl->windDirectionMean = _sdf->Get<gz::math::Vector3d>("windDirectionMean");

  // Find the link entity
  auto linkComp = _ecm.Component<gz::sim::components::Name>(_entity);
  if (linkComp)
  {
    gz::sim::Entity linkEntity = gz::sim::Model(_entity).LinkByName(_ecm, this->impl->linkName);
    if (linkEntity != gz::sim::kNullEntity)
    {
      this->impl->linkEntity = linkEntity;
    }
    else
    {
      gzerr << "Link with name[" << this->impl->linkName << "] not found. "
            << "The NormWindPlugin will not generate forces" << std::endl;
    }
  }

  // Setup transport
  std::string topic = this->impl->ns + "/" + this->impl->windForceTopic;
  this->impl->windForcePub = this->impl->node.Advertise<gz::msgs::Wrench>(topic);

  topic = this->impl->ns + "/" + this->impl->windSpeedTopic;
  this->impl->windSpeedPub = this->impl->node.Advertise<gz::msgs::Vector3d>(topic);

  // Initialize random number generator
  this->impl->randomGen.seed(std::chrono::steady_clock::now().time_since_epoch().count());
  this->impl->gaussianRandomNumber = std::normal_distribution<double>(0.0, 1.0);

  // Check for custom wind field
  if (_sdf->HasElement("useCustomStaticWindField"))
  {
    this->impl->useCustomStaticWindField = _sdf->Get<bool>("useCustomStaticWindField");
    if (this->impl->useCustomStaticWindField && _sdf->HasElement("customWindFieldPath"))
    {
      std::string customWindFieldPath = _sdf->Get<std::string>("customWindFieldPath");
      this->impl->ReadCustomWindField(customWindFieldPath);
    }
  }
}

/////////////////////////////////////////////////
void NormWindPlugin::PreUpdate(const gz::sim::UpdateInfo &/*_info*/,
                               gz::sim::EntityComponentManager &_ecm)
{
  if (this->impl->linkEntity == gz::sim::kNullEntity)
    return;

  // Get link pose
  auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->impl->linkEntity);
  if (!poseComp)
    return;

  gz::math::Pose3d linkPose = poseComp->Data();

  // Calculate wind speed
  gz::math::Vector3d windVel;
  if (this->impl->useCustomStaticWindField)
  {
    // For now, use simple wind model
    windVel = this->impl->windDirectionMean * this->impl->windSpeedMean;
  }
  else
  {
    // Simple wind model with turbulence
    double windSpeed = this->impl->windSpeedMean;
    if (this->impl->windTurbulenceLevel > 0)
    {
      double turbulence = this->impl->gaussianRandomNumber(this->impl->randomGen) 
                         * this->impl->windTurbulenceLevel * 0.1;
      windSpeed += turbulence;
    }
    windVel = this->impl->windDirectionMean * windSpeed;
  }

  // Create proper wrench message
  gz::msgs::Wrench wrenchMsg;
  wrenchMsg.mutable_force()->set_x(windVel.X());
  wrenchMsg.mutable_force()->set_y(windVel.Y());
  wrenchMsg.mutable_force()->set_z(windVel.Z());
  wrenchMsg.mutable_torque()->set_x(0.0);
  wrenchMsg.mutable_torque()->set_y(0.0);
  wrenchMsg.mutable_torque()->set_z(0.0);

  // Set the external wrench component
  auto wrenchComp = _ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(this->impl->linkEntity);
  if (!wrenchComp)
  {
    _ecm.CreateComponent(this->impl->linkEntity, 
                        gz::sim::components::ExternalWorldWrenchCmd(wrenchMsg));
  }
  else
  {
    wrenchComp->Data() = wrenchMsg;
  }

  // Publish wind speed
  gz::msgs::Vector3d windSpeedMsg;
  windSpeedMsg.set_x(windVel.X());
  windSpeedMsg.set_y(windVel.Y());
  windSpeedMsg.set_z(windVel.Z());
  this->impl->windSpeedPub.Publish(windSpeedMsg);
}

/////////////////////////////////////////////////
void NormWindPlugin::Implementation::ReadCustomWindField(const std::string &customWindFieldPath)
{
  // Implementation of custom wind field reading
  gzdbg << "Reading custom wind field from: " << customWindFieldPath << std::endl;
  // File reading logic would go here
}

// Plugin registration
GZ_ADD_PLUGIN(NormWindPlugin,
              gz::sim::System,
              NormWindPlugin::ISystemConfigure,
              NormWindPlugin::ISystemPreUpdate)