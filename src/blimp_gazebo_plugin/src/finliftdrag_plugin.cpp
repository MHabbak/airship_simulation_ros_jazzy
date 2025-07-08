// finliftdrag_plugin.cpp - Implementation (Fixed)
#include "finliftdrag_plugin.hpp"

#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

using namespace finliftdrag_plugin;

class FinLiftDragPlugin::Implementation
{
  /// \brief Model entity
  public: gz::sim::Entity modelEntity;

  /// \brief Link entity
  public: gz::sim::Entity linkEntity;

  /// \brief Control joint entity
  public: gz::sim::Entity controlJointEntity;

  /// \brief Transport node
  public: gz::transport::Node node;

  /// \brief Wind speed subscriber
  public: gz::transport::Node::Publisher airspeedPub;

  /// \brief Model name
  public: std::string modelName;

  /// \brief Link name
  public: std::string linkName;

  /// \brief Control joint name
  public: std::string controlJointName;

  /// \brief Namespace
  public: std::string ns;

  /// \brief Topic names
  public: std::string windSpeedSubTopic;
  public: std::string airspeedPubTopic;
  public: bool publishAirspeed{false};

  /// \brief Center of pressure in link frame
  public: gz::math::Vector3d cp{0, 0, 0};

  /// \brief Forward direction in link frame
  public: gz::math::Vector3d forward{1, 0, 0};

  /// \brief Upward direction in link frame
  public: gz::math::Vector3d upward{0, 0, 1};

  /// \brief Wing area
  public: double area{1.0};

  /// \brief Air density
  public: double rho{1.2041};

  /// \brief Angle of attack offset
  public: double alpha0{0.0};

  /// \brief Stall angle
  public: double alphaStall{0.5 * M_PI};

  /// \brief Lift coefficient slope
  public: double cla{1.0};

  /// \brief Drag coefficient
  public: double cda{0.01};

  /// \brief Moment coefficient
  public: double cma{0.01};

  /// \brief Stall coefficients
  public: double claStall{0.0};
  public: double cdaStall{1.0};
  public: double cmaStall{0.0};

  /// \brief Sweep angle
  public: double sweep{0.0};

  /// \brief Velocity at stall
  public: double velocityStall{0.0};

  /// \brief Radial symmetry flag
  public: bool radialSymmetry{false};

  /// \brief Control joint parameters
  public: double controlJointRadToCL{4.0};

  /// \brief Wind speed
  public: gz::math::Vector3d windSpeed{0, 0, 0};

  /// \brief Wind speed callback
  public: void WindSpeedCallback(const gz::msgs::Vector3d &_msg);
};

/////////////////////////////////////////////////
FinLiftDragPlugin::FinLiftDragPlugin()
  : impl(std::make_unique<Implementation>())
{
}

/////////////////////////////////////////////////
FinLiftDragPlugin::~FinLiftDragPlugin()
{
}

/////////////////////////////////////////////////
void FinLiftDragPlugin::Configure(const gz::sim::Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> &_sdf,
                                  gz::sim::EntityComponentManager &_ecm,
                                  gz::sim::EventManager &/*_eventMgr*/)
{
  this->impl->modelEntity = _entity;
  this->impl->modelName = gz::sim::scopedName(_entity, _ecm);

  // Get parameters from SDF
  if (_sdf->HasElement("robotNamespace"))
    this->impl->ns = _sdf->Get<std::string>("robotNamespace");

  if (_sdf->HasElement("link_name"))
    this->impl->linkName = _sdf->Get<std::string>("link_name");

  if (_sdf->HasElement("control_joint_name"))
    this->impl->controlJointName = _sdf->Get<std::string>("control_joint_name");

  if (_sdf->HasElement("windSpeedSubTopic"))
    this->impl->windSpeedSubTopic = _sdf->Get<std::string>("windSpeedSubTopic");
  else
    this->impl->windSpeedSubTopic = "wind_speed";

  if (_sdf->HasElement("airspeedPubTopic"))
  {
    this->impl->airspeedPubTopic = _sdf->Get<std::string>("airspeedPubTopic");
    this->impl->publishAirspeed = true;
  }

  // Get aerodynamic parameters
  if (_sdf->HasElement("cp"))
    this->impl->cp = _sdf->Get<gz::math::Vector3d>("cp");

  if (_sdf->HasElement("forward"))
    this->impl->forward = _sdf->Get<gz::math::Vector3d>("forward");

  if (_sdf->HasElement("upward"))
    this->impl->upward = _sdf->Get<gz::math::Vector3d>("upward");

  if (_sdf->HasElement("area"))
    this->impl->area = _sdf->Get<double>("area");

  if (_sdf->HasElement("rho"))
    this->impl->rho = _sdf->Get<double>("rho");

  if (_sdf->HasElement("alpha_0"))
    this->impl->alpha0 = _sdf->Get<double>("alpha_0");

  if (_sdf->HasElement("alpha_stall"))
    this->impl->alphaStall = _sdf->Get<double>("alpha_stall");

  if (_sdf->HasElement("cla"))
    this->impl->cla = _sdf->Get<double>("cla");

  if (_sdf->HasElement("cda"))
    this->impl->cda = _sdf->Get<double>("cda");

  if (_sdf->HasElement("cma"))
    this->impl->cma = _sdf->Get<double>("cma");

  if (_sdf->HasElement("cla_stall"))
    this->impl->claStall = _sdf->Get<double>("cla_stall");

  if (_sdf->HasElement("cda_stall"))
    this->impl->cdaStall = _sdf->Get<double>("cda_stall");

  if (_sdf->HasElement("cma_stall"))
    this->impl->cmaStall = _sdf->Get<double>("cma_stall");

  if (_sdf->HasElement("sweep"))
    this->impl->sweep = _sdf->Get<double>("sweep");

  if (_sdf->HasElement("velocity_stall"))
    this->impl->velocityStall = _sdf->Get<double>("velocity_stall");

  if (_sdf->HasElement("radial_symmetry"))
    this->impl->radialSymmetry = _sdf->Get<bool>("radial_symmetry");

  if (_sdf->HasElement("control_joint_rad_to_cl"))
    this->impl->controlJointRadToCL = _sdf->Get<double>("control_joint_rad_to_cl");

  // Find the link entity
  gz::sim::Entity linkEntity = gz::sim::Model(_entity).LinkByName(_ecm, this->impl->linkName);
  if (linkEntity != gz::sim::kNullEntity)
  {
    this->impl->linkEntity = linkEntity;
  }
  else
  {
    gzerr << "Link with name[" << this->impl->linkName << "] not found. "
          << "The FinLiftDragPlugin will not generate forces" << std::endl;
  }

  // Find the control joint entity if specified
  if (!this->impl->controlJointName.empty())
  {
    gz::sim::Entity controlJointEntity = gz::sim::Model(_entity).JointByName(_ecm, this->impl->controlJointName);
    if (controlJointEntity != gz::sim::kNullEntity)
    {
      this->impl->controlJointEntity = controlJointEntity;
    }
    else
    {
      gzerr << "Joint with name[" << this->impl->controlJointName << "] not found." << std::endl;
    }
  }

  // Setup transport
  std::string topic = this->impl->ns + "/" + this->impl->windSpeedSubTopic;
  this->impl->node.Subscribe(topic, &FinLiftDragPlugin::Implementation::WindSpeedCallback, this->impl.get());

  if (this->impl->publishAirspeed)
  {
    topic = this->impl->ns + "/" + this->impl->airspeedPubTopic;
    this->impl->airspeedPub = this->impl->node.Advertise<gz::msgs::Float>(topic);
  }
}

/////////////////////////////////////////////////
void FinLiftDragPlugin::PreUpdate(const gz::sim::UpdateInfo &/*_info*/,
                                  gz::sim::EntityComponentManager &_ecm)
{
  if (this->impl->linkEntity == gz::sim::kNullEntity)
    return;

  // Get link pose and velocity
  auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->impl->linkEntity);
  auto velComp = _ecm.Component<gz::sim::components::LinearVelocity>(this->impl->linkEntity);
  
  if (!poseComp || !velComp)
    return;

  gz::math::Pose3d linkPose = poseComp->Data();
  gz::math::Vector3d linkVel = velComp->Data();

  // Get velocity at center of pressure
  gz::math::Vector3d vel = linkVel - this->impl->windSpeed;
  
  if (vel.Length() <= 0.01)
    return;

  // Get forward and upward vectors in inertial frame
  gz::math::Vector3d forwardI = linkPose.Rot().RotateVector(this->impl->forward);
  gz::math::Vector3d upwardI;

  if (this->impl->radialSymmetry)
  {
    // Use inflow velocity to determine upward direction
    upwardI = vel - (vel.Dot(forwardI) * forwardI);
    upwardI.Normalize();
  }
  else
  {
    upwardI = linkPose.Rot().RotateVector(this->impl->upward);
  }

  // Calculate angle of attack
  double alpha = atan2(vel.Dot(upwardI), vel.Dot(forwardI)) + this->impl->alpha0;

  // Get control joint angle if available
  double controlAngle = 0.0;
  if (this->impl->controlJointEntity != gz::sim::kNullEntity)
  {
    auto jointPosComp = _ecm.Component<gz::sim::components::JointPosition>(this->impl->controlJointEntity);
    if (jointPosComp && !jointPosComp->Data().empty())
    {
      controlAngle = jointPosComp->Data()[0];
    }
  }

  // Calculate lift and drag coefficients
  double cl, cd, cm;
  if (std::abs(alpha) < this->impl->alphaStall)
  {
    cl = this->impl->cla * alpha + this->impl->controlJointRadToCL * controlAngle;
    cd = this->impl->cda;
    cm = this->impl->cma * alpha;
  }
  else
  {
    cl = this->impl->claStall;
    cd = this->impl->cdaStall;
    cm = this->impl->cmaStall;
  }

  // Calculate dynamic pressure
  double q = 0.5 * this->impl->rho * vel.Length() * vel.Length() * this->impl->area;

  // Calculate forces
  gz::math::Vector3d liftForce = cl * q * upwardI;
  gz::math::Vector3d dragForce = -cd * q * vel.Normalized();
  gz::math::Vector3d totalForce = liftForce + dragForce;

  // Calculate moment
  gz::math::Vector3d moment = cm * q * (forwardI.Cross(upwardI));

  // Create proper wrench message
  gz::msgs::Wrench wrenchMsg;
  wrenchMsg.mutable_force()->set_x(totalForce.X());
  wrenchMsg.mutable_force()->set_y(totalForce.Y());
  wrenchMsg.mutable_force()->set_z(totalForce.Z());
  wrenchMsg.mutable_torque()->set_x(moment.X());
  wrenchMsg.mutable_torque()->set_y(moment.Y());
  wrenchMsg.mutable_torque()->set_z(moment.Z());

  // Apply forces to link
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

  // Publish airspeed if requested
  if (this->impl->publishAirspeed)
  {
    gz::msgs::Float airspeedMsg;
    airspeedMsg.set_data(vel.Length());
    this->impl->airspeedPub.Publish(airspeedMsg);
  }
}

/////////////////////////////////////////////////
void FinLiftDragPlugin::Implementation::WindSpeedCallback(const gz::msgs::Vector3d &_msg)
{
  this->windSpeed.X() = _msg.x();
  this->windSpeed.Y() = _msg.y();
  this->windSpeed.Z() = _msg.z();
}

// Plugin registration
GZ_ADD_PLUGIN(FinLiftDragPlugin,
              gz::sim::System,
              FinLiftDragPlugin::ISystemConfigure,
              FinLiftDragPlugin::ISystemPreUpdate)