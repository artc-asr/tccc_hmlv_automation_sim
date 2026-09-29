// kinematic_grasp.cc — rigid sim-only grasp for gz-sim Fortress.
//
// A drop-in for ignition::gazebo::systems::DetachableJoint (same parameters,
// same attach/detach topics), for grasping objects that are heavy relative to
// the hand. Fortress's dartsim joins two MODELS with a soft
// dart::constraint::WeldJointConstraint; with a 4 kg jerry can hanging off a
// 0.26 kg gripper link on a velocity-controlled arm that constraint sometimes
// holds to the millimetre and sometimes lets the can swing 20 cm or flings the
// arm, run to run. This system instead:
//
//   * on attach, records the object's pose relative to the parent link;
//   * every step while attached, puts the object back at that relative pose
//     (Model::SetWorldPoseCmd) and zeroes its velocity — a rigid carry;
//   * applies the object's weight to the parent link (Link::AddWorldWrench, at
//     the object's centre of mass), so the arm still carries the real load.
//
// Contacts of the carried object are still simulated each step, so release
// onto a surface is physical. Parameters (under <plugin>):
//   <parent_link>   link of the model this plugin is attached to
//   <child_model>   model to carry (its canonical link is used)
//   <attach_topic>  ignition.msgs.Empty — attach at the current relative pose
//   <detach_topic>  ignition.msgs.Empty — release

#include <atomic>
#include <string>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/empty.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace moz1_sim_gazebo
{
using namespace ignition;
using namespace gazebo;

class KinematicGrasp : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
  void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm, EventManager &) override
  {
    this->model = Model(_entity);
    if (!this->model.Valid(_ecm)) {
      ignerr << "KinematicGrasp must be attached to a model\n";
      return;
    }
    this->parentLinkName = _sdf->Get<std::string>("parent_link");
    this->childModelName = _sdf->Get<std::string>("child_model");
    const auto attach = _sdf->Get<std::string>("attach_topic");
    const auto detach = _sdf->Get<std::string>("detach_topic");
    if (this->parentLinkName.empty() || this->childModelName.empty() || attach.empty() ||
        detach.empty()) {
      ignerr << "KinematicGrasp needs <parent_link>, <child_model>, <attach_topic> and "
                "<detach_topic>\n";
      return;
    }
    this->node.Subscribe(attach, &KinematicGrasp::OnAttach, this);
    this->node.Subscribe(detach, &KinematicGrasp::OnDetach, this);
    this->configured = true;
  }

  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
  {
    if (!this->configured || _info.paused) {
      return;
    }
    if (!this->Resolve(_ecm)) {
      return;
    }
    if (this->detachRequest.exchange(false) && this->attached) {
      this->attached = false;
      ignmsg << "KinematicGrasp: released [" << this->childModelName << "]\n";
    }
    if (this->attachRequest.exchange(false) && !this->attached) {
      this->relPose = worldPose(this->parentLink.Entity(), _ecm).Inverse() *
                      worldPose(this->childModel.Entity(), _ecm);
      this->attached = true;
      ignmsg << "KinematicGrasp: holding [" << this->childModelName << "] ("
             << this->mass << " kg) at " << this->relPose << " from ["
             << this->parentLinkName << "]\n";
    }
    if (!this->attached) {
      return;
    }

    const math::Pose3d parent = worldPose(this->parentLink.Entity(), _ecm);
    const math::Pose3d pose = parent * this->relPose;
    this->childModel.SetWorldPoseCmd(_ecm, pose);
    this->childLink.SetLinearVelocity(_ecm, math::Vector3d::Zero);
    this->childLink.SetAngularVelocity(_ecm, math::Vector3d::Zero);

    // the object's weight, on the hand, at the object's centre of mass
    const math::Vector3d com = pose.CoordPositionAdd(this->comOffset);
    const math::Vector3d force(0.0, 0.0, -this->mass * 9.80665);
    this->parentLink.AddWorldWrench(_ecm, force, (com - parent.Pos()).Cross(force));
  }

private:
  bool Resolve(EntityComponentManager &_ecm)
  {
    if (this->parentLink.Entity() == kNullEntity) {
      this->parentLink = Link(this->model.LinkByName(_ecm, this->parentLinkName));
      if (this->parentLink.Entity() == kNullEntity) {
        return false;
      }
      this->parentLink.EnableVelocityChecks(_ecm, true);
    }
    if (this->childModel.Entity() == kNullEntity) {
      const Entity e = _ecm.EntityByComponents(components::Model(),
                                               components::Name(this->childModelName));
      if (e == kNullEntity) {
        return false;
      }
      this->childModel = Model(e);
      this->childLink = Link(this->childModel.CanonicalLink(_ecm));
      const auto *inertial = _ecm.Component<components::Inertial>(this->childLink.Entity());
      if (inertial != nullptr) {
        this->mass = inertial->Data().MassMatrix().Mass();
        this->comOffset = inertial->Data().Pose().Pos();
      }
    }
    return true;
  }

  void OnAttach(const msgs::Empty &) { this->attachRequest = true; }
  void OnDetach(const msgs::Empty &) { this->detachRequest = true; }

  Model model{kNullEntity};
  Model childModel{kNullEntity};
  Link parentLink{kNullEntity};
  Link childLink{kNullEntity};
  std::string parentLinkName;
  std::string childModelName;
  transport::Node node;
  std::atomic<bool> attachRequest{false};
  std::atomic<bool> detachRequest{false};
  bool configured{false};
  bool attached{false};
  math::Pose3d relPose;
  math::Vector3d comOffset;
  double mass{0.0};
};
}  // namespace moz1_sim_gazebo

IGNITION_ADD_PLUGIN(moz1_sim_gazebo::KinematicGrasp, ignition::gazebo::System,
                    moz1_sim_gazebo::KinematicGrasp::ISystemConfigure,
                    moz1_sim_gazebo::KinematicGrasp::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(moz1_sim_gazebo::KinematicGrasp, "moz1_sim_gazebo::KinematicGrasp")
