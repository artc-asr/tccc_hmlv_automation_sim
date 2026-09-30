// conveyor_belt.cc — a conveyor with a filling station, for gz-sim Fortress.
//
// Fortress has no belt that carries objects, so this world system does it
// kinematically: on <start_topic> it takes every model whose name starts with
// <model_prefix> and whose origin rests on the belt (inside <min>..<max>), and
// slides them together along <direction> at <speed> for <travel> metres
// (Model::SetWorldPoseCmd each step, velocity zeroed). At the end it "fills"
// them: each one's link mass becomes <fill_mass> (inertia scaled to match), and
// an ignition.msgs.Empty goes out on <done_topic>.
//
// The fill changes the Inertial component. Fortress's physics keeps the mass the
// body was created with, so it is read by whatever looks at the component — the
// KinematicGrasp plugin, which puts the held object's weight on the arm — not by
// the contact dynamics of a can resting on the belt.
//
// With <fill_rgba> set, the fill also recolours the model's visuals (all but
// those whose name contains "cap"): the Material component is updated and a
// VisualCmd is issued, the same way UserCommands' visual_config does it, which
// is what the renderers (GUI and the Sensors system) apply at runtime.

#include <atomic>
#include <string>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/gazebo/components/Material.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/VisualCmd.hh>
#include <ignition/math/Color.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/msgs/empty.pb.h>
#include <ignition/msgs/visual.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace moz1_sim_gazebo
{
using namespace ignition;
using namespace gazebo;

class ConveyorBelt : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
  void Configure(const Entity &, const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &, EventManager &) override
  {
    this->prefix = _sdf->Get<std::string>("model_prefix", "").first;
    this->min = _sdf->Get<math::Vector3d>("min");
    this->max = _sdf->Get<math::Vector3d>("max");
    this->dir = _sdf->Get<math::Vector3d>("direction", math::Vector3d::UnitY).first.Normalized();
    this->speed = _sdf->Get<double>("speed", 0.2).first;
    this->travel = _sdf->Get<double>("travel", 1.0).first;
    this->fillMass = _sdf->Get<double>("fill_mass", 0.0).first;
    if (_sdf->HasElement("fill_rgba")) {
      this->fillColor = _sdf->Get<math::Color>("fill_rgba");
      this->recolor = true;
    }
    this->node.Subscribe(_sdf->Get<std::string>("start_topic", "/conveyor/start").first,
                         &ConveyorBelt::OnStart, this);
    this->donePub = this->node.Advertise<msgs::Empty>(
      _sdf->Get<std::string>("done_topic", "/conveyor/done").first);
  }

  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
  {
    if (_info.paused) {
      return;
    }
    if (this->startRequest.exchange(false) && this->cargo.empty()) {
      this->Load(_ecm);
    }
    if (this->cargo.empty()) {
      return;
    }
    const double dt = std::chrono::duration<double>(_info.dt).count();
    this->s = std::min(this->travel, this->s + this->speed * dt);
    for (auto & [model, start] : this->cargo) {
      math::Pose3d p = start;
      p.Pos() += this->dir * this->s;
      model.SetWorldPoseCmd(_ecm, p);
      Link link(model.CanonicalLink(_ecm));
      link.SetLinearVelocity(_ecm, math::Vector3d::Zero);
      link.SetAngularVelocity(_ecm, math::Vector3d::Zero);
    }
    if (this->s >= this->travel) {
      for (auto & [model, start] : this->cargo) {
        this->Fill(model, _ecm);
      }
      ignmsg << "ConveyorBelt: delivered " << this->cargo.size() << " model(s)\n";
      this->cargo.clear();
      this->donePub.Publish(msgs::Empty());
    }
  }

private:
  void Load(EntityComponentManager &_ecm)
  {
    _ecm.Each<components::Model, components::Name>(
      [&](const Entity & _e, const components::Model *, const components::Name * _name) {
        if (_name->Data().rfind(this->prefix, 0) != 0) {
          return true;
        }
        const math::Pose3d pose = worldPose(_e, _ecm);
        const auto & p = pose.Pos();
        if (p.X() >= min.X() && p.X() <= max.X() && p.Y() >= min.Y() && p.Y() <= max.Y() &&
            p.Z() >= min.Z() && p.Z() <= max.Z()) {
          this->cargo.emplace_back(Model(_e), pose);
          ignmsg << "ConveyorBelt: carrying [" << _name->Data() << "]\n";
        }
        return true;
      });
    this->s = 0.0;
    if (this->cargo.empty()) {
      ignwarn << "ConveyorBelt: started with nothing on the belt\n";
      this->donePub.Publish(msgs::Empty());
    }
  }

  void Fill(Model & _model, EntityComponentManager &_ecm)
  {
    if (this->fillMass <= 0.0) {
      return;
    }
    const Entity link = _model.CanonicalLink(_ecm);
    auto *inertial = _ecm.Component<components::Inertial>(link);
    if (inertial == nullptr) {
      return;
    }
    math::Inertiald in = inertial->Data();
    math::MassMatrix3d mm = in.MassMatrix();
    const double ratio = this->fillMass / mm.Mass();
    mm.SetMass(this->fillMass);
    mm.SetDiagonalMoments(mm.DiagonalMoments() * ratio);
    mm.SetOffDiagonalMoments(mm.OffDiagonalMoments() * ratio);
    in.SetMassMatrix(mm);
    inertial->Data() = in;
    _ecm.SetChanged(link, components::Inertial::typeId, ComponentState::OneTimeChange);
    ignmsg << "ConveyorBelt: filled [" << _model.Name(_ecm) << "] to " << this->fillMass
           << " kg\n";
    if (this->recolor) {
      this->Recolor(link, _ecm);
    }
  }

  void Recolor(Entity _link, EntityComponentManager &_ecm)
  {
    for (const Entity v : _ecm.ChildrenByComponents(_link, components::Visual())) {
      const auto *name = _ecm.Component<components::Name>(v);
      if (name == nullptr || name->Data().find("cap") != std::string::npos) {
        continue;
      }
      if (auto *mat = _ecm.Component<components::Material>(v)) {
        sdf::Material m = mat->Data();
        m.SetAmbient(this->fillColor);
        m.SetDiffuse(this->fillColor);
        mat->Data() = m;
        _ecm.SetChanged(v, components::Material::typeId, ComponentState::OneTimeChange);
      }
      msgs::Visual msg;
      msg.set_id(v);
      msg.set_name(name->Data());
      msgs::Set(msg.mutable_material()->mutable_ambient(), this->fillColor);
      msgs::Set(msg.mutable_material()->mutable_diffuse(), this->fillColor);
      if (auto *cmd = _ecm.Component<components::VisualCmd>(v)) {
        cmd->Data() = msg;
        _ecm.SetChanged(v, components::VisualCmd::typeId, ComponentState::OneTimeChange);
      } else {
        _ecm.CreateComponent(v, components::VisualCmd(msg));
      }
    }
  }

  void OnStart(const msgs::Empty &) { this->startRequest = true; }

  std::string prefix;
  math::Vector3d min, max, dir;
  double speed{0.2}, travel{1.0}, fillMass{0.0}, s{0.0};
  math::Color fillColor;
  bool recolor{false};
  std::vector<std::pair<Model, math::Pose3d>> cargo;
  transport::Node node;
  transport::Node::Publisher donePub;
  std::atomic<bool> startRequest{false};
};
}  // namespace moz1_sim_gazebo

IGNITION_ADD_PLUGIN(moz1_sim_gazebo::ConveyorBelt, ignition::gazebo::System,
                    moz1_sim_gazebo::ConveyorBelt::ISystemConfigure,
                    moz1_sim_gazebo::ConveyorBelt::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(moz1_sim_gazebo::ConveyorBelt, "moz1_sim_gazebo::ConveyorBelt")
