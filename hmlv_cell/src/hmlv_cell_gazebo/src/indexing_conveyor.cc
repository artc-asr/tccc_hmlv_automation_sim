// indexing_conveyor.cc — a belt that indexes one station per cycle, with a filling
// station in the middle, for gz-sim Fortress (the duo cell, launch/duo.launch.py).
//
// moz1_sim_gazebo's ConveyorBelt carries the cans put on it all the way along and
// fills whatever it carried. Here the belt holds cans at three stations
// (load, filler, unload) and every cycle, on <start_topic>:
//
//   1. FILL: every model whose name starts with <model_prefix> and whose origin is
//      inside the filling zone (<fill_min>..<fill_max>) is filled: its link mass
//      becomes <fill_mass> (inertia scaled to match) and, with <fill_rgba>, its
//      visuals are recoloured. Then <fill_time> s of dwell (the filling).
//   2. INDEX: every such model resting on the belt (<min>..<max>) slides along
//      <direction> by <step> m at <speed> m/s, kinematically
//      (Model::SetWorldPoseCmd each step, velocity zeroed), all together.
//   3. an ignition.msgs.Empty on <done_topic>.
//
// So the pair under the filler leaves filled, and the pair set down at the load
// station arrives at the filler still empty — to be filled on the next cycle.
// The mass change is read by whatever looks at the Inertial component (the
// KinematicGrasp plugin puts the held object's weight on the arm); see
// moz1_sim_gazebo/src/conveyor_belt.cc for that and for the recolouring.
#include <ignition/common/Console.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/gazebo/components/Material.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/VisualCmd.hh>
#include <ignition/math/Color.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/empty.pb.h>
#include <ignition/msgs/Utility.hh>
#include <ignition/msgs/visual.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <sdf/Material.hh>

#include <atomic>
#include <string>
#include <utility>
#include <vector>

namespace hmlv_cell_gazebo
{
using namespace ignition;
using namespace gazebo;

class IndexingConveyor : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
  void Configure(const Entity &, const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &, EventManager &) override
  {
    this->prefix = _sdf->Get<std::string>("model_prefix", "").first;
    this->min = _sdf->Get<math::Vector3d>("min");
    this->max = _sdf->Get<math::Vector3d>("max");
    this->fillMin = _sdf->Get<math::Vector3d>("fill_min");
    this->fillMax = _sdf->Get<math::Vector3d>("fill_max");
    this->dir = _sdf->Get<math::Vector3d>("direction", math::Vector3d::UnitY).first.Normalized();
    this->speed = _sdf->Get<double>("speed", 0.25).first;
    this->step = _sdf->Get<double>("step", 1.0).first;
    this->fillMass = _sdf->Get<double>("fill_mass", 0.0).first;
    this->fillTime = _sdf->Get<double>("fill_time", 2.0).first;
    if (_sdf->HasElement("fill_rgba")) {
      this->fillColor = _sdf->Get<math::Color>("fill_rgba");
      this->recolor = true;
    }
    this->node.Subscribe(_sdf->Get<std::string>("start_topic", "/conveyor/start").first,
                         &IndexingConveyor::OnStart, this);
    this->donePub = this->node.Advertise<msgs::Empty>(
      _sdf->Get<std::string>("done_topic", "/conveyor/done").first);
  }

  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
  {
    if (_info.paused) {
      return;
    }
    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (this->state == State::Idle) {
      if (!this->startRequest.exchange(false)) {
        return;
      }
      int filled = 0;
      this->ForEachCan(_ecm, [&](const Entity _e, const math::Pose3d &_p) {
        if (Inside(_p.Pos(), this->fillMin, this->fillMax)) {
          Model m(_e);
          this->Fill(m, _ecm);
          ++filled;
        }
      });
      ignmsg << "IndexingConveyor: filling " << filled << " can(s)\n";
      this->clock = 0.0;
      this->state = State::Filling;
      return;
    }
    if (this->state == State::Filling) {
      this->clock += dt;
      if (this->clock < this->fillTime) {
        return;
      }
      this->cargo.clear();
      this->ForEachCan(_ecm, [&](const Entity _e, const math::Pose3d &_p) {
        if (Inside(_p.Pos(), this->min, this->max)) {
          this->cargo.emplace_back(Model(_e), _p);
          ignmsg << "IndexingConveyor: moving ["
                 << _ecm.Component<components::Name>(_e)->Data() << "]\n";
        }
      });
      this->s = 0.0;
      this->state = State::Moving;
      return;
    }
    // State::Moving
    this->s = std::min(this->step, this->s + this->speed * dt);
    for (auto & [model, start] : this->cargo) {
      math::Pose3d p = start;
      p.Pos() += this->dir * this->s;
      model.SetWorldPoseCmd(_ecm, p);
      Link link(model.CanonicalLink(_ecm));
      link.SetLinearVelocity(_ecm, math::Vector3d::Zero);
      link.SetAngularVelocity(_ecm, math::Vector3d::Zero);
    }
    if (this->s >= this->step) {
      ignmsg << "IndexingConveyor: indexed " << this->cargo.size() << " can(s) by "
             << this->step << " m\n";
      this->cargo.clear();
      this->state = State::Idle;
      this->donePub.Publish(msgs::Empty());
    }
  }

private:
  enum class State { Idle, Filling, Moving };

  static bool Inside(const math::Vector3d &_p, const math::Vector3d &_lo,
                     const math::Vector3d &_hi)
  {
    return _p.X() >= _lo.X() && _p.X() <= _hi.X() && _p.Y() >= _lo.Y() &&
           _p.Y() <= _hi.Y() && _p.Z() >= _lo.Z() && _p.Z() <= _hi.Z();
  }

  template <typename F>
  void ForEachCan(EntityComponentManager &_ecm, F _f)
  {
    _ecm.Each<components::Model, components::Name>(
      [&](const Entity &_e, const components::Model *, const components::Name *_name) {
        if (_name->Data().rfind(this->prefix, 0) == 0) {
          _f(_e, worldPose(_e, _ecm));
        }
        return true;
      });
  }

  void Fill(Model &_model, EntityComponentManager &_ecm)
  {
    const Entity link = _model.CanonicalLink(_ecm);
    if (this->fillMass > 0.0) {
      if (auto *inertial = _ecm.Component<components::Inertial>(link)) {
        math::Inertiald in = inertial->Data();
        math::MassMatrix3d mm = in.MassMatrix();
        const double ratio = this->fillMass / mm.Mass();
        mm.SetMass(this->fillMass);
        mm.SetDiagonalMoments(mm.DiagonalMoments() * ratio);
        mm.SetOffDiagonalMoments(mm.OffDiagonalMoments() * ratio);
        in.SetMassMatrix(mm);
        inertial->Data() = in;
        _ecm.SetChanged(link, components::Inertial::typeId, ComponentState::OneTimeChange);
      }
    }
    ignmsg << "IndexingConveyor: filled [" << _model.Name(_ecm) << "] to " << this->fillMass
           << " kg\n";
    if (!this->recolor) {
      return;
    }
    for (const Entity v : _ecm.ChildrenByComponents(link, components::Visual())) {
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
  math::Vector3d min, max, fillMin, fillMax, dir;
  double speed{0.25}, step{1.0}, fillMass{0.0}, fillTime{2.0}, s{0.0}, clock{0.0};
  math::Color fillColor;
  bool recolor{false};
  State state{State::Idle};
  std::vector<std::pair<Model, math::Pose3d>> cargo;
  transport::Node node;
  transport::Node::Publisher donePub;
  std::atomic<bool> startRequest{false};
};
}  // namespace hmlv_cell_gazebo

IGNITION_ADD_PLUGIN(hmlv_cell_gazebo::IndexingConveyor, ignition::gazebo::System,
                    hmlv_cell_gazebo::IndexingConveyor::ISystemConfigure,
                    hmlv_cell_gazebo::IndexingConveyor::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(hmlv_cell_gazebo::IndexingConveyor,
                          "hmlv_cell_gazebo::IndexingConveyor")
