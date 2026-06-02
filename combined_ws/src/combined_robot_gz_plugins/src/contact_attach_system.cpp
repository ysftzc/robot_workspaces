#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>

#include <gz/common/Console.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace combined_robot_gz_plugins
{
namespace
{
using Entity = gz::sim::Entity;

struct AttachSpec
{
  std::string parentModel;
  std::string parentLink;
  std::string childModel;
  std::string childLink{"link"};
  std::string jointName;
};

struct PendingRequest
{
  enum class Action
  {
    Attach,
    Detach,
  };

  Action action;
  AttachSpec spec;
  bool done{false};
  bool success{false};
  std::string message;
  std::mutex mutex;
  std::condition_variable cv;
};

std::string Trim(const std::string &_value)
{
  const auto begin = _value.find_first_not_of(" \t\n\r");
  if (begin == std::string::npos)
  {
    return "";
  }
  const auto end = _value.find_last_not_of(" \t\n\r");
  return _value.substr(begin, end - begin + 1);
}

std::unordered_map<std::string, std::string> ParseKeyValuePayload(
    const std::string &_payload)
{
  std::unordered_map<std::string, std::string> values;
  std::stringstream stream(_payload);
  std::string item;
  while (std::getline(stream, item, ','))
  {
    const auto pos = item.find('=');
    if (pos == std::string::npos)
    {
      continue;
    }
    auto key = Trim(item.substr(0, pos));
    auto value = Trim(item.substr(pos + 1));
    if (!key.empty())
    {
      values[key] = value;
    }
  }
  return values;
}

std::string SafeName(std::string value)
{
  for (auto &ch : value)
  {
    if (!std::isalnum(static_cast<unsigned char>(ch)) && ch != '_')
    {
      ch = '_';
    }
  }
  return value;
}

std::string JointKey(const AttachSpec &_spec)
{
  return _spec.childModel + "::" + (_spec.childLink.empty() ? "link" : _spec.childLink);
}
}  // namespace

class ContactAttachSystem
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
      const Entity &_entity,
      const std::shared_ptr<const sdf::Element> & /*_sdf*/,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager & /*_eventMgr*/) override
  {
    this->worldEntity = _entity;
    auto name = _ecm.Component<gz::sim::components::Name>(_entity);
    this->worldName = name ? name->Data() : "default";

    const auto prefix = "/world/" + this->worldName + "/contact_attach";
    const bool attachAdvertised = this->node.Advertise(
        prefix + "/attach",
        &ContactAttachSystem::OnAttach,
        this);
    const bool detachAdvertised = this->node.Advertise(
        prefix + "/detach",
        &ContactAttachSystem::OnDetach,
        this);

    if (!attachAdvertised || !detachAdvertised)
    {
      gzerr << "ContactAttachSystem failed to advertise services under ["
            << prefix << "]" << std::endl;
      return;
    }

    gzmsg << "ContactAttachSystem ready under [" << prefix << "]" << std::endl;
  }

  void PreUpdate(
      const gz::sim::UpdateInfo & /*_info*/,
      gz::sim::EntityComponentManager &_ecm) override
  {
    std::vector<std::shared_ptr<PendingRequest>> requests;
    {
      std::lock_guard<std::mutex> lock(this->queueMutex);
      requests.swap(this->queue);
    }

    for (auto &request : requests)
    {
      bool success = false;
      std::string message;
      if (request->action == PendingRequest::Action::Attach)
      {
        success = this->ProcessAttach(request->spec, _ecm, message);
      }
      else
      {
        success = this->ProcessDetach(request->spec, _ecm, message);
      }

      {
        std::lock_guard<std::mutex> lock(request->mutex);
        request->success = success;
        request->message = message;
        request->done = true;
      }
      request->cv.notify_one();
    }
  }

private:
  bool OnAttach(const gz::msgs::StringMsg &_request, gz::msgs::Boolean &_reply)
  {
    AttachSpec spec;
    std::string error;
    if (!this->ParseSpec(_request.data(), true, spec, error))
    {
      gzerr << "Contact attach request rejected: " << error << std::endl;
      _reply.set_data(false);
      return true;
    }
    _reply.set_data(this->EnqueueAndWait(PendingRequest::Action::Attach, spec));
    return true;
  }

  bool OnDetach(const gz::msgs::StringMsg &_request, gz::msgs::Boolean &_reply)
  {
    AttachSpec spec;
    std::string error;
    if (!this->ParseSpec(_request.data(), false, spec, error))
    {
      gzerr << "Contact detach request rejected: " << error << std::endl;
      _reply.set_data(false);
      return true;
    }
    _reply.set_data(this->EnqueueAndWait(PendingRequest::Action::Detach, spec));
    return true;
  }

  bool ParseSpec(
      const std::string &_payload,
      bool _requireParent,
      AttachSpec &_spec,
      std::string &_error) const
  {
    const auto values = ParseKeyValuePayload(_payload);
    auto value = [&](const std::string &key) -> std::string
    {
      const auto it = values.find(key);
      return it == values.end() ? "" : it->second;
    };

    _spec.parentModel = value("parent_model");
    _spec.parentLink = value("parent_link");
    _spec.childModel = value("child_model");
    _spec.childLink = value("child_link");
    _spec.jointName = value("joint_name");
    if (_spec.childLink.empty())
    {
      _spec.childLink = "link";
    }
    if (_spec.jointName.empty() && !_spec.childModel.empty())
    {
      _spec.jointName = "contact_attach_" + SafeName(_spec.childModel);
    }

    if (_requireParent && (_spec.parentModel.empty() || _spec.parentLink.empty()))
    {
      _error = "parent_model and parent_link are required";
      return false;
    }
    if (_spec.childModel.empty())
    {
      _error = "child_model is required";
      return false;
    }
    return true;
  }

  bool EnqueueAndWait(PendingRequest::Action _action, const AttachSpec &_spec)
  {
    auto request = std::make_shared<PendingRequest>();
    request->action = _action;
    request->spec = _spec;

    {
      std::lock_guard<std::mutex> lock(this->queueMutex);
      this->queue.push_back(request);
    }

    std::unique_lock<std::mutex> lock(request->mutex);
    const bool finished = request->cv.wait_for(
        lock,
        std::chrono::milliseconds(1800),
        [&request]() { return request->done; });

    if (!finished)
    {
      gzerr << "ContactAttachSystem request timed out for ["
            << _spec.childModel << "]" << std::endl;
      return false;
    }
    if (!request->success)
    {
      gzerr << "ContactAttachSystem request failed for ["
            << _spec.childModel << "]: " << request->message << std::endl;
    }
    return request->success;
  }

  Entity FindModel(
      const std::string &_modelName,
      const gz::sim::EntityComponentManager &_ecm) const
  {
    Entity found{gz::sim::kNullEntity};
    _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
        [&](const Entity &_entity,
            const gz::sim::components::Model *,
            const gz::sim::components::Name *_name) -> bool
        {
          if (_name->Data() == _modelName)
          {
            found = _entity;
            return false;
          }
          return true;
        });
    return found;
  }

  Entity FindLinkInModel(
      Entity _modelEntity,
      const std::string &_linkName,
      const gz::sim::EntityComponentManager &_ecm) const
  {
    if (_modelEntity == gz::sim::kNullEntity || _linkName.empty())
    {
      return gz::sim::kNullEntity;
    }

    const auto direct = gz::sim::Model(_modelEntity).LinkByName(_ecm, _linkName);
    if (direct != gz::sim::kNullEntity)
    {
      return direct;
    }

    Entity found{gz::sim::kNullEntity};
    _ecm.Each<gz::sim::components::Link, gz::sim::components::Name>(
        [&](const Entity &_entity,
            const gz::sim::components::Link *,
            const gz::sim::components::Name *_name) -> bool
        {
          if (_name->Data() != _linkName)
          {
            return true;
          }
          if (gz::sim::topLevelModel(_entity, _ecm) != _modelEntity)
          {
            return true;
          }
          found = _entity;
          return false;
        });
    return found;
  }

  bool ProcessAttach(
      const AttachSpec &_spec,
      gz::sim::EntityComponentManager &_ecm,
      std::string &_message)
  {
    const auto parentModel = this->FindModel(_spec.parentModel, _ecm);
    if (parentModel == gz::sim::kNullEntity)
    {
      _message = "parent model not found: " + _spec.parentModel;
      return false;
    }
    const auto childModel = this->FindModel(_spec.childModel, _ecm);
    if (childModel == gz::sim::kNullEntity)
    {
      _message = "child model not found: " + _spec.childModel;
      return false;
    }

    const auto parentLink = this->FindLinkInModel(parentModel, _spec.parentLink, _ecm);
    if (parentLink == gz::sim::kNullEntity)
    {
      _message = "parent link not found: " + _spec.parentModel + "::" + _spec.parentLink;
      return false;
    }
    const auto childLink = this->FindLinkInModel(childModel, _spec.childLink, _ecm);
    if (childLink == gz::sim::kNullEntity)
    {
      _message = "child link not found: " + _spec.childModel + "::" + _spec.childLink;
      return false;
    }

    const auto key = JointKey(_spec);
    this->RemoveJointForKey(key, _ecm);

    const auto jointEntity = _ecm.CreateEntity();
    if (jointEntity == gz::sim::kNullEntity)
    {
      _message = "could not create joint entity";
      return false;
    }

    gz::sim::components::DetachableJointInfo info;
    info.parentLink = parentLink;
    info.childLink = childLink;
    info.jointType = "fixed";

    _ecm.CreateComponent(jointEntity, gz::sim::components::Name(_spec.jointName));
    _ecm.CreateComponent(jointEntity, gz::sim::components::ParentEntity(parentModel));
    _ecm.CreateComponent(jointEntity, gz::sim::components::DetachableJoint(info));
    this->activeJoints[key] = jointEntity;

    gzmsg << "ContactAttachSystem fixed joint created: "
          << _spec.parentModel << "::" << _spec.parentLink << " -> "
          << _spec.childModel << "::" << _spec.childLink
          << " entity=" << jointEntity << std::endl;
    return true;
  }

  bool ProcessDetach(
      const AttachSpec &_spec,
      gz::sim::EntityComponentManager &_ecm,
      std::string &_message)
  {
    const auto key = JointKey(_spec);
    const bool removed = this->RemoveJointForKey(key, _ecm);
    if (!removed)
    {
      _message = "active joint not found for " + key;
      return false;
    }
    gzmsg << "ContactAttachSystem fixed joint removed for "
          << key << std::endl;
    return true;
  }

  bool RemoveJointForKey(
      const std::string &_key,
      gz::sim::EntityComponentManager &_ecm)
  {
    const auto it = this->activeJoints.find(_key);
    if (it == this->activeJoints.end())
    {
      return false;
    }
    const auto jointEntity = it->second;
    _ecm.RequestRemoveEntity(jointEntity, true);
    this->activeJoints.erase(it);
    return true;
  }

  Entity worldEntity{gz::sim::kNullEntity};
  std::string worldName{"default"};
  gz::transport::Node node;
  std::mutex queueMutex;
  std::vector<std::shared_ptr<PendingRequest>> queue;
  std::unordered_map<std::string, Entity> activeJoints;
};
}  // namespace combined_robot_gz_plugins

GZ_ADD_PLUGIN(
    combined_robot_gz_plugins::ContactAttachSystem,
    gz::sim::System,
    combined_robot_gz_plugins::ContactAttachSystem::ISystemConfigure,
    combined_robot_gz_plugins::ContactAttachSystem::ISystemPreUpdate)
