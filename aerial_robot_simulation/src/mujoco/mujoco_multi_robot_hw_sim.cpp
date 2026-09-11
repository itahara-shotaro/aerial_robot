#include <aerial_robot_simulation/mujoco/mujoco_aerial_robot_hw_sim.h>
#include <controller_manager/controller_manager.h>
#include <memory>
#include <set>
#include <stdexcept>

namespace mujoco_ros_control
{
// One physics world, with an independent ros_control manager and spinal
// controller for each robot. The runner's outer controller manager stays empty.
class MultiAerialRobotHWSim : public RobotHWSim
{
public:
  bool init(const std::string&, ros::NodeHandle nh, mjModel* model, mjData* data) override
  {
    XmlRpc::XmlRpcValue robots;
    if (!nh.getParam("simulation/robots", robots) ||
        robots.getType() != XmlRpc::XmlRpcValue::TypeArray || robots.size() == 0)
      throw std::runtime_error("simulation/robots must be a nonempty list of robot namespaces");

    std::set<std::string> namespaces, prefixes;
    for (int i = 0; i < robots.size(); ++i)
      {
        std::string ns = static_cast<std::string>(robots[i]);
        if (ns.empty() || ns[0] != '/')
          throw std::runtime_error("Multi-robot namespaces must be absolute");
        ros::NodeHandle robot_nh(ns);
        if (robot_nh.getNamespace() == nh.getNamespace() ||
            !namespaces.insert(robot_nh.getNamespace()).second)
          throw std::runtime_error("Duplicate robot or simulator namespace: " + ns);
        std::string prefix;
        if (!robot_nh.getParam("simulation/model_prefix", prefix) || prefix.empty())
          throw std::runtime_error("Missing simulation/model_prefix for " + ns);
        for (const auto& other : prefixes)
          if (prefix.compare(0, other.size(), other) == 0 || other.compare(0, prefix.size(), prefix) == 0)
            throw std::runtime_error("Overlapping MuJoCo model prefixes: " + prefix + " and " + other);
        prefixes.insert(prefix);
        auto hw = std::make_unique<AerialRobotHWSim>();
        if (!hw->init(ns, robot_nh, model, data))
          throw std::runtime_error("Failed to initialize MuJoCo hardware for " + ns);
        managers_.emplace_back(new controller_manager::ControllerManager(hw.get(), robot_nh));
        robots_.push_back(std::move(hw));
        ROS_INFO_STREAM("MuJoCo robot " << ns << " owns model prefix " << prefix);
      }
    return true;
  }

  void read(const ros::Time& time, const ros::Duration& period) override
  {
    for (auto& robot : robots_) robot->read(time, period);
  }

  void write(const ros::Time& time, const ros::Duration& period) override
  {
    for (auto& manager : managers_) manager->update(time, period);
    for (auto& robot : robots_) robot->write(time, period);
  }

private:
  // Destroy controller managers before the hardware they reference.
  std::vector<std::unique_ptr<AerialRobotHWSim>> robots_;
  std::vector<std::unique_ptr<controller_manager::ControllerManager>> managers_;
};
}
PLUGINLIB_EXPORT_CLASS(mujoco_ros_control::MultiAerialRobotHWSim, mujoco_ros_control::RobotHWSim)
