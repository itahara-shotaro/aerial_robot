#include <aerial_robot_simulation/mujoco/mujoco_aerial_robot_hw_sim.h>
#include <aerial_robot_simulation/mujoco/mujoco_default_aerial_robot_hw_sim.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include <unistd.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>

using mujoco_ros::control::AerialRobotHWSim;
using mujoco_ros::control::DefaultAerialRobotHWSim;
using transmission_interface::TransmissionInfo;

void check(bool ok, const std::string& what)
{
  if (!ok) throw std::runtime_error(what);
}
void near(double actual, double expected, const std::string& what)
{
  check(std::abs(actual - expected) < 1e-6, what + ": " + std::to_string(actual));
}

class Probe : public AerialRobotHWSim
{
 public:
  int sensor(int i) const { return sensor_ids_[i]; }
  bool ignoresActuators() const { return ignore_actuators_; }
  void seedDrift()
  {
    ground_truth_rot_curr_drift_[2] = 0.1;
    ground_truth_vel_curr_drift_[0] = 0.5;
    ground_truth_angular_curr_drift_[0] = 0.4;
  }
};

TransmissionInfo rotor(const std::string& name)
{
  TransmissionInfo t;
  t.name_ = name + "_trans";
  transmission_interface::JointInfo j;
  j.name_ = name;
  t.joints_.push_back(j);
  transmission_interface::ActuatorInfo a;
  a.name_ = name;
  a.hardware_interfaces_.push_back("hardware_interface/RotorInterface");
  t.actuators_.push_back(a);
  return t;
}

std::string robotBody(const std::string& p, const std::string& pos)
{
  return "<body name='" + p + "base' pos='" + pos + "'><freejoint name='" + p +
         "root_joint'/><geom type='sphere' size='.1' mass='1'/><site name='" + p + "fc'/><site name='" + p +
         "rotor1' pos='.1 0 0'/><site name='" + p + "rotor2' pos='-.1 0 0'/></body>";
}

std::string robotActuators(const std::string& p)
{
  return "<motor name='" + p + "rotor1' site='" + p +
         "rotor1' gear='0 0 1 0 0 0'/>"
         "<motor name='" +
         p + "rotor2' site='" + p + "rotor2' gear='0 0 1 0 0 0'/>";
}
std::string robotSensors(const std::string& p)
{
  return "<accelerometer name='" + p + "acc' site='" + p + "fc'/><gyro name='" + p + "gyro' site='" + p +
         "fc'/><magnetometer name='" + p + "mag' site='" + p + "fc'/>";
}

TEST(MujocoHardware, PrefixedIsolationAndValidation)
{
  try
  {
    const std::string file = "/tmp/mujoco3-hardware-isolation-" + std::to_string(getpid()) + ".xml";
    std::ofstream(file) << "<mujoco><worldbody>" << robotBody("gimbalrotor1/", "1 2 3")
                        << robotBody("gimbalrotor2/", "4 5 6") << "</worldbody><actuator>"
                        << robotActuators("gimbalrotor1/") << robotActuators("gimbalrotor2/")
                        << "</actuator><sensor>" << robotSensors("gimbalrotor1/")
                        << robotSensors("gimbalrotor2/") << "</sensor></mujoco>";
    char error[1024] = {};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_loadXML(file.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
    std::remove(file.c_str());
    mjModel* m = model.get();
    check(m, error);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(m), mj_deleteData);
    mjData* d = data.get();
    mj_forward(m, d);
    urdf::Model urdf;
    check(urdf.initString("<robot name='test'><link name='base'/></robot>"), "URDF parse");
    ros::NodeHandle a("/isolation/a"), b("/isolation/b");
    a.setParam("simulation/model_prefix", "gimbalrotor1/");
    b.setParam("simulation/model_prefix", "gimbalrotor2/");
    // Deliberately reverse transmission order: spinal outputs must remain rotor1, rotor2.
    const std::vector<TransmissionInfo> transmissions{rotor("rotor2"), rotor("rotor1")};
    Probe ahw, bhw;
    check(ahw.InitSim(m, d, nullptr, a.getNamespace(), a, &urdf, transmissions, true), "A init");
    check(bhw.InitSim(m, d, nullptr, b.getNamespace(), b, &urdf, transmissions), "B init");
    check(ahw.ignoresActuators(), "ignore_actuators forwarding");
    check(ahw.sensor(0) != bhw.sensor(0), "isolated accelerometers");
    ahw.EStopActive(false);
    bhw.EStopActive(false);
    auto* ai = ahw.get<hardware_interface::MujocoSpinalInterface>();
    auto* bi = bhw.get<hardware_interface::MujocoSpinalInterface>();
    ai->setForce(0, 2);
    ai->setForce(1, 3);
    bi->setForce(0, 7);
    bi->setForce(1, 8);
    for (int i = 0; i < m->nu; ++i) d->ctrl[i] = -9;
    ahw.WriteSim(ros::Time(1), ros::Duration(.001));
    near(d->ctrl[0], 2, "A rotor1");
    near(d->ctrl[1], 3, "A rotor2");
    near(d->ctrl[2], -9, "A leaves B rotor1");
    near(d->ctrl[3], -9, "A leaves B rotor2");
    bhw.WriteSim(ros::Time(1), ros::Duration(.001));
    near(d->ctrl[0], 2, "B leaves A rotor1");
    near(d->ctrl[1], 3, "B leaves A rotor2");
    near(d->ctrl[2], 7, "B rotor1");
    near(d->ctrl[3], 8, "B rotor2");

    // Reject malformed models/transmissions before trying to read an invalid sensor/actuator.
    {
      Probe hw;
      a.setParam("simulation/model_prefix", "absent/");
      check(!hw.InitSim(m, d, nullptr, a.getNamespace(), a, &urdf, transmissions), "missing fc rejected");
      a.setParam("simulation/model_prefix", "gimbalrotor1/");
    }
    {
      Probe hw;
      const int id = ahw.sensor(0);
      const int dim = m->sensor_dim[id];
      m->sensor_dim[id] = 2;
      check(!hw.InitSim(m, d, nullptr, a.getNamespace(), a, &urdf, transmissions), "2-axis IMU rejected");
      m->sensor_dim[id] = dim;
    }
    {
      Probe hw;
      const int id = ahw.sensor(0);
      const int site = m->sensor_objid[id];
      m->sensor_objid[id] = m->sensor_objid[bhw.sensor(0)];
      check(!hw.InitSim(m, d, nullptr, a.getNamespace(), a, &urdf, transmissions), "foreign IMU rejected");
      m->sensor_objid[id] = site;
    }
    {
      Probe hw;
      check(!hw.InitSim(m, d, nullptr, a.getNamespace(), a, &urdf, {rotor("rotor2")}), "rotor gap rejected");
    }
    {
      Probe hw;
      check(!hw.InitSim(m, d, nullptr, a.getNamespace(), a, &urdf, {rotor("rotor1"), rotor("rotor1")}),
            "duplicate rotor rejected");
    }
    {
      Probe hw;
      auto malformed = rotor("rotor1");
      malformed.joints_.clear();
      check(!hw.InitSim(m, d, nullptr, a.getNamespace(), a, &urdf, {malformed}),
            "missing rotor joint rejected");
    }
    {
      Probe hw;
      auto missing = rotor("missing");
      missing.actuators_[0].hardware_interfaces_ = {"EffortJointInterface"};
      auto ts = transmissions;
      ts.push_back(missing);
      check(!hw.InitSim(m, d, nullptr, a.getNamespace(), a, &urdf, ts), "base InitSim failure propagated");
    }

    DefaultAerialRobotHWSim default_a, default_b;
    check(default_a.InitSim(m, d, nullptr, a.getNamespace(), a, &urdf, transmissions), "default A init");
    check(default_b.InitSim(m, d, nullptr, b.getNamespace(), b, &urdf, transmissions), "default B init");
    default_a.EStopActive(false);
    default_b.EStopActive(false);
    auto* rai = default_a.get<hardware_interface::RotorInterface>();
    auto* rbi = default_b.get<hardware_interface::RotorInterface>();
    rai->getHandle("rotor1").setCommand(11);
    rbi->getHandle("rotor1").setCommand(13);
    default_a.WriteSim(ros::Time(1), ros::Duration(.001));
    near(d->ctrl[0], 11, "default A command");
    near(d->ctrl[2], 7, "default A leaves B");
    default_b.WriteSim(ros::Time(1), ros::Duration(.001));
    near(d->ctrl[0], 11, "default B leaves A");
    near(d->ctrl[2], 13, "default B command");

    // Published odometry must use each robot's own freejoint offsets and local ROS frame names.
    nav_msgs::Odometry odom_a, odom_b, gt_a, gt_b;
    bool got_a = false, got_b = false, got_ga = false, got_gb = false;
    auto sa = a.subscribe<nav_msgs::Odometry>("odom", 1, [&](const nav_msgs::Odometry::ConstPtr& msg) {
      odom_a = *msg;
      got_a = true;
    });
    auto sb = b.subscribe<nav_msgs::Odometry>("odom", 1, [&](const nav_msgs::Odometry::ConstPtr& msg) {
      odom_b = *msg;
      got_b = true;
    });
    auto sga =
        a.subscribe<nav_msgs::Odometry>("ground_truth", 1, [&](const nav_msgs::Odometry::ConstPtr& msg) {
          gt_a = *msg;
          got_ga = true;
        });
    auto sgb =
        b.subscribe<nav_msgs::Odometry>("ground_truth", 1, [&](const nav_msgs::Odometry::ConstPtr& msg) {
          gt_b = *msg;
          got_gb = true;
        });
    for (int i = 0; i < 100 && (!sa.getNumPublishers() || !sb.getNumPublishers() || !sga.getNumPublishers() ||
                                !sgb.getNumPublishers());
         ++i)
    {
      ros::spinOnce();
      ros::WallDuration(.01).sleep();
    }
    const int bj = mj_name2id(m, mjOBJ_JOINT, "gimbalrotor2/root_joint");
    const int qp = m->jnt_qposadr[bj], qv = m->jnt_dofadr[bj];
    d->qpos[qp + 3] = std::sqrt(.5);
    d->qpos[qp + 6] = std::sqrt(.5);
    d->qvel[qv] = 2;
    d->qvel[qv + 1] = 3;
    d->qvel[qv + 2] = 4;
    mj_forward(m, d);
    default_a.ReadSim(ros::Time(2), ros::Duration(.001));
    default_b.ReadSim(ros::Time(2), ros::Duration(.001));
    ahw.ReadSim(ros::Time(2), ros::Duration(.001));
    bhw.ReadSim(ros::Time(2), ros::Duration(.001));
    for (int i = 0; i < 100 && !(got_a && got_b && got_ga && got_gb); ++i)
    {
      ros::spinOnce();
      ros::WallDuration(.01).sleep();
    }
    check(got_a && got_b && got_ga && got_gb, "both odometry and ground-truth topics publish");
    near(odom_a.pose.pose.position.x, 1, "A position");
    near(odom_b.pose.pose.position.x, 4, "B position");
    near(odom_b.twist.twist.linear.x, 3, "B local vx");
    near(odom_b.twist.twist.linear.y, -2, "B local vy");
    check(tf::strip_leading_slash(odom_b.child_frame_id) == "isolation/b/base",
          "local root frame: " + odom_b.child_frame_id);
    near(gt_a.pose.pose.position.x, 1, "A ground truth");
    near(gt_b.pose.pose.position.x, 4, "B ground truth");
    near(gt_b.twist.twist.linear.x, 2, "B ground truth world vx");
    near(gt_b.twist.twist.linear.y, 3, "B ground truth world vy");

    // Drift is applied to both the published observation and spinal's controller state.
    bhw.seedDrift();
    bi->useGroundTruth(true);
    got_gb = false;
    bhw.ReadSim(ros::Time(2.1), ros::Duration(.001));
    for (int i = 0; i < 100 && !got_gb; ++i)
    {
      ros::spinOnce();
      ros::WallDuration(.01).sleep();
    }
    check(got_gb, "ground truth with drift publishes");
    near(gt_b.twist.twist.linear.x, 2.5, "linear velocity drift");
    near(gt_b.twist.twist.angular.x, 0.4, "angular velocity drift");
    near(tf::getYaw(gt_b.pose.pose.orientation), M_PI / 2 + 0.1, "rotation drift");
    near(bi->getEstimatorPtr()->getAttEstimator()->getAngular().x, 0.4, "controller sees angular drift");
  }
  catch (const std::exception& e)
  {
    FAIL() << e.what();
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "mujoco_hardware_isolation");
  return RUN_ALL_TESTS();
}
