#include <gtest/gtest.h>
#include <mujoco_ros/ros_one/initial_joint_states.hpp>

using JointMap = std::map<std::string, std::vector<double>>;

TEST(InitialJointStates, RecoversSlashPrefixesAfterRoslaunchExpansion)
{
  XmlRpc::XmlRpcValue values;
  values["left"]["gimbal1"] = std::string("0.25");
  values["right"]["nested"]["gimbal1"] = std::string("-0.5");
  values["root_joint"] = std::string(" 1 2 3 1 0 0 0 ");
  JointMap joints;
  std::string error;
  ASSERT_TRUE(mujoco_ros::detail::ParseInitialJointMap(values, joints, error)) << error;
  EXPECT_EQ(joints.at("left/gimbal1"), std::vector<double>({0.25}));
  EXPECT_EQ(joints.at("right/nested/gimbal1"), std::vector<double>({-0.5}));
  EXPECT_EQ(joints.at("root_joint"), std::vector<double>({1, 2, 3, 1, 0, 0, 0}));
}

TEST(InitialJointStates, PreservesFlatMapsAndAlreadyPrefixedKeys)
{
  XmlRpc::XmlRpcValue values;
  values["gimbal1"] = std::string("0");
  values["left/gimbal1"] = std::string("1");
  JointMap joints;
  std::string error;
  ASSERT_TRUE(mujoco_ros::detail::ParseInitialJointMap(values, joints, error)) << error;
  EXPECT_EQ(joints.size(), 2U);
  EXPECT_EQ(joints.at("gimbal1"), std::vector<double>({0}));
  EXPECT_EQ(joints.at("left/gimbal1"), std::vector<double>({1}));
}

TEST(InitialJointStates, RejectsMalformedEntriesWithoutPartialState)
{
  for (const std::string text : {"", "nan", "inf", "1 broken", "1,2"}) {
    XmlRpc::XmlRpcValue values;
    values["first"] = std::string("0.25");
    values["nested"]["bad"] = text;
    JointMap joints;
    std::string error;
    EXPECT_FALSE(mujoco_ros::detail::ParseInitialJointMap(values, joints, error)) << text;
    EXPECT_TRUE(joints.empty());
    EXPECT_FALSE(error.empty());
  }
  XmlRpc::XmlRpcValue values;
  values["first"] = 0; // ROS1's existing API deliberately requires string scalars.
  JointMap joints;
  std::string error;
  EXPECT_FALSE(mujoco_ros::detail::ParseInitialJointMap(values, joints, error));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
