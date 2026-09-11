#include <gtest/gtest.h>
#include <ros/ros.h>
#include <state_estimate/attitude/complementary_ahrs.h>
#include <vector>

TEST(MujocoEstimator, InterleavedRobotsMatchIndependentEstimation)
{
  ros::Time::init();
  const ap::Vector3f gyro(0, 0, 0), mag(1, 0, 0);
  const ap::Vector3f acc_a(1, 0, 9.7), acc_b(0, -2, 9.5);
  ComplementaryAHRS reference_a, reference_b, a, b;
  std::vector<ap::Vector3f> expected_a, expected_b;
  for (int i = 0; i < 30; ++i)
    {
      reference_a.update(gyro, acc_a, mag);
      expected_a.push_back(reference_a.getEstG());
    }
  for (int i = 0; i < 30; ++i)
    {
      reference_b.update(gyro, acc_b, mag);
      expected_b.push_back(reference_b.getEstG());
    }
  for (int i = 0; i < 30; ++i)
    {
      a.update(gyro, acc_a, mag);
      b.update(gyro, acc_b, mag);
      for (int axis = 0; axis < 3; ++axis)
        {
          EXPECT_FLOAT_EQ(expected_a[i][axis], a.getEstG()[axis]);
          EXPECT_FLOAT_EQ(expected_b[i][axis], b.getEstG()[axis]);
        }
    }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
