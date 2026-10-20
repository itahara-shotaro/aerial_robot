#include <aerial_robot_control/control/utils/integral_projection.h>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <cmath>

using namespace aerial_robot_control::integral_projection;

namespace
{
  const double EPS = 1e-12;

  std::vector<Eigen::Vector2d> horizontal(const std::vector<Eigen::Vector3d>& body_axes,
                                          const Eigen::Matrix3d& rot = Eigen::Matrix3d::Identity())
  {
    return horizontalAxes(rot, body_axes);
  }
}

TEST(IntegralProjection, AxisXDropsXIncrementKeepsY)
{
  auto axes = horizontal({Eigen::Vector3d(1, 0, 0)});
  Eigen::Vector2d w_prev(0.3, -0.2);
  Eigen::Vector2d w = w_prev + Eigen::Vector2d(0.01, 0.02);
  Eigen::Vector2d w_new = projectOutIntegralAxes(w, w_prev, axes, 0.0);
  EXPECT_NEAR(w_new.x(), 0.3, EPS);
  EXPECT_NEAR(w_new.y(), -0.18, EPS);
}

TEST(IntegralProjection, SignOfAxisIsIrrelevant)
{
  Eigen::Vector2d w_prev(0.3, -0.2), w(0.31, -0.18);
  Eigen::Vector2d a = projectOutIntegralAxes(w, w_prev, horizontal({Eigen::Vector3d(1, 0, 0)}), 0.1);
  Eigen::Vector2d b = projectOutIntegralAxes(w, w_prev, horizontal({Eigen::Vector3d(-2, 0, 0)}), 0.1);
  EXPECT_NEAR((a - b).norm(), 0, EPS);
}

TEST(IntegralProjection, RotatedAxisRemovesOnlyParallelComponent)
{
  // body x axis with the body yawed by 30 deg
  Eigen::Matrix3d rot = Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  auto axes = horizontal({Eigen::Vector3d(1, 0, 0)}, rot);
  ASSERT_EQ(axes.size(), 1u);
  Eigen::Vector2d a(std::cos(M_PI / 6), std::sin(M_PI / 6));
  Eigen::Vector2d n(-a.y(), a.x());
  EXPECT_NEAR((axes[0] - a).norm(), 0, EPS);

  Eigen::Vector2d w_prev(0.1, 0.4);
  Eigen::Vector2d delta = 0.02 * a + 0.05 * n;
  Eigen::Vector2d w_new = projectOutIntegralAxes(w_prev + delta, w_prev, axes, 0.0);
  EXPECT_NEAR((w_new - (w_prev + 0.05 * n)).norm(), 0, EPS);
}

TEST(IntegralProjection, StoredAxialComponentDecays)
{
  auto axes = horizontal({Eigen::Vector3d(1, 0, 0)});
  Eigen::Vector2d w(1.0, 0.5);
  const double decay = 0.1;
  for(int i = 0; i < 10; i++) w = projectOutIntegralAxes(w, w, axes, decay);
  EXPECT_NEAR(w.x(), std::pow(1 - decay, 10), EPS);
  EXPECT_NEAR(w.y(), 0.5, EPS);

  // decay is clamped to [0, 1]
  Eigen::Vector2d w_prev(1.0, 0.5);
  EXPECT_NEAR(projectOutIntegralAxes(w_prev, w_prev, axes, 5.0).x(), 0, EPS);
  EXPECT_NEAR(projectOutIntegralAxes(w_prev, w_prev, axes, -1.0).x(), 1.0, EPS);
}

TEST(IntegralProjection, ParallelAxesActAsOne)
{
  // interior drone of a chain: clamps at +x and -x
  auto axes = horizontal({Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(-1, 0, 0)});
  ASSERT_EQ(axes.size(), 2u);
  EXPECT_EQ(orthonormalBasis(axes).size(), 1u);

  Eigen::Vector2d w_prev(0.3, -0.2), w(0.4, -0.1);
  Eigen::Vector2d single = projectOutIntegralAxes(w, w_prev, horizontal({Eigen::Vector3d(1, 0, 0)}), 0.2);
  Eigen::Vector2d both = projectOutIntegralAxes(w, w_prev, axes, 0.2);
  EXPECT_NEAR((single - both).norm(), 0, EPS);
}

TEST(IntegralProjection, OrthogonalAxesRemoveEverything)
{
  auto axes = horizontal({Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(1, 1, 0)});
  EXPECT_EQ(orthonormalBasis(axes).size(), 2u);

  Eigen::Vector2d w_prev(0.3, -0.2), w(0.4, -0.1);
  EXPECT_NEAR((projectOutIntegralAxes(w, w_prev, axes, 0.0) - w_prev).norm(), 0, EPS);
  EXPECT_NEAR((projectOutIntegralAxes(w, w_prev, axes, 0.25) - 0.75 * w_prev).norm(), 0, EPS);
}

TEST(IntegralProjection, NearVerticalAxisIgnored)
{
  // body x pitched by 70 deg: horizontal part cos(70 deg) = 0.34 < 0.5
  Eigen::Matrix3d rot = Eigen::AngleAxisd(70 * M_PI / 180, Eigen::Vector3d::UnitY()).toRotationMatrix();
  int skipped = -1;
  auto axes = horizontalAxes(rot, {Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, 0)}, 0.5, &skipped);
  EXPECT_EQ(skipped, 1);
  ASSERT_EQ(axes.size(), 1u);
  EXPECT_NEAR((axes[0] - Eigen::Vector2d(0, 1)).norm(), 0, EPS);

  auto none = horizontalAxes(rot, {Eigen::Vector3d(1, 0, 0)}, 0.5, &skipped);
  EXPECT_EQ(skipped, 1);
  EXPECT_TRUE(none.empty());
  Eigen::Vector2d w_prev(0.3, -0.2), w(0.4, -0.1);
  EXPECT_NEAR((projectOutIntegralAxes(w, w_prev, none, 0.5) - w).norm(), 0, EPS);

  // a moderate tilt is renormalised, not shortened
  Eigen::Matrix3d tilt = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()).toRotationMatrix();
  auto tilted = horizontalAxes(tilt, {Eigen::Vector3d(1, 0, 0)});
  ASSERT_EQ(tilted.size(), 1u);
  EXPECT_NEAR(tilted[0].norm(), 1, EPS);
}

TEST(IntegralProjection, UnequalGainsProjectInITermSpace)
{
  // axis at 45 deg, kx = 2 ky: the removed part must be parallel to the axis in i-term (acceleration) space
  Eigen::Vector2d k(1.0, 0.5);
  Eigen::Matrix3d rot = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  auto axes = horizontal({Eigen::Vector3d(1, 0, 0)}, rot);
  Eigen::Vector2d a = axes[0];

  Eigen::Vector2d err_i_prev(0.2, -0.4);
  Eigen::Vector2d err_i(0.25, -0.3);
  Eigen::Vector2d err_i_new = projectOutErrI(err_i, err_i_prev, k, axes, 0.0);

  Eigen::Vector2d w_prev = k.cwiseProduct(err_i_prev);
  Eigen::Vector2d w_inc = k.cwiseProduct(err_i_new) - w_prev;
  EXPECT_NEAR(w_inc.dot(a), 0, EPS);  // nothing new along the axis
  Eigen::Vector2d delta = k.cwiseProduct(err_i) - w_prev;
  EXPECT_NEAR((w_inc - (delta - delta.dot(a) * a)).norm(), 0, EPS);  // lateral increment kept

  // equal gains reduce to the err_i-space projection
  Eigen::Vector2d same = projectOutErrI(err_i, err_i_prev, Eigen::Vector2d(0.5, 0.5), axes, 0.1);
  EXPECT_NEAR((same - projectOutIntegralAxes(err_i, err_i_prev, axes, 0.1)).norm(), 0, EPS);
}

TEST(IntegralProjection, ZeroGainAxisIsLeftAlone)
{
  auto axes = horizontal({Eigen::Vector3d(1, 1, 0)});
  Eigen::Vector2d err_i_prev(0.2, -0.4), err_i(0.25, -0.3);
  Eigen::Vector2d err_i_new = projectOutErrI(err_i, err_i_prev, Eigen::Vector2d(0.0, 0.5), axes, 0.1);
  EXPECT_TRUE(err_i_new.allFinite());
  EXPECT_EQ(err_i_new.x(), err_i.x());

  Eigen::Vector2d both_zero = projectOutErrI(err_i, err_i_prev, Eigen::Vector2d::Zero(), axes, 0.1);
  EXPECT_EQ(both_zero, err_i);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
