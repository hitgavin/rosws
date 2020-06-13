
#include <gtest/gtest.h>

#include <eigen3/Eigen/Dense>

#include <cmath>

#include "../../frames/include/frames.h"

class RotationTest : public ::testing::Test {
protected:
  void SetUp() override { rot_ = Rotation::eulerZXZ(PI / 3, PI / 4, PI / 6); }

  void TearDown() override {}

  Rotation rot_;
};

TEST_F(RotationTest, TestEulerZXZ) {
  double data[9] = {0.1268,  -0.7803, 0.6124, 0.9268, -0.1268,
                    -0.3536, 0.3536,  0.6124, 0.7071};

  for (int i = 0; i < sizeof(data) / sizeof(double); ++i) {
    ASSERT_LT(fabs(rot_.data[i] - data[i]), 0.01);
  }
}

TEST_F(RotationTest, TestGetEulerZXZ) {
  double alpha = 0.0;
  double beta = 0.0;
  double gamma = 0.0;
  rot_.getEulerZXZ(alpha, beta, gamma);

  ASSERT_LT(fabs(alpha - PI / 3), 0.0001);
  ASSERT_LT(fabs(beta - PI / 4), 0.0001);
  ASSERT_LT(fabs(gamma - PI / 6), 0.0001);
}

TEST_F(RotationTest, TestQuaternion) {
  // rotation pi/3 along z axis
  // q=[cos(pi/6), sin(pi/6)[0,0,1]]
  // w = cos(pi/6), x = y = 0, z = sin(pi/6)
  double data[9] = {
      cos(PI / 3), -sin(PI / 3), 0, sin(PI / 3), cos(PI / 3), 0, 0, 0, 1};
  double epsilon = 1e-6;
  Rotation rot = Rotation::quaternion(0, 0, sin(PI / 6), cos(PI / 6));
  for (int i = 0; i < sizeof(data) / sizeof(double); ++i) {
    ASSERT_LT(fabs(rot.data[i] - data[i]), epsilon);
  }
}

TEST_F(RotationTest, TestGetQuaternion) {
  double x, y, z, w;
  rot_.getQuaternion(x, y, z, w);
  Eigen::Quaterniond q(w, x, y, z);
  Eigen::Matrix3d r = q.toRotationMatrix();

  double epsilon = 1e-6;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      ASSERT_LT(fabs(r(i, j) - rot_.data[i * 3 + j]), epsilon);
    }
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
