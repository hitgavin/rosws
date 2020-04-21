
#include <gtest/gtest.h>

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

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
