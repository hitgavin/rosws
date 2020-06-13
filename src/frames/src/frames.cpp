#include "../../frames/include/frames.h"

#include <cmath>

const double PI = 3.1415926535897932384626433832795;

Rotation::Rotation() {
  data[0] = 1;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 1;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;
  data[8] = 1;
}

Rotation::Rotation(double Xx, double Yx, double Zx, double Xy, double Yy,
                   double Zy, double Xz, double Yz, double Zz) {
  data[0] = Xx;
  data[1] = Yx;
  data[2] = Zx;
  data[3] = Xy;
  data[4] = Yy;
  data[5] = Zy;
  data[6] = Xz;
  data[7] = Yz;
  data[8] = Zz;
}

Rotation Rotation::eulerZXZ(double alpha, double beta, double gamma) {
  double sa, ca, sb, cb, sg, cg;
  sa = sin(alpha);
  ca = cos(alpha);
  sb = sin(beta);
  cb = cos(beta);
  sg = sin(gamma);
  cg = cos(gamma);

  return Rotation(ca * cg - cb * sa * sg, -ca * sg - cb * cg * sa, sa * sb,
                  cg * sa + ca * cb * sg, ca * cb * cg - sa * sg, -ca * sb,
                  sb * sg, cg * sb, cb);
}

void Rotation::getEulerZXZ(double &alpha, double &beta, double &gamma) const {
  double epsilon = 1E-12;
  if (fabs(data[8] > 1 - epsilon)) {
    gamma = 0.0;
    if (data[8] > 0.0) {
      beta = 0.0;
      alpha = atan2(data[3], data[0]);
    } else {
      beta = PI;
      alpha = atan2(data[3], data[0]);
    }
  } else {
    alpha = atan2(data[2], -data[5]);
    beta = atan2(sqrt(pow(data[6], 2) + pow(data[7], 2)), data[8]);
    gamma = atan2(data[6], data[7]);
  }
}

void Rotation::getQuaternion(double &x, double &y, double &z, double &w) const {
  double epsilon = 1E-12;
  double r11 = data[0];
  double r12 = data[1];
  double r13 = data[2];
  double r21 = data[3];
  double r22 = data[4];
  double r23 = data[5];
  double r31 = data[6];
  double r32 = data[7];
  double r33 = data[8];
  w = sqrt(1.0f + r11 + r22 + r33) / 2.0f;
  if (fabs(w) > epsilon) {
    // compute w first.
    x = (r32 - r23) / (4 * w);
    y = (r13 - r31) / (4 * w);
    z = (r21 - r12) / (4 * w);
  } else {
    if (r11 >= r22 && r11 >= r33) {
      // compute x first
      x = sqrt(1.0f + r11 - r22 - r33) / 2.0f;
      w = (r32 - r23) / (4 * x);
      y = (r21 + r12) / (4 * x);
      z = (r31 + r13) / (4 * x);
    } else if (r22 >= r11 && r22 >= r33) {
      // compute y first
      y = sqrt(1.0f - r11 + r22 - r33) / 2.0f;
      w = (r13 - r31) / (4 * y);
      x = (r21 + r12) / (4 * y);
      z = (r23 + r32) / (4 * y);
    } else {
      // compute z first
      z = sqrt(1.0f - r11 - r22 + r33) / 2.0f;
      w = (r21 - r12) / (4 * z);
      x = (r13 + r31) / (4 * z);
      y = (r23 + r32) / (4 * z);
    }
  }
}

Rotation Rotation::quaternion(double x, double y, double z, double w) {
  Rotation r;
  r.data[0] = 1 - 2 * y * y - 2 * z * z;
  r.data[1] = 2 * (x * y - w * z);
  r.data[2] = 2 * (x * z + w * y);
  r.data[3] = 2 * (x * y + w * z);
  r.data[4] = 1 - 2 * x * x - 2 * z * z;
  r.data[5] = 2 * (y * z - w * x);
  r.data[6] = 2 * (x * z - w * y);
  r.data[7] = 2 * (y * z + w * x);
  r.data[8] = 1 - 2 * x * x - 2 * y * y;

  return r;
}
