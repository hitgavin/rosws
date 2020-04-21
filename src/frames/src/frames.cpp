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
