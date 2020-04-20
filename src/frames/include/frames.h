#pragma once

extern const double PI;

/**
 * @brief This class is used to represent rotation in 3d space
 * */
class Rotation {
public:
  Rotation();
  Rotation(double Xx, double Yx, double Zx, double Xy, double Yy, double Zy,
           double Xz, double Yz, double Zz);
  static Rotation eulerZXZ(double alpha, double beta, double gamma);
  void getEulerZXZ(double &alpha, double &beta, double &gamma) const;

  /**
   * @brief data of the rotation matrix
   * @note the rotation matrix is defined as:
   * [data[0], data[1], data[2];
   *  data[3], data[4], data[5];
   *  data[6], data[7], data[8]]
   * */
  double data[9];
};
