#ifndef CAMERA_PINHOLE_BROWN_UNDISTORTION_H_
#define CAMERA_PINHOLE_BROWN_UNDISTORTION_H_

#include <Eigen/Core>
#include <Eigen/LU>
#include <vector>

using Vec2 = Eigen::Vector2d;
using Mat3 = Eigen::Matrix3d;

class PinholeBrownUndistortion {
 private:
  /// Width of image
  unsigned int w_;

  /// Height of image
  unsigned int h_;

  /// Intrinsic matrix : Focal & principal point are embed into the calibration
  /// matrix K
  Mat3 K_;

  /// Inverse of intrinsic matrix
  Mat3 Kinv_;

  /// center of distortion is applied by the Intrinsics class
  std::vector<double> params_;  // K1, K2, K3, T1, T2

 public:
  /**
   * @brief Constructor
   * @param w Width of image
   * @param h Height of image
   * @param focal Focal distance (in pixel)
   * @param ppx Principal point on X-axis
   * @param ppy Principal point on Y-axis
   * @param k1 First radial distortion coefficient
   * @param k2 Second radial distortion coefficient
   * @param k3 Third radial distortion coefficient
   * @param t1 First tangential distortion coefficient
   * @param t2 Second tangential distortion coefficient
   */
  PinholeBrownUndistortion(int w = 0, int h = 0, double focal = 0.0,
                           double ppx = 0, double ppy = 0, double k1 = 0.0,
                           double k2 = 0.0, double k3 = 0.0, double t1 = 0.0,
                           double t2 = 0.0)
      : w_(w), h_(h), params_({k1, k2, k3, t1, t2}) {
    K_ << focal, 0., ppx, 0., focal, ppy, 0., 0., 1.;
    Kinv_ = K_.inverse();
  }

  /**
   * @brief Does the camera model handle a distortion field?
   * @retval true
   */
  bool have_disto() const { return true; }

  /**
   * @brief Add the distortion field to a point (that is in normalized camera
   * frame)
   * @param p Point before distortion computation (in normalized camera frame)
   * @return point with distortion
   */
  Vec2 add_disto(const Vec2& p) const {
    return (p + distoFunction(params_, p));
  }

  /**
   * @brief Return the un-distorted pixel (with removed distortion)
   * @param p Input distorted pixel
   * @return Point without distortion
   */
  Vec2 get_ud_pixel(const Vec2& p) const {
    return cam2ima(remove_disto(ima2cam(p)));
  }

  /**
   * @brief Return the distorted pixel (with added distortion)
   * @param p Input pixel
   * @return Distorted pixel
   */
  Vec2 get_d_pixel(const Vec2& p) const {
    return cam2ima(add_disto(ima2cam(p)));
  }

  /**
   * @brief Transform a point from the image plane to the camera plane
   * @param p Image plane point
   * @return camera plane point
   */
  Vec2 ima2cam(const Vec2& p) const {
    return (p - principal_point()) / focal();
  }

  /**
   * @brief Transform a point from the camera plane to the image plane
   * @param p Camera plane point
   * @return Point on image plane
   */
  Vec2 cam2ima(const Vec2& p) const { return focal() * p + principal_point(); }

  /**
   * @brief Return the value of the focal in pixels
   * @return Focal of the camera (in pixel)
   */
  inline double focal() const { return K_(0, 0); }

  /**
   * @brief Get principal point of the camera
   * @return Principal point of the camera
   */
  inline Vec2 principal_point() const { return {K_(0, 2), K_(1, 2)}; }

  /**
   * @brief Remove the distortion to a camera point (that is in normalized
   * camera frame)
   * @param p Point with distortion
   * @return Point without distortion
   * @note numerical approximation based on
   * Heikkila J (2000) Geometric Camera Calibration Using Circular Control
   * Points. IEEE Trans. Pattern Anal. Mach. Intell., 22:1066-1077
   */
  Vec2 remove_disto(const Vec2& p) const {
    const double epsilon = 1e-10;  // criteria to stop the iteration
    Vec2 p_u = p;

    Vec2 d = distoFunction(params_, p_u);
    while ((p_u + d - p).lpNorm<1>() >
           epsilon)  // manhattan distance between the two points
    {
      p_u = p - d;
      d = distoFunction(params_, p_u);
    }

    return p_u;
  }

  /**
   * @brief Functor to calculate distortion offset accounting for both radial
   * and tangential distortion
   * @param params List of parameters to define a Brown camera
   * @param p Input point
   * @return Transformed point
   */
  static Vec2 distoFunction(const std::vector<double>& params, const Vec2& p) {
    const double k1 = params[0], k2 = params[1], k3 = params[2], t1 = params[3],
                 t2 = params[4];
    const double r2 = p(0) * p(0) + p(1) * p(1);
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double k_diff = (k1 * r2 + k2 * r4 + k3 * r6);
    const double t_x = t2 * (r2 + 2 * p(0) * p(0)) + 2 * t1 * p(0) * p(1);
    const double t_y = t1 * (r2 + 2 * p(1) * p(1)) + 2 * t2 * p(0) * p(1);
    return {p(0) * k_diff + t_x, p(1) * k_diff + t_y};
  }
};

#endif  // #ifndef CAMERA_PINHOLE_BROWN_UNDISTORTION_H_
