
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_CAMERA_PINHOLE_HPP
#define OPENMVG_CAMERA_PINHOLE_HPP

#include "openMVG/numeric/numeric.h"
#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/geometry/pose3.hpp"
#include "ceres/rotation.h"

#include <vector>

namespace openMVG {
namespace cameras {

/// Define a classic Pinhole camera (store a K 3x3 matrix)
///  with intrinsic parameters defining the K calibration matrix
class Pinhole_Intrinsic : public IntrinsicBase
{
  protected:
    // Focal & principal point are embed into the calibration matrix K
    Mat3 _K, _Kinv;

  public:
  Pinhole_Intrinsic(
    unsigned int w = 0, unsigned int h = 0,
    double focal_length_pix = 0.0,
    double ppx = 0.0, double ppy = 0.0)
    :IntrinsicBase(w,h)
  {
    _K << focal_length_pix, 0., ppx, 0., focal_length_pix, ppy, 0., 0., 1.;
    _Kinv = _K.inverse();
  }

  virtual EINTRINSIC getType() const { return PINHOLE_CAMERA; }

  const Mat3& K() const { return _K; }
  const Mat3& Kinv() const { return _Kinv; }
  /// Return the value of the focal in pixels
  inline const double focal() const {return _K(0,0);}
  inline const Vec2 principal_point() const {return Vec2(_K(0,2), _K(1,2));}

  // Get bearing vector of p point (image coord)
  Vec3 operator () (const Vec2& p) const
  {
    Vec3 p3(p(0),p(1),1.0);
    return (_Kinv * p3).normalized();
  }

  // Transform a point from the camera plane to the image plane
  Vec2 cam2ima(const Vec2& p) const
  {
    return focal() * p + principal_point();
  }

  // Transform a point from the image plane to the camera plane
  Vec2 ima2cam(const Vec2& p) const
  {
    return ( p -  principal_point() ) / focal();
  }

  virtual bool have_disto() const {  return false; }

  virtual Vec2 add_disto(const Vec2& p) const  { return p; }

  virtual Vec2 remove_disto(const Vec2& p) const  { return p; }

  virtual double imagePlane_toCameraPlaneError(double value) const
  {
    return value / focal();
  }

  virtual Mat34 get_projective_equivalent(const geometry::Pose3 & pose) const
  {
    Mat34 P;
    P_From_KRt(K(), pose.rotation(), pose.translation(), &P);
    return P;
  }

  // Data wrapper for non linear optimization (get data)
  virtual std::vector<double> getParams() const
  {
    const std::vector<double> params = {_K(0,0), _K(0,2), _K(1,2)};
    return params;
  }

  // Data wrapper for non linear optimization (update from data)
  virtual bool updateFromParams(const std::vector<double> & params)
  {
    if (params.size() == 3) {
      *this = Pinhole_Intrinsic(_w, _h, params[0], params[1], params[2]);
      return true;
    }
    else  {
      return false;
    }
  }

  /// Return the un-distorted pixel (with removed distortion)
  virtual Vec2 get_ud_pixel(const Vec2& p) const {return p;}

  /// Return the distorted pixel (with added distortion)
  virtual Vec2 get_d_pixel(const Vec2& p) const {return p;}

  // Serialization
  template <class Archive>
  void save( Archive & ar) const
  {
    IntrinsicBase::save(ar);
    ar(cereal::make_nvp("focal_length", _K(0,0) ));
    const std::vector<double> pp = {_K(0,2), _K(1,2)};
    ar(cereal::make_nvp("principal_point", pp));
  }

  // Serialization
  template <class Archive>
  void load( Archive & ar)
  {
    IntrinsicBase::load(ar);
    double focal_length;
    ar(cereal::make_nvp("focal_length", focal_length ));
    std::vector<double> pp(2);
    ar(cereal::make_nvp("principal_point", pp));
    *this = Pinhole_Intrinsic(_w, _h, focal_length, pp[0], pp[1]);
  }
};

// Define Intrinsic with a subpose (used for rig configuration)
// Subpose is hidden and not used directly in the class.
// Use intrinsic->subpose() * globalPose when you send a pose to this class.
class Rig_Pinhole_Intrinsic : public Pinhole_Intrinsic
{
  geometry::Pose3 _subpose;

public:

  Rig_Pinhole_Intrinsic(
    unsigned int w = 0, unsigned int h = 0,
    double focal_length_pix = 0.0,
    double ppx = 0.0, double ppy = 0.0,
    geometry::Pose3 subpose = geometry::Pose3())
    : Pinhole_Intrinsic(w, h, focal_length_pix, ppx, ppy),
      _subpose(subpose)
  {}

  bool subpose() const { return true; }

  geometry::Pose3 get_subpose() const { return _subpose; }

  virtual EINTRINSIC getType() const { return PINHOLE_RIG_CAMERA; }

  // Serialization
  template <class Archive>
  void save(Archive & ar) const
  {
    Pinhole_Intrinsic::save(ar);
    ar(cereal::make_nvp("subpose", _subpose));
  }

  // Serialization
  template <class Archive>
  void load(Archive & ar)
  {
    Pinhole_Intrinsic::load(ar);
    ar(cereal::make_nvp("subpose", _subpose));
  }

  // Data wrapper for non linear optimization (get data)
  virtual std::vector<double> getParams() const
  {
    std::vector<double> params = Pinhole_Intrinsic::getParams();
    const Mat3 R = _subpose.rotation();
    double angleAxis[3];
    ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);
    params.push_back(angleAxis[0]);
    params.push_back(angleAxis[1]);
    params.push_back(angleAxis[2]);
    const Vec3 translation = _subpose.translation();
    params.push_back(translation[0]);
    params.push_back(translation[1]);
    params.push_back(translation[2]);
    return params;
  }

  // Data wrapper for non linear optimization (update from data)
  virtual bool updateFromParams(const std::vector<double> & params)
  {
    if (params.size() == 9) {
      const double angleAxis[3] = { params[3], params[4], params[5] }; // rx, ry, rz
      Mat3 R;
      ceres::AngleAxisToRotationMatrix(&angleAxis[0], R.data());
      Vec3 translation;
      translation << params[6], params[7], params[8]; // tx, ty, tz
      *this = Rig_Pinhole_Intrinsic(
        _w, _h,
        params[0], params[1], params[2], // focal, ppx, ppy
        geometry::Pose3(R, -R.transpose() * translation));
      return true;
    }
    else  {
      return false;
    }
  }
};

} // namespace cameras
} // namespace openMVG

#include <cereal/types/polymorphic.hpp>
#include <cereal/types/vector.hpp>

CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::cameras::Pinhole_Intrinsic, "pinhole");
CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::cameras::Rig_Pinhole_Intrinsic, "rig_pinhole");

#endif // #ifndef OPENMVG_CAMERA_PINHOLE_HPP

