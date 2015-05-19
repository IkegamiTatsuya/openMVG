// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_SFM_VIEW_HPP
#define OPENMVG_SFM_VIEW_HPP

#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include <cereal/cereal.hpp> // Serialization

namespace openMVG {

enum EView
{
  VIEW = 1,
  RIG_VIEW = 2 
};

/// A view define an image by a string and unique indexes for the view, the camera intrinsic & the pose
struct View
{
  // image path on disk
  std::string s_Img_path;

  // Id of the view
  IndexT id_view;

  // Index of intrinsics and the pose
  IndexT id_intrinsic, id_pose;

  // image size
  IndexT ui_width, ui_height;

  // Constructor (use unique index for the view_id)
  View(
    const std::string & sImgPath = "",
    IndexT view_id = UndefinedIndexT,
    IndexT intrinsic_id = UndefinedIndexT,
    IndexT pose_id = UndefinedIndexT,
    IndexT width = UndefinedIndexT, IndexT height = UndefinedIndexT)
    :s_Img_path(sImgPath), id_view(view_id), id_intrinsic(intrinsic_id),
    id_pose(pose_id), ui_width(width), ui_height(height)
    {}

  // Serialization
  template <class Archive>
  void serialize( Archive & ar )
  {
    //Define a view with two string (base_path & basename)
    std::string local_path = stlplus::folder_append_separator(
      stlplus::folder_part(s_Img_path));
    std::string filename = stlplus::filename_part(s_Img_path);

    ar(cereal::make_nvp("local_path", local_path),
       cereal::make_nvp("filename", filename),
       cereal::make_nvp("width", ui_width),
       cereal::make_nvp("height", ui_height),
       cereal::make_nvp("id_view", id_view),
       cereal::make_nvp("id_intrinsic", id_intrinsic),
       cereal::make_nvp("id_pose", id_pose));

    s_Img_path = stlplus::create_filespec(local_path, filename);
  }

  virtual EView get_type() const { return VIEW; }
};

// Define a View that belongs to a Rigid rig
// Define a rig index and a subrig index
struct Rig_View : public View
{
  IndexT id_rig;
  IndexT id_subrig; // subrig is also hidden in id_intrinsic

  // Constructor (use unique index for the view_id)
  Rig_View(
    const std::string & sImgPath = "",
    IndexT view_id = UndefinedIndexT,
    IndexT intrinsic_id = UndefinedIndexT,
    IndexT pose_id = UndefinedIndexT,
    IndexT width = UndefinedIndexT, IndexT height = UndefinedIndexT,
    IndexT rig_id = UndefinedIndexT,
    IndexT subrig_id = UndefinedIndexT)
    : View(sImgPath, view_id, intrinsic_id,
        pose_id, width, height),
        id_rig(rig_id), id_subrig(subrig_id)
  {}

  // Serialization
  template <class Archive>
  void serialize(Archive & ar)
  {
    View::serialize(ar);
    ar(cereal::make_nvp("id_rig", id_rig),
      cereal::make_nvp("id_subrig", id_subrig));
  }
  virtual EView get_type() const { return RIG_VIEW; }
};

} // namespace openMVG

#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::Rig_View, "rig_view");


#endif // OPENMVG_SFM_VIEW_HPP
