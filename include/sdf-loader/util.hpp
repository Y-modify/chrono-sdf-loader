#ifndef SDF_LOADER_UTIL_HPP
#define SDF_LOADER_UTIL_HPP

#include <cstdlib>
#include <iostream>
#include <regex>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/range/adaptor/indexed.hpp>

namespace sdfl
{
std::map<std::string, chrono::ChColor> presets = {
    {"Gazebo/Orange", chrono::ChColor(1, 0.5088, 0.0468)},
    {"Gazebo/Grey", chrono::ChColor(0.2, 0.2, 0.2)},
    {"Gazebo/Black", chrono::ChColor(0, 0, 0)},
};

std::shared_ptr<chrono::ChColorAsset> get_color_by_name(std::string const& name);
std::shared_ptr<chrono::ChColorAsset> get_color_by_name(std::string const& name)
{
  return std::make_shared<chrono::ChColorAsset>(presets[name]);
}

template <typename T>
std::vector<std::string> split(std::string const& src, T pat)
{
  std::vector<std::string> result{};
  boost::algorithm::split(result, src, boost::is_any_of(pat));
  return result;
}

template <typename T>
std::tuple<T, T, T> destruct_three(std::string const& value)
{
  auto const list = split(value, " ");
  if (list.size() != 3) {
    throw std::runtime_error("Not three pair but " + std::to_string(list.size()));
  }

  return std::make_tuple(boost::lexical_cast<T>(list[0]), boost::lexical_cast<T>(list[1]),
                         boost::lexical_cast<T>(list[2]));
}

template <typename T = double>
std::tuple<chrono::ChVector<T>, chrono::ChMatrix33<T>> destruct_pose(std::string const& pose)
{
  chrono::ChVector<T> pos;
  chrono::ChVector<T> rot;

  auto const list = split(pose, " ");
  for (auto const& val : list | boost::adaptors::indexed()) {
    auto const idx = val.index();
    T dval;
    try {
      dval = boost::lexical_cast<T>(val.value());
    } catch (boost::bad_lexical_cast&) {
      throw std::runtime_error("Invalid value in pose: " + val.value());
    }
    if (idx < 3) {
      pos[idx] = dval;
    } else {
      rot[idx - 3] = dval;
    }
  }

  return std::make_tuple(pos, chrono::ChMatrix33(rot));
}

template <typename T = double>
decltype(auto) get_pose(sdf::ElementPtr e)
{
  auto elem = e->GetElement("pose");
  if (!elem) {
    throw std::runtime_error("No pose found");
  }
  auto const str = elem->GetValue()->GetAsString();
  return destruct_pose<T>(str);
}

template <typename T = double>
std::tuple<T, T, T> get_box_size(sdf::ElementPtr e)
{
  auto elem = e->GetElement("geometry");
  if (!elem) {
    throw std::runtime_error("No geometry found");
  }
  if (!elem->HasElement("box")) {
    throw std::runtime_error("not a box");
  }
  auto size_e = elem->GetElement("box")->GetElement("size");
  if (!size_e) {
    throw std::runtime_error("No size in box");
  }
  auto const str = size_e->GetValue()->GetAsString();
  return destruct_three<T>(str);
}

template <typename T = double>
chrono::ChMatrix33<T> get_inertia(sdf::ElementPtr e)
{
  if (!e->HasElement("inertia")) {
    throw std::runtime_error("No inertia found");
  }
  auto elem = e->GetElement("inertia");
  T ixx, ixy, ixz, iyy, iyz, izz;
  elem->GetElement("ixx")->GetValue()->Get<T>(ixx);
  elem->GetElement("ixy")->GetValue()->Get<T>(ixy);
  elem->GetElement("ixz")->GetValue()->Get<T>(ixz);
  elem->GetElement("iyy")->GetValue()->Get<T>(iyy);
  elem->GetElement("iyz")->GetValue()->Get<T>(iyz);
  elem->GetElement("izz")->GetValue()->Get<T>(izz);
  return chrono::ChMatrix33<T>(ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz);
}

template <typename T>
void print_vec(chrono::ChVector<T> vec, std::ostream& os)
{
  os << '(' << vec.x() << ',' << vec.y() << ',' << vec.z() << ')';
}

}  // namespace sdfl

#endif  // SDF_LOADER_UTIL_HPP
