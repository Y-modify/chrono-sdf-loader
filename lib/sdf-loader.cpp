#include <iostream>

#include <sdf/sdf.hh>

#include "sdf-loader/sdf-loader.hpp"

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/geometry/ChBox.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChSystemNSC.h"

#include "sdf-loader/util.hpp"

namespace sdfl
{
bool loadSDF(chrono::ChSystemNSC& mphysicalSystem, std::string const& sdfPath)
{
  sdf::SDFPtr sdfElement(new sdf::SDF());
  sdf::init(sdfElement);
  if (!sdf::readFile(sdfPath, sdfElement)) {
    std::cerr << sdfPath << " is not a valid SDF file!" << std::endl;
    return false;
  }

  // start parsing model
  const sdf::ElementPtr rootElement = sdfElement->Root();
  if (!rootElement->HasElement("model")) {
    std::cerr << sdfPath << " is not a model SDF file!" << std::endl;
    return false;
  }
  const sdf::ElementPtr modelElement = rootElement->GetElement("model");
  const std::string modelName        = modelElement->Get<std::string>("name");
  std::cout << "Found " << modelName << " model!" << std::endl;

  chrono::ChFrame<> base(chrono::ChVector<>(0, 1.5, 0), -3.14 / 2, chrono::ChVector<>(1, 0, 0));

  // parse model links
  sdf::ElementPtr linkElement = modelElement->GetElement("link");
  while (linkElement) {
    auto const name = linkElement->Get<std::string>("name");
    auto body       = std::make_shared<chrono::ChBody>();
    body->SetNameString(name);

    auto const [bpos, brot] = get_pose(linkElement);  // 相対座標系 原点: base_link
    auto const absCoord     = base.GetA() * bpos + base.GetPos();  // 絶対座標系

    body->SetPos(absCoord);
    body->SetRot(base.GetA());

    std::cout << "**" << name << "**" << std::endl;
    {
      if (!linkElement->HasElement("collision")) {
        std::cerr << "No collision found" << std::endl;
      } else {
        auto elem               = linkElement->GetElement("collision");
        auto const [pos, rot]   = get_pose(elem);
        auto const [hx, hy, hz] = get_box_size(elem);

        body->GetCollisionModel()->ClearModel();
        body->GetCollisionModel()->AddBox(hx * 0.5, hy * 0.5, hz * 0.5);
        body->GetCollisionModel()->BuildModel();
        body->SetCollide(true);
      }
    }
    {
      if (!linkElement->HasElement("inertial")) {
        std::cerr << "No inertial found" << std::endl;
        return false;
      }
      auto elem = linkElement->GetElement("inertial");

      {
        if (!elem->HasElement("mass")) {
          std::cerr << "No mass found" << std::endl;
          return false;
        }
        double mass;
        elem->GetElement("mass")->GetValue()->Get<double>(mass);
        body->SetMass(mass);
      }

      // TODO: Convert inertial matrix based on <pose>
      if (elem->HasElement("pose"))
        std::cerr << "Ignoring <pose> in <inertial>" << std::endl;

      auto inertia = get_inertia(elem);
      body->SetInertia(inertia);
    }
    {
      if (!linkElement->HasElement("visual")) {
        std::cerr << "No visual found" << std::endl;
      } else {
        auto elem = linkElement->GetElement("visual");
        {
          if (!elem->HasElement("material")) {
            std::cerr << "No material found" << std::endl;
          } else {
            std::string colorName;
            elem->GetElement("material")
                ->GetElement("script")
                ->GetElement("name")
                ->GetValue()
                ->Get<std::string>(colorName);
            body->AddAsset(get_color_by_name(colorName));
          }
        }

        auto [pos, rot]         = get_pose(elem);  // 相対座標系 原点: body->GetPos()
        auto const [hx, hy, hz] = get_box_size(elem);

        auto const box      = chrono::geometry::ChBox(pos, rot, chrono::ChVector<>(hx, hy, hz));
        auto const boxShape = std::make_shared<chrono::ChBoxShape>(box);
        body->AddAsset(boxShape);
      }
    }
    mphysicalSystem.Add(body);
    linkElement = linkElement->GetNextElement("link");
  }

  sdf::ElementPtr jointElement = modelElement->GetElement("joint");
  while (jointElement) {
    const auto name = jointElement->Get<std::string>("name");

    const auto parentName = jointElement->GetElement("parent")->Get<std::string>();
    const auto parent     = mphysicalSystem.SearchBody(parentName.c_str());

    const auto childName = jointElement->GetElement("child")->Get<std::string>();
    const auto child     = mphysicalSystem.SearchBody(childName.c_str());

    auto rotmotor = std::make_shared<chrono::ChLinkMotorRotationAngle>();
    // Connect the rotor and the stator and add the motor to the system:
    rotmotor->Initialize(child,                               // body A (slave)
                         parent,                              // body B (master)
                         chrono::ChFrame<>(parent->GetPos())  // motor frame, in abs. coords
    );
    rotmotor->SetNameString(name);
    mphysicalSystem.Add(rotmotor);
    auto const f = std::make_shared<chrono::ChFunction_Const>(0);
    rotmotor->SetAngleFunction(f);

    jointElement = jointElement->GetNextElement("joint");
  }

  return true;
}

}  // namespace sdfl
