#include <iostream>

#include <sdf/sdf.hh>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/geometry/ChBox.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/collision/ChCModelBullet.h"

#include "util.hpp"

int main(int argc, char* argv[]) {

  // check arguments
  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0]
      << " <sdf-path>" << std::endl;
    return -1;
  }
  const std::string sdfPath(argv[1]);

  sdf::SDFPtr sdfElement(new sdf::SDF());
  sdf::init(sdfElement);
  if (!sdf::readFile(sdfPath, sdfElement))
  {
    std::cerr << sdfPath << " is not a valid SDF file!" << std::endl;
    return -2;
  }

  // Use the namespace of Chrono

  using namespace chrono;
  using namespace chrono::irrlicht;

  // Use the main namespaces of Irrlicht

  using namespace irr;
  using namespace irr::core;
  using namespace irr::scene;
  using namespace irr::video;
  using namespace irr::io;
  using namespace irr::gui;

  // Set path to Chrono data directory
  SetChronoDataPath(CHRONO_DATA_DIR);

  // Create a Chrono physical system
  ChSystemNSC mphysicalSystem;

  // Create the Irrlicht visualization (open the Irrlicht device,
  // bind a simple user interface, etc. etc.)
  ChIrrApp application(&mphysicalSystem, L"A simple project template", core::dimension2d<u32>(800, 600),
      false);  // screen dimensions

  // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
  application.AddTypicalLogo();
  application.AddTypicalSky();
  application.AddTypicalLights();
  application.AddTypicalCamera(core::vector3df(2, 2, -5),
      core::vector3df(0, 1, 0));  // to change the position of camera
  // application.AddLightWithShadow(vector3df(1,25,-5), vector3df(0,0,0), 35, 0.2,35, 55, 512, video::SColorf(1,1,1));


  // start parsing model
  const sdf::ElementPtr rootElement = sdfElement->Root();
  if (!rootElement->HasElement("model"))
  {
    std::cerr << sdfPath << " is not a model SDF file!" << std::endl;
    return -3;
  }
  const sdf::ElementPtr modelElement = rootElement->GetElement("model");
  const std::string modelName = modelElement->Get<std::string>("name");
  std::cout << "Found " << modelName << " model!" << std::endl;

  // parse model links
  sdf::ElementPtr linkElement = modelElement->GetElement("link");
  while (linkElement)
  {
    auto const name = linkElement->Get<std::string>("name");
    auto body = std::make_shared<chrono::ChBody>();
    body->SetNameString(name);

    auto const [pos, rot] = get_pose(linkElement);

    body->SetPos(pos);
    body->SetRot(rot);

    std::cout << "**" <<name << "**" << std::endl;
    {
      if(!linkElement->HasElement("collision")) {
        std::cerr << "No collision found" << std::endl;
      } else {
        auto elem = linkElement->GetElement("collision");
        auto const [pos, rot] = get_pose(elem);
        auto const [hx, hy, hz] = get_box_size(elem);

        auto collision = std::make_shared<chrono::collision::ChModelBullet>();
        collision->AddBox(hx, hy, hz, pos, rot);
        body->SetCollisionModel(collision);
      }
    }
    {
      if(!linkElement->HasElement("inertial")) {
        std::cerr << "No inertial found" << std::endl;
        return -3;
      }
      auto elem = linkElement->GetElement("inertial");

      {
        if(!elem->HasElement("mass")) {
          std::cerr << "No mass found" << std::endl;
          return -3;
        }
        double mass;
        elem->GetElement("mass")->GetValue()->Get<double>(mass);
        body->SetMass(mass);
      }

      // TODO: Convert inertial matrix based on <pose>
      if(elem->HasElement("pose"))
        std::cerr << "Ignoring <pose> in <inertial>" << std::endl;

      auto inertia = get_inertia(elem);
      body->SetInertia(inertia);
    }
    {
      if(!linkElement->HasElement("visual")) {
        std::cerr << "No visual found" << std::endl;
      } else {
        auto elem = linkElement->GetElement("visual");

        auto const [pos, rot] = get_pose(elem);
        auto const [hx, hy, hz] = get_box_size(elem);

        auto const box = chrono::geometry::ChBox(pos, rot, chrono::ChVector<>(hx, hy, hz));
        auto const boxShape = std::make_shared<chrono::ChBoxShape>(box);

        body->AddAsset(boxShape);
      }
    }
    mphysicalSystem.Add(body);
    linkElement = linkElement->GetNextElement("link");
  }

  //======================================================================

  // Use this function for adding a ChIrrNodeAsset to all items
  // Otherwise use application.AssetBind(myitem); on a per-item basis.
  application.AssetBindAll();

  // Use this function for 'converting' assets into Irrlicht meshes
  application.AssetUpdateAll();

  // Adjust some settings:
  application.SetTimestep(0.005);
  application.SetTryRealtime(true);

  //
  // THE SOFT-REAL-TIME CYCLE
  //

  while (application.GetDevice()->run()) {
    application.BeginScene();

    application.DrawAll();

    // This performs the integration timestep!
    application.DoStep();

    application.EndScene();
  }

  return 0;
}
