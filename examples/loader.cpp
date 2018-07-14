#include <iostream>

#include "sdf-loader/sdf-loader.hpp"

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChTexture.h"
#include "chrono_irrlicht/ChIrrApp.h"


int main(int argc, char* argv[]) {

  // check arguments
  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0]
      << " <sdf-path>" << std::endl;
    return -1;
  }

  const std::string sdfPath(argv[1]);

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
  ChIrrApp application(&mphysicalSystem, L"sdf-loader example", core::dimension2d<u32>(800, 600),
      false);  // screen dimensions

  // Create a floor that is fixed (that is used also to represent the absolute reference)

  auto floorBody = std::make_shared<ChBodyEasyBox>(20, 2, 20, 3000, true, true);
  floorBody->SetPos(ChVector<>(0, 0, 0));
  floorBody->SetBodyFixed(true);
  mphysicalSystem.Add(floorBody);

  auto mtexture = std::make_shared<ChTexture>();
  mtexture->SetTextureFilename(GetChronoDataFile("blu.png"));
  floorBody->AddAsset(mtexture);

  // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
  application.AddTypicalLogo();
  application.AddTypicalSky();
  application.AddTypicalLights();
  application.AddTypicalCamera(core::vector3df(2, 2, -5),
      core::vector3df(0, 1, 0));  // to change the position of camera
  // application.AddLightWithShadow(vector3df(1,25,-5), vector3df(0,0,0), 35, 0.2,35, 55, 512, video::SColorf(1,1,1));


  if(!loadSDF(mphysicalSystem, sdfPath)) {
    std::cerr << "Failed to load sdf." << std::endl;
    return -1;
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
