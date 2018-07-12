#include <iostream>
#include <cstdlib>
#include <regex>

#include <boost/range/adaptor/indexed.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <sdf/sdf.hh>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono_irrlicht/ChIrrApp.h"

template<typename T>
std::vector<std::string> split(std::string const& src, T pat){
  std::vector<std::string> result{};
  boost::algorithm::split(result, src, boost::is_any_of(pat));
  return result;
}

template<typename T>
std::tuple<T, T, T> destruct_three(std::string const& value){
  auto const list = split(value, " ");
  if(list.size() != 3) {
    throw std::runtime_error("Not three pair but " + std::to_string(list.size()));
  }

  return std::make_tuple(
      boost::lexical_cast<T>(list[0]),
      boost::lexical_cast<T>(list[1]),
      boost::lexical_cast<T>(list[2])
    );
}

template<typename T=double>
std::tuple<chrono::ChVector<T>, chrono::ChVector<T>> destruct_pose(std::string const& pose){
  chrono::ChVector<T> pos;
  chrono::ChVector<T> rot;

  auto const list = split(pose, " ");
  for(auto const& val : list | boost::adaptors::indexed()) {
    auto const idx = val.index();
    T dval;
    try {
      dval = boost::lexical_cast<T>(val.value());
    } catch(boost::bad_lexical_cast&) {
      throw std::runtime_error("Invalid value in pose: " + val.value());
    }
    if(idx < 3) {
      pos[idx] = dval;
    } else {
      rot[idx - 3] = dval;
    }
  }

  return std::make_tuple(pos, rot);
}

template<typename T>
void print_vec(chrono::ChVector<T> vec, std::ostream& os) {
  os << '(' << vec.x() << ',' << vec.y() << ',' << vec.z() << ')';
}

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

  //======================================================================

  // HERE YOU CAN POPULATE THE PHYSICAL SYSTEM WITH BODIES AND LINKS.
  //
  // An example: a pendulum.

  // 1-Create a floor that is fixed (that is used also to represent the absolute reference)
  // load and check sdf file

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
    const auto linkName = linkElement->Get<std::string>("name");
    auto v = linkElement->GetElement("pose");
    if(!v) {
      std::cerr << "No pose found" << std::endl;
      return -3;
    }

    auto const poseStr = v->GetValue()->GetAsString();
    auto [pose, rot] = destruct_pose<double>(poseStr);

    std::cout << poseStr << std::endl;
    print_vec(pose, std::cout);
    print_vec(rot, std::cout);
    std::cout << std::endl;

    /* auto link = std::make_shared<ChBodyEasyBox>(0.5, 2, 0.5,  // x, y, z dimensions */
    /*     3000,         // density */
    /*     false,        // no contact geometry */
    /*     true          // enable visualization geometry */
    /*     ); */
    /* pendulumBody->SetPos(ChVector<>(0, 3, 0)); */
    /* pendulumBody->SetPos_dt(ChVector<>(1, 0, 0)); */
    linkElement = linkElement->GetNextElement("link");
  }

  auto floorBody = std::make_shared<ChBodyEasyBox>(10, 2, 10,  // x, y, z dimensions
      false,      // no contact geoclientetry
      true        // enable visualization geometry
      );
  floorBody->SetPos(ChVector<>(0, -2, 0));
  floorBody->SetBodyFixed(true);

  mphysicalSystem.Add(floorBody);

  // 2-Create a pendulum

  auto pendulumBody = std::make_shared<ChBodyEasyBox>(0.5, 2, 0.5,  // x, y, z dimensions
      3000,         // density
      false,        // no contact geometry
      true          // enable visualization geometry
      );
  pendulumBody->SetPos(ChVector<>(0, 3, 0));
  pendulumBody->SetPos_dt(ChVector<>(1, 0, 0));

  mphysicalSystem.Add(pendulumBody);

  // 3-Create a spherical constraint.
  //   Here we'll use a ChLinkMateGeneric, but we could also use ChLinkLockSpherical

  auto sphericalLink =
    std::make_shared<ChLinkMateGeneric>(true, true, true, false, false, false);  // x,y,z,Rx,Ry,Rz constrains
  ChFrame<> link_position_abs(ChVector<>(0, 4, 0));

  sphericalLink->Initialize(pendulumBody,        // the 1st body to connect
      floorBody,           // the 2nd body to connect
      false,               // the two following frames are in absolute, not relative, coords.
      link_position_abs,   // the link reference attached to 1st body
      link_position_abs);  // the link reference attached to 2nd body

  mphysicalSystem.Add(sphericalLink);

  // Optionally, attach a RGB color asset to the floor, for better visualization
  auto color = std::make_shared<ChColorAsset>();
  color->SetColor(ChColor(0.2f, 0.25f, 0.25f));
  floorBody->AddAsset(color);

  // Optionally, attach a texture to the pendulum, for better visualization
  auto texture = std::make_shared<ChTexture>();
  texture->SetTextureFilename(GetChronoDataFile("cubetexture_bluwhite.png"));  // texture in ../data
  pendulumBody->AddAsset(texture);

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
