#ifndef SDF_LOADER_HPP
#define SDF_LOADER_HPP

#include <string>
#include "chrono/physics/ChSystemNSC.h"

bool loadSDF(chrono::ChSystemNSC& mphysicalSystem, std::string const& sdfPath);

#endif // SDF_LOADER_HPP
