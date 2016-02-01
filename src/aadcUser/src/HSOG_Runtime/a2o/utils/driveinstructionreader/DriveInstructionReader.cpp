#include "DriveInstructionReader.h"

#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

using namespace A2O;
using namespace boost;
using namespace boost::property_tree;


void DriveInstructionReader::load(const std::string& filename,
				  std::vector<std::string>& driveInstructions)
{
  // Clear maybe present instructions
  driveInstructions.clear();

  ptree tree;

  try {
    read_xml(filename, tree);
  } catch(...) {
    std::cerr << "Could not read maneuver list." << std::endl;
    return;
  }

  for (const ptree::value_type& v : tree.get_child("AADC-Maneuver-List")) {
    if (v.first == "AADC-Sector") {
      for (const ptree::value_type& maneuver : v.second) {
        if (maneuver.first == "AADC-Maneuver") {
          int id = maneuver.second.get<int>("<xmlattr>.id", -1);
          if (id != driveInstructions.size()) {
            std::cerr << "OOOPS!!! Maneuver ID in maneuver-xml file NOT strictly increasing by one!!!" << std::endl;
          }
          std::string instruction = maneuver.second.get<std::string>("<xmlattr>.action");
          std::cout << "Found drive instruction " << " (index): " << driveInstructions.size() << " (id): " << id << " " << instruction << std::endl;
          driveInstructions.push_back(instruction);
        }	
      }
    }
  }
}
