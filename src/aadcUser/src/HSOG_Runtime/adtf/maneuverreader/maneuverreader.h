#pragma once

#include <string>
#include <iostream>
#include <vector>


#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

struct AADC_Maneuver {
	int id;
	string action;
};

struct Sector{
	int id;
	vector<AADC_Maneuver> maneuverList;
};



class ManeuverReader {

	public:
		static std::vector<Sector> readFile(std::string filename)
		{
			ptree tree;
			read_xml(filename, tree);

			vector<Sector> sectors;

			for( const boost::property_tree::ptree::value_type & v : tree.get_child("AADC-Maneuver-List") ) {
				if( v.first == "AADC-Sector" ) {
					Sector sector;
					sector.id = v.second.get<int>("<xmlattr>.id", -1);

					std::cout << "found sector " << sector.id << std::endl;

					for(const ptree::value_type& maneuver : v.second) {

						if(maneuver.first == "AADC-Maneuver")
						{
							AADC_Maneuver man;
							man.id = maneuver.second.get<int>("<xmlattr>.id", -1);
							man.action = maneuver.second.get<string>("<xmlattr>.action");
							cout << "found maneuver " << man.id << " " << man.action << endl;
							sector.maneuverList.push_back(man);
						}	
					}
					sectors.push_back(sector);
				}
			}
			return sectors;
		}
};
