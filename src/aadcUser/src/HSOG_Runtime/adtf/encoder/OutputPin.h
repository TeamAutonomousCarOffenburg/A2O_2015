#pragma once

#include <string>


namespace A2O {
  
struct OutputPin {
  std::string name;
  std::string signalType;

  OutputPin(){};
  OutputPin(std::string _name,
	    std::string _signalType
	   ) : name(_name), signalType(_signalType){};

  bool operator==(const OutputPin& other) const
  {
    return name == other.name && signalType == other.signalType;
  }
};

}
