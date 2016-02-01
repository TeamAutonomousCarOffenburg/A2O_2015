#pragma once

#include "perception/IJuryPerceptor.h"

#include <vector>
#include <boost/smart_ptr.hpp>

namespace A2O{

class DriveInstructionManager {
public:
  typedef boost::shared_ptr<DriveInstructionManager> Ptr;
  typedef boost::shared_ptr<const DriveInstructionManager> ConstPtr;

  DriveInstructionManager();
  DriveInstructionManager(std::vector<std::string> instructionList);
  DriveInstructionManager(const std::string& filename);
  
  const size_t& getCurrentInstructionIndex() const;
  size_t getNumberOfInstructions() const;
  const std::string& getInstruction(const size_t& idx) const;
  const std::string& getCurrentInstruction() const;
  
  void progressInstructionIndex();
  void setStartInstructionIndex(int startIndex);

private:
  size_t _currentInstructionIndex;

  // The jury filter gives us the index where to start
  int _startInstructionIndex;
  
  std::vector<std::string> _instructionList;
};

}
