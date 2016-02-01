#include "DriveInstructionManager.h"
#include "worldmodel/IDriveInstructionConstants.h"
#include "utils/driveinstructionreader/DriveInstructionReader.h"

#include <iostream>


using namespace A2O;

DriveInstructionManager::DriveInstructionManager()
      : _currentInstructionIndex(0), _startInstructionIndex(-1)
{
}

DriveInstructionManager::DriveInstructionManager(std::vector<std::string> instructionList)
      : _currentInstructionIndex(0), _startInstructionIndex(-1), _instructionList(instructionList)
{
}

DriveInstructionManager::DriveInstructionManager(const std::string& filename)
      : DriveInstructionManager()
{
  DriveInstructionReader::load(filename, _instructionList);
}

const size_t& DriveInstructionManager::getCurrentInstructionIndex() const
{
  return _currentInstructionIndex;
}

size_t DriveInstructionManager::getNumberOfInstructions() const
{
  return _instructionList.size();
}

const std::string& DriveInstructionManager::getInstruction(const size_t& idx) const
{
  if (idx >= _instructionList.size()) {
    return DRIVE_INSTRUCTION::DRIVE_STRAIGHT;
  }
  
  return _instructionList[idx];
}

const std::string& DriveInstructionManager::getCurrentInstruction() const
{
  return getInstruction(_currentInstructionIndex);
}

void DriveInstructionManager::progressInstructionIndex()
{
  _currentInstructionIndex++;
}

void DriveInstructionManager::setStartInstructionIndex(int startIndex)
{
  if (_startInstructionIndex != startIndex) {
    _startInstructionIndex = startIndex;
    _currentInstructionIndex = startIndex;
  }
}
