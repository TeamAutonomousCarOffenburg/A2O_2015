#pragma once

#include <string>
#include <vector>

namespace A2O {

class DriveInstructionReader {
public:
  static void load(const std::string& filename,
                   std::vector<std::string>& driveInstructions);
};

}
