#ifndef RIGHTOFWAYANALYZERTESTS_H
#define RIGHTOFWAYANALYZERTESTS_H

#include "IRightOfWayAnalyzer.h"
#include "RightOfWayAnalyzerNESWTests.h"
#include "RightOfWayAnalyzerESWTests.h"
#include "RightOfWayAnalyzerNESTests.h"
#include "RightOfWayAnalyzerNSWTests.h"

class RightOfWayAnalyzerTests
{
public:
  
  int runTests(IRightOfWayAnalyzer* analyzer);

private:
  
};

#endif // RIGHTOFWAYANALYZERTESTS_H
