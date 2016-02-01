#include "RightOfWayAnalyzerTests.h"

int RightOfWayAnalyzerTests::runTests(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;

  /* test_NESW_ */
  
  RightOfWayAnalyzerNESWTests neswTests;
  err += neswTests.runTests(analyzer);
  
  /* test_ESW_ */
  
  RightOfWayAnalyzerESWTests eswTests;
  err += eswTests.runTests(analyzer);
  
  /* test_NES_ */
  
  RightOfWayAnalyzerNESTests nesTests;
  err += nesTests.runTests(analyzer);
  
  /* test_NSW_ */
  
  RightOfWayAnalyzerNSWTests nswTests;
  err += nswTests.runTests(analyzer);
      
  return err;
}
