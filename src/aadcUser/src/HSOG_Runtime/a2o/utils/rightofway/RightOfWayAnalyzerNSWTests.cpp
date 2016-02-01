#include "RightOfWayAnalyzerNSWTests.h"

int RightOfWayAnalyzerNSWTests::runTests(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  /* test_NSW_StVO102 */
  
  err += test_NSW_StVO102__W(analyzer);
  err += test_NSW_StVO102__N(analyzer);
  
  err += test_NSW_StVO102_W_W(analyzer);
  err += test_NSW_StVO102_W_N(analyzer);
  
  err += test_NSW_StVO102_N_W(analyzer);
  err += test_NSW_StVO102_N_N(analyzer);
    
  err += test_NSW_StVO102_WN_W(analyzer);
  err += test_NSW_StVO102_WN_N(analyzer);
  
  /* test_NSW_StVO206 */
  
  err += test_NSW_StVO206__W(analyzer);
  err += test_NSW_StVO206__N(analyzer);
  
  err += test_NSW_StVO206_W_W(analyzer);
  err += test_NSW_StVO206_W_N(analyzer);
  
  err += test_NSW_StVO206_N_W(analyzer);
  err += test_NSW_StVO206_N_N(analyzer);
    
  err += test_NSW_StVO206_WN_W(analyzer);
  err += test_NSW_StVO206_WN_N(analyzer);
  
  /* test_NSW_StVO205 */
  
  err += test_NSW_StVO205__W(analyzer);
  err += test_NSW_StVO205__N(analyzer);
  
  err += test_NSW_StVO205_W_W(analyzer);
  err += test_NSW_StVO205_W_N(analyzer);
  
  err += test_NSW_StVO205_N_W(analyzer);
  err += test_NSW_StVO205_N_N(analyzer);
  
  err += test_NSW_StVO205_WN_W(analyzer);
  err += test_NSW_StVO205_WN_N(analyzer);
  
  /* test_NSW_StVO301 */
  
  err += test_NSW_StVO301__W(analyzer);
  err += test_NSW_StVO301__N(analyzer);
  
  err += test_NSW_StVO301_W_W(analyzer);
  err += test_NSW_StVO301_W_N(analyzer);
  
  err += test_NSW_StVO301_N_W(analyzer);
  err += test_NSW_StVO301_N_N(analyzer);
  
  err += test_NSW_StVO301_WN_W(analyzer);
  err += test_NSW_StVO301_WN_N(analyzer);
  
  /* test_NSW_StVO20930 */
  
  err += test_NSW_StVO20930__W(analyzer);
  err += test_NSW_StVO20930__N(analyzer);
  
  err += test_NSW_StVO20930_W_W(analyzer);
  err += test_NSW_StVO20930_W_N(analyzer);
  
  err += test_NSW_StVO20930_N_W(analyzer);
  err += test_NSW_StVO20930_N_N(analyzer);
  
  err += test_NSW_StVO20930_WN_W(analyzer);
  err += test_NSW_StVO20930_WN_N(analyzer);
      
  return err;
}

int RightOfWayAnalyzerNSWTests::equals(int v1, int v2)
{
 if(v1 != v2)
 {
   return 1; 
 }
 else
 {
   return 0;
 }
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO102__W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO102__N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO102_W_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO102_W_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO102_N_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO102_N_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO102_WN_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO102_WN_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO206__W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::StopThenDrive);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO206__N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::StopThenDrive);
  
  return err;
}


int RightOfWayAnalyzerNSWTests::test_NSW_StVO206_W_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO206_W_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO206_N_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO206_N_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}


int RightOfWayAnalyzerNSWTests::test_NSW_StVO206_WN_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO206_WN_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO205__W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO205__N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO205_W_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO205_W_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO205_N_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO205_N_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}


int RightOfWayAnalyzerNSWTests::test_NSW_StVO205_WN_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO205_WN_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO301__W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO301__N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO301_W_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO301_W_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO301_N_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO301_N_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO301_WN_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO301_WN_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO20930__W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::NotPermitted);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO20930__N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO20930_W_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::NotPermitted);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO20930_W_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO20930_N_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::NotPermitted);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO20930_N_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO20930_WN_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::NotPermitted);
  
  return err;
}

int RightOfWayAnalyzerNSWTests::test_NSW_StVO20930_WN_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NSWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}
