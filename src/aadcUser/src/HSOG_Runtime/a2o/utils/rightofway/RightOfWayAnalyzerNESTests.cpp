#include "RightOfWayAnalyzerNESTests.h"

int RightOfWayAnalyzerNESTests::runTests(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  /* test_NES_StVO102 */
  
  err += test_NES_StVO102__E(analyzer);
  err += test_NES_StVO102__N(analyzer);
  
  err += test_NES_StVO102_E_E(analyzer);
  err += test_NES_StVO102_E_N(analyzer);
  
  err += test_NES_StVO102_N_E(analyzer);
  err += test_NES_StVO102_N_N(analyzer);
    
  err += test_NES_StVO102_EN_E(analyzer);
  err += test_NES_StVO102_EN_N(analyzer);
  
  /* test_NES_StVO206 */
  
  err += test_NES_StVO206__E(analyzer);
  err += test_NES_StVO206__N(analyzer);
  
  err += test_NES_StVO206_E_E(analyzer);
  err += test_NES_StVO206_E_N(analyzer);
  
  err += test_NES_StVO206_N_E(analyzer);
  err += test_NES_StVO206_N_N(analyzer);
    
  err += test_NES_StVO206_EN_E(analyzer);
  err += test_NES_StVO206_EN_N(analyzer);
  
  /* test_NES_StVO205 */
  
  err += test_NES_StVO205__E(analyzer);
  err += test_NES_StVO205__N(analyzer);
  
  err += test_NES_StVO205_E_E(analyzer);
  err += test_NES_StVO205_E_N(analyzer);
  
  err += test_NES_StVO205_N_E(analyzer);
  err += test_NES_StVO205_N_N(analyzer);
  
  err += test_NES_StVO205_EN_E(analyzer);
  err += test_NES_StVO205_EN_N(analyzer);
  
  /* test_NES_StVO301 */
  
  err += test_NES_StVO301__E(analyzer);
  err += test_NES_StVO301__N(analyzer);
  
  err += test_NES_StVO301_E_E(analyzer);
  err += test_NES_StVO301_E_N(analyzer);
  
  err += test_NES_StVO301_N_E(analyzer);
  err += test_NES_StVO301_N_N(analyzer);
  
  err += test_NES_StVO301_EN_E(analyzer);
  err += test_NES_StVO301_EN_N(analyzer);
  
  /* test_NES_StVO20930 */
  
  err += test_NES_StVO20930__E(analyzer);
  err += test_NES_StVO20930__N(analyzer);
  
  err += test_NES_StVO20930_E_E(analyzer);
  err += test_NES_StVO20930_E_N(analyzer);
  
  err += test_NES_StVO20930_N_E(analyzer);
  err += test_NES_StVO20930_N_N(analyzer);
  
  err += test_NES_StVO20930_EN_E(analyzer);
  err += test_NES_StVO20930_EN_N(analyzer);
      
  return err;
}

int RightOfWayAnalyzerNESTests::equals(int v1, int v2)
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

int RightOfWayAnalyzerNESTests::test_NES_StVO102__E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO102__N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
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

int RightOfWayAnalyzerNESTests::test_NES_StVO102_E_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO102_E_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO102_N_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO102_N_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
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

int RightOfWayAnalyzerNESTests::test_NES_StVO102_EN_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO102_EN_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO206__E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::StopThenDrive);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO206__N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
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


int RightOfWayAnalyzerNESTests::test_NES_StVO206_E_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO206_E_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO206_N_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO206_N_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
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


int RightOfWayAnalyzerNESTests::test_NES_StVO206_EN_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO206_EN_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO205__E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO205__N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
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

int RightOfWayAnalyzerNESTests::test_NES_StVO205_E_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO205_E_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO205_N_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO205_N_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
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


int RightOfWayAnalyzerNESTests::test_NES_StVO205_EN_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO205_EN_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO301__E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO301__N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
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

int RightOfWayAnalyzerNESTests::test_NES_StVO301_E_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO301_E_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
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

int RightOfWayAnalyzerNESTests::test_NES_StVO301_N_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO301_N_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
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

int RightOfWayAnalyzerNESTests::test_NES_StVO301_EN_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Drive);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO301_EN_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
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

int RightOfWayAnalyzerNESTests::test_NES_StVO20930__E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::NotPermitted);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO20930__N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
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

int RightOfWayAnalyzerNESTests::test_NES_StVO20930_E_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::NotPermitted);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO20930_E_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO20930_N_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::NotPermitted);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO20930_N_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
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

int RightOfWayAnalyzerNESTests::test_NES_StVO20930_EN_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::NotPermitted);
  
  return err;
}

int RightOfWayAnalyzerNESTests::test_NES_StVO20930_EN_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}
