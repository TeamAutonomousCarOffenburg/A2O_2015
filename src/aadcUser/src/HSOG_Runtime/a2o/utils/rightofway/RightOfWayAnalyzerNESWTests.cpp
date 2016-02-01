#include "RightOfWayAnalyzerNESWTests.h"

int RightOfWayAnalyzerNESWTests::runTests(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;

  /* test_NESW_StVO102 */
  
  err += test_NESW_StVO102__N(analyzer);
  err += test_NESW_StVO102__E(analyzer);
  err += test_NESW_StVO102__W(analyzer);
  
  err += test_NESW_StVO102_E_N(analyzer);
  err += test_NESW_StVO102_E_E(analyzer);
  err += test_NESW_StVO102_E_W(analyzer);
  
  err += test_NESW_StVO102_W_N(analyzer);
  err += test_NESW_StVO102_W_E(analyzer);
  err += test_NESW_StVO102_W_W(analyzer);
  
  err += test_NESW_StVO102_N_N(analyzer);
  err += test_NESW_StVO102_N_E(analyzer);
  err += test_NESW_StVO102_N_W(analyzer);
  
  err += test_NESW_StVO102_NW_N(analyzer);
  err += test_NESW_StVO102_NW_E(analyzer);
  err += test_NESW_StVO102_NW_W(analyzer);
  
  err += test_NESW_StVO102_NE_N(analyzer);
  err += test_NESW_StVO102_NE_E(analyzer);
  err += test_NESW_StVO102_NE_W(analyzer);
  
  err += test_NESW_StVO102_EW_N(analyzer);
  err += test_NESW_StVO102_EW_E(analyzer);
  err += test_NESW_StVO102_EW_W(analyzer);
  
  err += test_NESW_StVO102_NEW_N(analyzer);
  err += test_NESW_StVO102_NEW_E(analyzer);
  err += test_NESW_StVO102_NEW_W(analyzer);
  
  /* test_NESW_StVO206 */
  
  err += test_NESW_StVO206__N(analyzer);
  err += test_NESW_StVO206__E(analyzer);
  err += test_NESW_StVO206__W(analyzer);
  
  err += test_NESW_StVO206_E_N(analyzer);
  err += test_NESW_StVO206_E_E(analyzer);
  err += test_NESW_StVO206_E_W(analyzer);
  
  err += test_NESW_StVO206_W_N(analyzer);
  err += test_NESW_StVO206_W_E(analyzer);
  err += test_NESW_StVO206_W_W(analyzer);
  
  err += test_NESW_StVO206_N_N(analyzer);
  err += test_NESW_StVO206_N_E(analyzer);
  err += test_NESW_StVO206_N_W(analyzer);
  
  err += test_NESW_StVO206_NW_N(analyzer);
  err += test_NESW_StVO206_NW_E(analyzer);
  err += test_NESW_StVO206_NW_W(analyzer);
  
  err += test_NESW_StVO206_NE_N(analyzer);
  err += test_NESW_StVO206_NE_E(analyzer);
  err += test_NESW_StVO206_NE_W(analyzer);
  
  err += test_NESW_StVO206_EW_N(analyzer);
  err += test_NESW_StVO206_EW_E(analyzer);
  err += test_NESW_StVO206_EW_W(analyzer);
  
  err += test_NESW_StVO206_NEW_N(analyzer);
  err += test_NESW_StVO206_NEW_E(analyzer);
  err += test_NESW_StVO206_NEW_W(analyzer);
  
  /* test_NESW_StVO205 */
  
  err += test_NESW_StVO205__N(analyzer);
  err += test_NESW_StVO205__E(analyzer);
  err += test_NESW_StVO205__W(analyzer);
  
  err += test_NESW_StVO205_E_N(analyzer);
  err += test_NESW_StVO205_E_E(analyzer);
  err += test_NESW_StVO205_E_W(analyzer);
  
  err += test_NESW_StVO205_W_N(analyzer);
  err += test_NESW_StVO205_W_E(analyzer);
  err += test_NESW_StVO205_W_W(analyzer);
  
  err += test_NESW_StVO205_N_N(analyzer);
  err += test_NESW_StVO205_N_E(analyzer);
  err += test_NESW_StVO205_N_W(analyzer);
  
  err += test_NESW_StVO205_NW_N(analyzer);
  err += test_NESW_StVO205_NW_E(analyzer);
  err += test_NESW_StVO205_NW_W(analyzer);
  
  err += test_NESW_StVO205_NE_N(analyzer);
  err += test_NESW_StVO205_NE_E(analyzer);
  err += test_NESW_StVO205_NE_W(analyzer);
  
  err += test_NESW_StVO205_EW_N(analyzer);
  err += test_NESW_StVO205_EW_E(analyzer);
  err += test_NESW_StVO205_EW_W(analyzer);
  
  err += test_NESW_StVO205_NEW_N(analyzer);
  err += test_NESW_StVO205_NEW_E(analyzer);
  err += test_NESW_StVO205_NEW_W(analyzer);
  
  /* test_NESW_StVO301 */
  
  err += test_NESW_StVO301__N(analyzer);
  err += test_NESW_StVO301__E(analyzer);
  err += test_NESW_StVO301__W(analyzer);
  
  err += test_NESW_StVO301_E_N(analyzer);
  err += test_NESW_StVO301_E_E(analyzer);
  err += test_NESW_StVO301_E_W(analyzer);
  
  err += test_NESW_StVO301_W_N(analyzer);
  err += test_NESW_StVO301_W_E(analyzer);
  err += test_NESW_StVO301_W_W(analyzer);
  
  err += test_NESW_StVO301_N_N(analyzer);
  err += test_NESW_StVO301_N_E(analyzer);
  err += test_NESW_StVO301_N_W(analyzer);
  
  err += test_NESW_StVO301_NW_N(analyzer);
  err += test_NESW_StVO301_NW_E(analyzer);
  err += test_NESW_StVO301_NW_W(analyzer);
  
  err += test_NESW_StVO301_NE_N(analyzer);
  err += test_NESW_StVO301_NE_E(analyzer);
  err += test_NESW_StVO301_NE_W(analyzer);
  
  err += test_NESW_StVO301_EW_N(analyzer);
  err += test_NESW_StVO301_EW_E(analyzer);
  err += test_NESW_StVO301_EW_W(analyzer);
  
  err += test_NESW_StVO301_NEW_N(analyzer);
  err += test_NESW_StVO301_NEW_E(analyzer);
  err += test_NESW_StVO301_NEW_W(analyzer);
  
  /* test_NESW_StVO20930 */
  
  err += test_NESW_StVO20930__N(analyzer);
  err += test_NESW_StVO20930__E(analyzer);
  err += test_NESW_StVO20930__W(analyzer);
  
  err += test_NESW_StVO20930_E_N(analyzer);
  err += test_NESW_StVO20930_E_E(analyzer);
  err += test_NESW_StVO20930_E_W(analyzer);
  
  err += test_NESW_StVO20930_W_N(analyzer);
  err += test_NESW_StVO20930_W_E(analyzer);
  err += test_NESW_StVO20930_W_W(analyzer);
  
  err += test_NESW_StVO20930_N_N(analyzer);
  err += test_NESW_StVO20930_N_E(analyzer);
  err += test_NESW_StVO20930_N_W(analyzer);
  
  err += test_NESW_StVO20930_NW_N(analyzer);
  err += test_NESW_StVO20930_NW_E(analyzer);
  err += test_NESW_StVO20930_NW_W(analyzer);
  
  err += test_NESW_StVO20930_NE_N(analyzer);
  err += test_NESW_StVO20930_NE_E(analyzer);
  err += test_NESW_StVO20930_NE_W(analyzer);
  
  err += test_NESW_StVO20930_EW_N(analyzer);
  err += test_NESW_StVO20930_EW_E(analyzer);
  err += test_NESW_StVO20930_EW_W(analyzer);
  
  err += test_NESW_StVO20930_NEW_N(analyzer);
  err += test_NESW_StVO20930_NEW_E(analyzer);
  err += test_NESW_StVO20930_NEW_W(analyzer);
        
  return err;
}

int RightOfWayAnalyzerNESWTests::equals(int v1, int v2)
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102__N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102__E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102__W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_E_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_E_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_E_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_W_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_W_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = true;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_W_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_N_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_N_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_N_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_NW_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_NW_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = true;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_NW_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_NE_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_NE_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_NE_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_EW_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = true;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_EW_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = true;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_EW_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_NEW_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_NEW_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO102_NEW_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO102RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206__N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206__E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206__W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_E_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_E_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_E_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_W_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_W_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_W_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_N_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_N_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_N_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_NW_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_NW_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_NW_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_NE_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_NE_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_NE_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_EW_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_EW_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_EW_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_NEW_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_NEW_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO206_NEW_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO206RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205__N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205__E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205__W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_E_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_E_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_E_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_W_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_W_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_W_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_N_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_N_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_N_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_NW_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_NW_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, false);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_NW_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_NE_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_NE_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_NE_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_EW_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_EW_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_EW_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_NEW_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_NEW_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO205_NEW_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO205RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301__N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301__E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301__W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_E_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_E_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_E_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_W_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_W_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = true;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_W_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_N_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_N_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_N_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_NW_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_NW_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = true;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_NW_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_NE_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_NE_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_NE_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_EW_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_EW_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = true;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_EW_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_NEW_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_NEW_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = true;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO301_NEW_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO301RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930__N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930__E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930__W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_E_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_E_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_E_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::NotPermitted);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_W_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_W_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = false;
  carPositions.west = true;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_W_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_N_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_N_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_N_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_NW_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_NW_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = false;
  carPositions.west = true;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_NW_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::NotPermitted);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_NE_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_NE_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_NE_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = false;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::NotPermitted);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_EW_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = true;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_EW_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = true;
  
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

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_EW_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = false;
  carPositions.east = true;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, false);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, false);
  err += equals(situation.action, Action::NotPermitted);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_NEW_N(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = North;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::Wait);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_NEW_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = East;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::NotPermitted);
  
  return err;
}

int RightOfWayAnalyzerNESWTests::test_NESW_StVO20930_NEW_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = NESWCrossing;
  
  // Create road sign
  RoadSignType_t roadSignType = StVO20930RoadSign;
  
  // Create car positions
  CarPositions_t carPositions;
  carPositions.north = true;
  carPositions.east = true;
  carPositions.west = true;
  
  // Create drive destination
  DriveDestination_t driveDestination = West;
 
  // Analyze situation
  Situation_t situation = analyzer->AnalyzeSituation(crossingType, roadSignType, carPositions, driveDestination);
  
  // Test results
  err += equals(situation.rightOfWay.north, true);
  err += equals(situation.rightOfWay.east, true);
  err += equals(situation.rightOfWay.west, true);
  err += equals(situation.action, Action::NotPermitted);
  
  return err;
}

