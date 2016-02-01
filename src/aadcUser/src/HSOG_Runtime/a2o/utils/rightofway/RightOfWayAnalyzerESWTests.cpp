#include "RightOfWayAnalyzerESWTests.h"

int RightOfWayAnalyzerESWTests::runTests(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  /* test_ESW_StVO102 */
  
  err += test_ESW_StVO102__E(analyzer);
  err += test_ESW_StVO102__W(analyzer);
  
  err += test_ESW_StVO102_E_E(analyzer);
  err += test_ESW_StVO102_E_W(analyzer);
  
  err += test_ESW_StVO102_W_E(analyzer);
  err += test_ESW_StVO102_W_W(analyzer);
    
  err += test_ESW_StVO102_EW_E(analyzer);
  err += test_ESW_StVO102_EW_W(analyzer);
  
  /* test_ESW_StVO206 */
  
  err += test_ESW_StVO206__E(analyzer);
  err += test_ESW_StVO206__W(analyzer);
  
  err += test_ESW_StVO206_E_E(analyzer);
  err += test_ESW_StVO206_E_W(analyzer);
  
  err += test_ESW_StVO206_W_E(analyzer);
  err += test_ESW_StVO206_W_W(analyzer);
    
  err += test_ESW_StVO206_EW_E(analyzer);
  err += test_ESW_StVO206_EW_W(analyzer);
  
  /* test_ESW_StVO205 */
  
  err += test_ESW_StVO205__E(analyzer);
  err += test_ESW_StVO205__W(analyzer);
  
  err += test_ESW_StVO205_E_E(analyzer);
  err += test_ESW_StVO205_E_W(analyzer);
  
  err += test_ESW_StVO205_W_E(analyzer);
  err += test_ESW_StVO205_W_W(analyzer);
  
  err += test_ESW_StVO205_EW_E(analyzer);
  err += test_ESW_StVO205_EW_W(analyzer);
  
  /* test_ESW_StVO301 */
  
  err += test_ESW_StVO301__E(analyzer);
  err += test_ESW_StVO301__W(analyzer);
  
  err += test_ESW_StVO301_E_E(analyzer);
  err += test_ESW_StVO301_E_W(analyzer);
  
  err += test_ESW_StVO301_W_E(analyzer);
  err += test_ESW_StVO301_W_W(analyzer);
  
  err += test_ESW_StVO301_EW_E(analyzer);
  err += test_ESW_StVO301_EW_W(analyzer);
  
  /* test_ESW_StVO20930 */
  
  err += test_ESW_StVO20930__E(analyzer);
  err += test_ESW_StVO20930__W(analyzer);
  
  err += test_ESW_StVO20930_E_E(analyzer);
  err += test_ESW_StVO20930_E_W(analyzer);
  
  err += test_ESW_StVO20930_W_E(analyzer);
  err += test_ESW_StVO20930_W_W(analyzer);
  
  err += test_ESW_StVO20930_EW_E(analyzer);
  err += test_ESW_StVO20930_EW_W(analyzer);
      
  return err;
}

int RightOfWayAnalyzerESWTests::equals(int v1, int v2)
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO102__E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO102__W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO102_E_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO102_E_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO102_W_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO102_W_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO102_EW_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO102_EW_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO206__E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO206__W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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


int RightOfWayAnalyzerESWTests::test_ESW_StVO206_E_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO206_E_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO206_W_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO206_W_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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


int RightOfWayAnalyzerESWTests::test_ESW_StVO206_EW_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO206_EW_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO205__E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO205__W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO205_E_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO205_E_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO205_W_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO205_W_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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


int RightOfWayAnalyzerESWTests::test_ESW_StVO205_EW_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO205_EW_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO301__E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO301__W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO301_E_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO301_E_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO301_W_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO301_W_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO301_EW_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO301_EW_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO20930__E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO20930__W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO20930_E_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO20930_E_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO20930_W_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO20930_W_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO20930_EW_E(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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

int RightOfWayAnalyzerESWTests::test_ESW_StVO20930_EW_W(IRightOfWayAnalyzer* analyzer)
{
  int err = 0;
  
  // Create crossing
  CrossingType_t crossingType = ESWCrossing;
  
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
