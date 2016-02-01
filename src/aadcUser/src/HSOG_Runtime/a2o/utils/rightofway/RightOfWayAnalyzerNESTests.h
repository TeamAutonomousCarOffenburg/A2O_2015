#ifndef RIGHTOFWAYANALYZERNESTESTS_H
#define RIGHTOFWAYANALYZERNESTESTS_H

#include "IRightOfWayAnalyzer.h"

class RightOfWayAnalyzerNESTests
{
public:
  
  int runTests(IRightOfWayAnalyzer* analyzer);

private:
  
  int equals(int v1, int v2);
  
  
  /* Test naming convention
   * test_CrossingType_RoadSign_CarPositions_Destination
   */
  
  /* test_NES_StVO102 */
  
  int test_NES_StVO102__E(IRightOfWayAnalyzer* analyzer);
  int test_NES_StVO102__N(IRightOfWayAnalyzer* analyzer);
  
  int test_NES_StVO102_E_E(IRightOfWayAnalyzer* analyzer);
  int test_NES_StVO102_E_N(IRightOfWayAnalyzer* analyzer);
  
  int test_NES_StVO102_N_E(IRightOfWayAnalyzer* analyzer);
  int test_NES_StVO102_N_N(IRightOfWayAnalyzer* analyzer);
    
  int test_NES_StVO102_EN_E(IRightOfWayAnalyzer* analyzer);
  int test_NES_StVO102_EN_N(IRightOfWayAnalyzer* analyzer);
  
  /* test_NES_StVO206 */
  
  int test_NES_StVO206__E(IRightOfWayAnalyzer* analyzer);
  int test_NES_StVO206__N(IRightOfWayAnalyzer* analyzer);
  
  int test_NES_StVO206_E_E(IRightOfWayAnalyzer* analyzer);
  int test_NES_StVO206_E_N(IRightOfWayAnalyzer* analyzer);
  
  int test_NES_StVO206_N_E(IRightOfWayAnalyzer* analyzer);
  int test_NES_StVO206_N_N(IRightOfWayAnalyzer* analyzer);
  
  int test_NES_StVO206_EN_E(IRightOfWayAnalyzer* analyzer);
  int test_NES_StVO206_EN_N(IRightOfWayAnalyzer* analyzer);
  
  /* test_NES_StVO205 */
  
  int test_NES_StVO205__E(IRightOfWayAnalyzer* analyzer);
  int test_NES_StVO205__N(IRightOfWayAnalyzer* analyzer);
  
  int test_NES_StVO205_E_E(IRightOfWayAnalyzer* analyzer);
  int test_NES_StVO205_E_N(IRightOfWayAnalyzer* analyzer);
  
  int test_NES_StVO205_N_E(IRightOfWayAnalyzer* analyzer);
  int test_NES_StVO205_N_N(IRightOfWayAnalyzer* analyzer);

  int test_NES_StVO205_EN_E(IRightOfWayAnalyzer* analyzer);
  int test_NES_StVO205_EN_N(IRightOfWayAnalyzer* analyzer);
  
  /* test_NES_StVO301 */
  
  int test_NES_StVO301__E(IRightOfWayAnalyzer* analyzer);
  int test_NES_StVO301__N(IRightOfWayAnalyzer* analyzer);
  
  int test_NES_StVO301_E_E(IRightOfWayAnalyzer* analyzer);
  int test_NES_StVO301_E_N(IRightOfWayAnalyzer* analyzer);
  
  int test_NES_StVO301_N_E(IRightOfWayAnalyzer* analyzer);
  int test_NES_StVO301_N_N(IRightOfWayAnalyzer* analyzer);
    
  int test_NES_StVO301_EN_E(IRightOfWayAnalyzer* analyzer);
  int test_NES_StVO301_EN_N(IRightOfWayAnalyzer* analyzer);
  
  /* test_NES_StVO20930 */
  
  int test_NES_StVO20930__E(IRightOfWayAnalyzer* analyzer);
  int test_NES_StVO20930__N(IRightOfWayAnalyzer* analyzer);
  
  int test_NES_StVO20930_E_E(IRightOfWayAnalyzer* analyzer);
  int test_NES_StVO20930_E_N(IRightOfWayAnalyzer* analyzer);
  
  int test_NES_StVO20930_N_E(IRightOfWayAnalyzer* analyzer);
  int test_NES_StVO20930_N_N(IRightOfWayAnalyzer* analyzer);

  int test_NES_StVO20930_EN_E(IRightOfWayAnalyzer* analyzer);
  int test_NES_StVO20930_EN_N(IRightOfWayAnalyzer* analyzer);
  
};

#endif // RIGHTOFWAYANALYZERNESTESTS_H
