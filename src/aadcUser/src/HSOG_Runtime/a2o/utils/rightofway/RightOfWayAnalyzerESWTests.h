#ifndef RIGHTOFWAYANALYZERESWTESTS_H
#define RIGHTOFWAYANALYZERESWTESTS_H

#include "IRightOfWayAnalyzer.h"

class RightOfWayAnalyzerESWTests
{
public:
  
  int runTests(IRightOfWayAnalyzer* analyzer);

private:
  
  int equals(int v1, int v2);
  
  
  /* Test naming convention
   * test_CrossingType_RoadSign_CarPositions_Destination
   */
  
  /* test_ESW_StVO102 */
  
  int test_ESW_StVO102__E(IRightOfWayAnalyzer* analyzer);
  int test_ESW_StVO102__W(IRightOfWayAnalyzer* analyzer);
  
  int test_ESW_StVO102_E_E(IRightOfWayAnalyzer* analyzer);
  int test_ESW_StVO102_E_W(IRightOfWayAnalyzer* analyzer);
  
  int test_ESW_StVO102_W_E(IRightOfWayAnalyzer* analyzer);
  int test_ESW_StVO102_W_W(IRightOfWayAnalyzer* analyzer);
    
  int test_ESW_StVO102_EW_E(IRightOfWayAnalyzer* analyzer);
  int test_ESW_StVO102_EW_W(IRightOfWayAnalyzer* analyzer);
  
  /* test_ESW_StVO206 */
  
  int test_ESW_StVO206__E(IRightOfWayAnalyzer* analyzer);
  int test_ESW_StVO206__W(IRightOfWayAnalyzer* analyzer);
  
  int test_ESW_StVO206_E_E(IRightOfWayAnalyzer* analyzer);
  int test_ESW_StVO206_E_W(IRightOfWayAnalyzer* analyzer);
  
  int test_ESW_StVO206_W_E(IRightOfWayAnalyzer* analyzer);
  int test_ESW_StVO206_W_W(IRightOfWayAnalyzer* analyzer);
  
  int test_ESW_StVO206_EW_E(IRightOfWayAnalyzer* analyzer);
  int test_ESW_StVO206_EW_W(IRightOfWayAnalyzer* analyzer);
  
  /* test_ESW_StVO205 */
  
  int test_ESW_StVO205__E(IRightOfWayAnalyzer* analyzer);
  int test_ESW_StVO205__W(IRightOfWayAnalyzer* analyzer);
  
  int test_ESW_StVO205_E_E(IRightOfWayAnalyzer* analyzer);
  int test_ESW_StVO205_E_W(IRightOfWayAnalyzer* analyzer);
  
  int test_ESW_StVO205_W_E(IRightOfWayAnalyzer* analyzer);
  int test_ESW_StVO205_W_W(IRightOfWayAnalyzer* analyzer);

  int test_ESW_StVO205_EW_E(IRightOfWayAnalyzer* analyzer);
  int test_ESW_StVO205_EW_W(IRightOfWayAnalyzer* analyzer);
  
  /* test_ESW_StVO301 */
  
  int test_ESW_StVO301__E(IRightOfWayAnalyzer* analyzer);
  int test_ESW_StVO301__W(IRightOfWayAnalyzer* analyzer);
  
  int test_ESW_StVO301_E_E(IRightOfWayAnalyzer* analyzer);
  int test_ESW_StVO301_E_W(IRightOfWayAnalyzer* analyzer);
  
  int test_ESW_StVO301_W_E(IRightOfWayAnalyzer* analyzer);
  int test_ESW_StVO301_W_W(IRightOfWayAnalyzer* analyzer);
    
  int test_ESW_StVO301_EW_E(IRightOfWayAnalyzer* analyzer);
  int test_ESW_StVO301_EW_W(IRightOfWayAnalyzer* analyzer);
  
  /* test_ESW_StVO20930 */
  
  int test_ESW_StVO20930__E(IRightOfWayAnalyzer* analyzer);
  int test_ESW_StVO20930__W(IRightOfWayAnalyzer* analyzer);
  
  int test_ESW_StVO20930_E_E(IRightOfWayAnalyzer* analyzer);
  int test_ESW_StVO20930_E_W(IRightOfWayAnalyzer* analyzer);
  
  int test_ESW_StVO20930_W_E(IRightOfWayAnalyzer* analyzer);
  int test_ESW_StVO20930_W_W(IRightOfWayAnalyzer* analyzer);

  int test_ESW_StVO20930_EW_E(IRightOfWayAnalyzer* analyzer);
  int test_ESW_StVO20930_EW_W(IRightOfWayAnalyzer* analyzer);
  
};

#endif // RIGHTOFWAYANALYZERESWTESTS_H
