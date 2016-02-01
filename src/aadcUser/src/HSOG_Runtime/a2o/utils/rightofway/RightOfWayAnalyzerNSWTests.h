#ifndef RIGHTOFWAYANALYZERNSWTESTS_H
#define RIGHTOFWAYANALYZERNSWTESTS_H

#include "IRightOfWayAnalyzer.h"

class RightOfWayAnalyzerNSWTests
{
public:
  
  int runTests(IRightOfWayAnalyzer* analyzer);

private:
  
  int equals(int v1, int v2);
  
  
  /* Test naming convention
   * test_CrossingType_RoadSign_CarPositions_Destination
   */
  
  /* test_NSW_StVO102 */
  
  int test_NSW_StVO102__W(IRightOfWayAnalyzer* analyzer);
  int test_NSW_StVO102__N(IRightOfWayAnalyzer* analyzer);
  
  int test_NSW_StVO102_W_W(IRightOfWayAnalyzer* analyzer);
  int test_NSW_StVO102_W_N(IRightOfWayAnalyzer* analyzer);
  
  int test_NSW_StVO102_N_W(IRightOfWayAnalyzer* analyzer);
  int test_NSW_StVO102_N_N(IRightOfWayAnalyzer* analyzer);
    
  int test_NSW_StVO102_WN_W(IRightOfWayAnalyzer* analyzer);
  int test_NSW_StVO102_WN_N(IRightOfWayAnalyzer* analyzer);
  
  /* test_NSW_StVO206 */
  
  int test_NSW_StVO206__W(IRightOfWayAnalyzer* analyzer);
  int test_NSW_StVO206__N(IRightOfWayAnalyzer* analyzer);
  
  int test_NSW_StVO206_W_W(IRightOfWayAnalyzer* analyzer);
  int test_NSW_StVO206_W_N(IRightOfWayAnalyzer* analyzer);
  
  int test_NSW_StVO206_N_W(IRightOfWayAnalyzer* analyzer);
  int test_NSW_StVO206_N_N(IRightOfWayAnalyzer* analyzer);
  
  int test_NSW_StVO206_WN_W(IRightOfWayAnalyzer* analyzer);
  int test_NSW_StVO206_WN_N(IRightOfWayAnalyzer* analyzer);
  
  /* test_NSW_StVO205 */
  
  int test_NSW_StVO205__W(IRightOfWayAnalyzer* analyzer);
  int test_NSW_StVO205__N(IRightOfWayAnalyzer* analyzer);
  
  int test_NSW_StVO205_W_W(IRightOfWayAnalyzer* analyzer);
  int test_NSW_StVO205_W_N(IRightOfWayAnalyzer* analyzer);
  
  int test_NSW_StVO205_N_W(IRightOfWayAnalyzer* analyzer);
  int test_NSW_StVO205_N_N(IRightOfWayAnalyzer* analyzer);

  int test_NSW_StVO205_WN_W(IRightOfWayAnalyzer* analyzer);
  int test_NSW_StVO205_WN_N(IRightOfWayAnalyzer* analyzer);
  
  /* test_NSW_StVO301 */
  
  int test_NSW_StVO301__W(IRightOfWayAnalyzer* analyzer);
  int test_NSW_StVO301__N(IRightOfWayAnalyzer* analyzer);
  
  int test_NSW_StVO301_W_W(IRightOfWayAnalyzer* analyzer);
  int test_NSW_StVO301_W_N(IRightOfWayAnalyzer* analyzer);
  
  int test_NSW_StVO301_N_W(IRightOfWayAnalyzer* analyzer);
  int test_NSW_StVO301_N_N(IRightOfWayAnalyzer* analyzer);
    
  int test_NSW_StVO301_WN_W(IRightOfWayAnalyzer* analyzer);
  int test_NSW_StVO301_WN_N(IRightOfWayAnalyzer* analyzer);
  
  /* test_NSW_StVO20930 */
  
  int test_NSW_StVO20930__W(IRightOfWayAnalyzer* analyzer);
  int test_NSW_StVO20930__N(IRightOfWayAnalyzer* analyzer);
  
  int test_NSW_StVO20930_W_W(IRightOfWayAnalyzer* analyzer);
  int test_NSW_StVO20930_W_N(IRightOfWayAnalyzer* analyzer);
  
  int test_NSW_StVO20930_N_W(IRightOfWayAnalyzer* analyzer);
  int test_NSW_StVO20930_N_N(IRightOfWayAnalyzer* analyzer);

  int test_NSW_StVO20930_WN_W(IRightOfWayAnalyzer* analyzer);
  int test_NSW_StVO20930_WN_N(IRightOfWayAnalyzer* analyzer);
  
};

#endif // RIGHTOFWAYANALYZERNSWTESTS_H
