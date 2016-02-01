#ifndef RIGHTOFWAYANALYZERNESWTESTS_H
#define RIGHTOFWAYANALYZERNESWTESTS_H

#include "IRightOfWayAnalyzer.h"

class RightOfWayAnalyzerNESWTests
{
public:
  
  int runTests(IRightOfWayAnalyzer* analyzer);

private:
  
  int equals(int v1, int v2);
  
  
  /* Test naming convention
   * test_CrossingType_RoadSign_CarPositions_Destination
   */

  /* test_NESW_StVO102 */
  
  int test_NESW_StVO102__N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO102__E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO102__W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO102_E_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO102_E_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO102_E_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO102_W_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO102_W_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO102_W_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO102_N_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO102_N_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO102_N_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO102_NW_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO102_NW_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO102_NW_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO102_NE_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO102_NE_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO102_NE_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO102_EW_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO102_EW_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO102_EW_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO102_NEW_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO102_NEW_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO102_NEW_W(IRightOfWayAnalyzer* analyzer);
  
  /* test_NESW_StVO206 */
  
  int test_NESW_StVO206__N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO206__E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO206__W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO206_E_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO206_E_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO206_E_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO206_W_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO206_W_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO206_W_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO206_N_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO206_N_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO206_N_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO206_NW_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO206_NW_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO206_NW_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO206_NE_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO206_NE_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO206_NE_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO206_EW_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO206_EW_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO206_EW_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO206_NEW_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO206_NEW_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO206_NEW_W(IRightOfWayAnalyzer* analyzer);
  
  /* test_NESW_StVO205 */
  
  int test_NESW_StVO205__N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO205__E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO205__W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO205_E_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO205_E_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO205_E_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO205_W_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO205_W_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO205_W_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO205_N_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO205_N_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO205_N_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO205_NW_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO205_NW_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO205_NW_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO205_NE_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO205_NE_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO205_NE_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO205_EW_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO205_EW_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO205_EW_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO205_NEW_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO205_NEW_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO205_NEW_W(IRightOfWayAnalyzer* analyzer);
  
  /* test_NESW_StVO301 */
  
  int test_NESW_StVO301__N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO301__E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO301__W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO301_E_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO301_E_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO301_E_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO301_W_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO301_W_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO301_W_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO301_N_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO301_N_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO301_N_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO301_NW_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO301_NW_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO301_NW_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO301_NE_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO301_NE_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO301_NE_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO301_EW_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO301_EW_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO301_EW_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO301_NEW_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO301_NEW_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO301_NEW_W(IRightOfWayAnalyzer* analyzer);
  
  /* test_NESW_StVO20930 */
  
  int test_NESW_StVO20930__N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO20930__E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO20930__W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO20930_E_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO20930_E_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO20930_E_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO20930_W_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO20930_W_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO20930_W_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO20930_N_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO20930_N_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO20930_N_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO20930_NW_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO20930_NW_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO20930_NW_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO20930_NE_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO20930_NE_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO20930_NE_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO20930_EW_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO20930_EW_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO20930_EW_W(IRightOfWayAnalyzer* analyzer);
  
  int test_NESW_StVO20930_NEW_N(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO20930_NEW_E(IRightOfWayAnalyzer* analyzer);
  int test_NESW_StVO20930_NEW_W(IRightOfWayAnalyzer* analyzer);
    
};

#endif // RIGHTOFWAYANALYZERNESWTESTS_H
