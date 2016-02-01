#pragma once

#include <boost/smart_ptr.hpp>


namespace A2O {

/**
 * Basic Interface for all decision makers.
 * 
 * \author Stefan Glaser
 */
class IDecisionMaker {
public:
  typedef boost::shared_ptr<IDecisionMaker> Ptr;
  typedef boost::shared_ptr<const IDecisionMaker> ConstPtr;

  virtual ~IDecisionMaker(){};
  
  /** Called during each action cycle to decide for the next action.
   *
   * \returns success
   */
  virtual const bool decide() = 0;
};

}