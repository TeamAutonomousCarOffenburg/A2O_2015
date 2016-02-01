#pragma once

#include "Effector.h"
#include "action/IValueEffector.h"


namespace A2O {

/**
 * The ValueEffector represents an effector for simple double values.
 *
 * \author Stefan Glaser
 */
template<typename T>
class ValueEffector : public Effector, public virtual IValueEffector<T> {
public:
  typedef boost::shared_ptr<ValueEffector> Ptr;
  typedef boost::shared_ptr<const ValueEffector> ConstPtr;

  ValueEffector(const std::string& name) : Effector(name){};
  virtual ~ValueEffector(){};
  
  virtual void setValue(const T& newValue, const long& time)
  {
    _value = newValue;
    _time = time;
  };
  
  virtual const T& getValue() const
  {
    return _value;
  };
  
protected:
  /** The effector value. */
  T _value;
};

typedef ValueEffector<bool> BoolValueEffector;
typedef ValueEffector<int> IntValueEffector;
typedef ValueEffector<double> DoubleValueEffector;

}
