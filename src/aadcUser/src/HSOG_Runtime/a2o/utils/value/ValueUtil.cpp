#include "ValueUtil.h"


using namespace A2O;

template<typename T>
void limitAbs(T& value, const T& limit)
{
  if (value > limit) {
    value = limit;
  } else if (value < -limit) {
    value = -limit;
  }
}

template<typename T>
void ValueUtil::limit(T& value, const T& lowerBound, const T& upperBound)
{
  if (value > upperBound) {
    value = upperBound;
  } else if (value < lowerBound) {
    value = lowerBound;
  }
}

template<typename T>
const T ValueUtil::valueInAbsLimit(const T& value, const T& limit)
{
  T res = value;
  limitAbs(res, limit);
  return res;
}

template<typename T>
const T ValueUtil::valueInLimit(const T& value, const T& lowerBound, const T& upperBound)
{
  T res = value;
  limit(res, lowerBound, upperBound);
  return res;
}
