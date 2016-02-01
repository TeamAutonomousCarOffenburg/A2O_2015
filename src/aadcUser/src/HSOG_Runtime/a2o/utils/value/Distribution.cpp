#include "Distribution.h"

#include <cmath>

using namespace A2O;


Distribution::Distribution(const size_t& size)
      : _size(size), _mean(0), _varianz(0)
{
}

Distribution::~Distribution()
{
}

void Distribution::addValue(const double& value)
{
  _values.push_back(value);

  // remove first element if size is too big
  if (_values.size() > _size) {
    _values.erase(_values.begin());
  }

  // Calculate mean
  _mean = 0;
  for (double val : _values) {
    _mean += val;
  }
  _mean /= _values.size();

  // Calculate varianz
  _varianz = 0;
  double dev;
  for (double val : _values) {
    dev = (val - _mean);
    _varianz += dev * dev;
  }
}

const double Distribution::getMean() const
{
  return _mean;
}

const double Distribution::getVarianz() const
{
  return _varianz;
}

const double Distribution::getSTDDEV() const
{
  return std::sqrt(_varianz);
}

const bool Distribution::isValid() const
{
  return _values.size() == _size;
}
