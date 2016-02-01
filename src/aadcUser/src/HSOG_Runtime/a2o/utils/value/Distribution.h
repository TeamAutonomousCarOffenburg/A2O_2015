#pragma once

#include <vector>
#include <boost/concept_check.hpp>


namespace A2O {

class Distribution {
public:
  Distribution(const size_t& size = 10);
  ~Distribution();

  void addValue(const double& value);

  const double getMean() const;
  const double getVarianz() const;
  const double getSTDDEV() const;

  const bool isValid() const;

private:
  const size_t _size;
  double _mean;
  double _varianz;
  
  std::vector<double> _values;
};

}