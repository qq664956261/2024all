#ifndef LOCALIZER_H
#define LOCALIZER_H
#include <Eigen/Core>


class Localizer
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Localizer();

  Localizer(std::string config_file);

  virtual ~Localizer();


protected:

};



#endif
