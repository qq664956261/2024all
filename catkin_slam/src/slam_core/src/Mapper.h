#ifndef MAPPER_H
#define MAPPER_H
#include <Eigen/Core>


class Mapper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Mapper();

  Mapper(std::string config_file);

  virtual ~Mapper();


protected:

};



#endif
