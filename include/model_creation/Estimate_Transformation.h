#ifndef ESTIMATE_TRANSFORMATION_H
#define ESTIMATE_TRANSFORMATION_H

#include <eigen3/Eigen/src/Core/MatrixBase.h>

class Estimate_Transformation {
public:
     Estimate_Transformation(std::string f_path); //f_path > path to plane
     ~Estimate_Transformation();
     Eigen::Matrix4d get_transformation() ; //return the final transformation to the center of the table

};

#endif // ESTIMATE_TRANSFORMATION_H
