#include <Eigen/Geometry>
#include <iostream>

int main(int argc, char** argv){
    const Eigen::AngleAxisf rotation(90.0 * 3.1415 / 180.f, Eigen::Vector3f::UnitZ());
    Eigen::Vector3f point = rotation * (1.f * Eigen::Vector3f::UnitX());
    const Eigen::Matrix3f matrix = rotation.toRotationMatrix();
    Eigen::Vector3f angle = matrix.eulerAngles(0, 1, 2);
    std::cout << "matrix: " << matrix << "\n angle: " << angle  << "\n point: " << point << " befor: " << Eigen::Vector3f::UnitX() << std::endl;
    return 0;
}