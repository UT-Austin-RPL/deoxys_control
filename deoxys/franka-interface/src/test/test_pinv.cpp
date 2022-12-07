#include "utils/control_utils.h"
#include <Eigen/Dense>
#include <chrono>
#include <iostream>

int main() {
  Eigen::MatrixXd m = Eigen::MatrixXd::Random(7, 6);
  m.row(m.rows() - 1).setZero();

  Eigen::MatrixXd m_inv;

  std::chrono::high_resolution_clock::time_point t1 =
      std::chrono::high_resolution_clock::now();
  control_utils::PInverse(m, m_inv);
  std::chrono::high_resolution_clock::time_point t2 =
      std::chrono::high_resolution_clock::now();
  auto time = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
  std::cout << time.count() << std::endl;
  // std::cout << m << std::endl;
  // std::cout << " ------------ " << std::endl;

  // std::cout << m_inv << std::endl;
  // std::cout << " ------------ " << std::endl;
  // std::cout << m_inv * m  << std::endl;
}
