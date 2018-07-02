#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};


void SetParams(size_t step, double interval, double v, double cte_f, double epsi_f,
               double steer_f, double a_f, double steer_diff_f, double a_diff_f);

#endif /* MPC_H */
