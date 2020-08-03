// Adapted with permission from code by Evan Drumwright
// (https://github.com/edrumwri).
// Taken from The Drake Library http://drake.mit.edu/
// see: https://github.com/ggould-tri/drake/blob/master/drake/solvers/moby_lcp_solver.h
//      https://github.com/RobotLocomotion/drake/blob/master/drake/solvers/moby_lcp_solver.h

#ifndef SOLVERS_MOBYLCP_H_
#define SOLVERS_MOBYLCP_H_

#include <fstream>
#include <Eigen/SparseCore>


class MobyLCPSolver {
 public:
  MobyLCPSolver();
  virtual ~MobyLCPSolver() {};
  void setLoggingEnabled(bool enabled);

  bool lcp_fast(
      const Eigen::MatrixXd& M, const Eigen::VectorXd& q, Eigen::VectorXd* z,
      double zero_tol = -1.0) const;
  bool lcp_fast_regularized(
      const Eigen::MatrixXd& M, const Eigen::VectorXd& q, Eigen::VectorXd* z,
      int min_exp = -20, unsigned step_exp = 4, int max_exp = 20,
      double zero_tol = -1.0) const;
  bool lcp_lemke(
      const Eigen::MatrixXd& M, const Eigen::VectorXd& q, Eigen::VectorXd* z,
      double piv_tol = -1.0, double zero_tol = -1.0) const;
  bool lcp_lemke_regularized(
      const Eigen::MatrixXd& M, const Eigen::VectorXd& q, Eigen::VectorXd* z,
      int min_exp = -20, unsigned step_exp = 1, int max_exp = 1,
      double piv_tol = -1.0, double zero_tol = -1.0) const;
  bool lcp_lemke(
      const Eigen::SparseMatrix<double>& M, const Eigen::VectorXd& q,
      Eigen::VectorXd* z,
      double piv_tol = -1.0, double zero_tol = -1.0) const;
  bool lcp_lemke_regularized(
      const Eigen::SparseMatrix<double>& M, const Eigen::VectorXd& q,
      Eigen::VectorXd* z,
      int min_exp = -20, unsigned step_exp = 4, int max_exp = 20,
      double piv_tol = -1.0, double zero_tol = -1.0) const;

//  virtual bool available() const override { return true; }
  virtual bool available() const { return true; }

 private:
  void clearIndexVectors() const;
  bool checkLemkeTrivial(int n, double zero_tol,
                         const Eigen::VectorXd& q, Eigen::VectorXd* z) const;
  template <typename MatrixType>
  void lemkeFoundSolution(const MatrixType& M, const Eigen::VectorXd& q,
                          Eigen::VectorXd* z) const;

  // TODO(sammy-tri) replace this with a proper logging hookup
  std::ostream& LOG_LCP() const;
  bool log_enabled_;
  mutable std::ofstream null_stream_;

  // TODO(sammy-tri) why is this a member variable?
  mutable unsigned pivots;

  // NOTE:  The temporaries below are stored in the class to minimize
  // allocations; all are marked 'mutable' as they do not affect the
  // semantic const'ness of the class under its methods.

  // temporaries for regularized solver
  mutable Eigen::MatrixXd _MM;
  mutable Eigen::VectorXd _wx;

  // temporaries for fast pivoting solver
  mutable Eigen::VectorXd _z, _w, _qbas, _qprime;
  mutable Eigen::MatrixXd _Msub, _Mmix, _M;

  // temporaries for Lemke solver
  mutable Eigen::VectorXd _d, _Be, _u, _z0, _x, _dl, _xj, _dj, _wl, _result;
  mutable Eigen::MatrixXd _Bl, _Al, _t1, _t2;

  // Vectors which correspond to indices into other data.
  mutable std::vector<unsigned> _all, _tlist, _bas, _nonbas, _j;

  // temporary for sparse Lemke solver
  mutable Eigen::SparseMatrix<double> _sBl;
  mutable Eigen::SparseMatrix<double> _MMs, _MMx, _eye, _zero, _diag_lambda;
};


#endif // SOLVERS_MOBYLCP_H_