#ifndef _QUADPROGPP
#define _QUADPROGPP


/* WARNING!!!!!
 * This is a modified version of Quadprog++ by Joseph Salini (salini@isir.upmc.fr)
 * it deletes the matrix/vector classes defined in the orignal
 * version, and replaces it by Eigen classes
 */
#include <Eigen/Core>

namespace QuadProgPP{
    
    /**
     *   The quadprog_solve() function implements the algorithm of Goldfarb and Idnani
         for the solution of a (convex) Quadratic Programming problem
         by means of an active-set dual method.
         
         The quadratic problem with \f$n\f$ variables and \f$m\f$ constraints is:
     
         \f{align*}{
             \argmin{\x} &: \;  \frac{1}{2} \x\tp G \x + \vec{g}_0\tp \x \\
             & CE\tp \x + \vec{ce}_0 =    \vec{0} \\
             & CI\tp \x + \vec{ci}_0 \geq \vec{0}
         \f}
              
         The matrix and vectors dimensions are as follows:
     
         \f$G\f$: \f$n \times n\f$
     
         \f$\vec{g}_0\f$: \f$n\f$
         
         \f$CE\f$: \f$n \times p\f$
     
         \f$\vec{ce}_0\f$: \f$p\f$
         
         \f$CI\f$: \f$n \times m\f$
     
         \f$\vec{ci}_0\f$: \f$m\f$
         
         \f$x\f$: \f$n\f$
         
         The function will return the cost of the solution written in the \f$x\f$ vector or
         std::numeric_limits::infinity() if the problem is infeasible. In the latter case
         the value of the \f$x\f$ vector is not correct.
     *
     *  @param _G  n x n Symmetric positive definite matrix.
     *  @param g0  Vector of size n, where n is the number of variables.
     *  @param _CE Matrix of size n x p, where p is the number of equalities.
     *  @param ce0 Vector of size p.
     *  @param _CI Matrix of size n x m, where m is the number of inequalities.
     *  @param ci0 Vector of size m.
     *  @param x   Problem variables of size n.
     *  @note  Pay attention in setting up the vectors \f$\vec{ce}_0\f$ and \f$\vec{ci}_0\f$.
               If the constraints of your problem are specified in the form
               \f$A^T x = b\f$ and \f$C^T x \geq d\f$, then you should set \f$\vec{ce}_0 = -b\f$ and \f$\vec{ci}_0 = -d\f$. 
               Also the matrix \f$G\f$ is modified within the function since it is used to compute
               the \f$G = L^T L\f$ cholesky factorization for further computations inside the function.
               If you need the original matrix \f$G\f$ you should make a copy of it and pass the copy
               to the function.
     *
     *  @return cost of the solution written in the x vector or
     std::numeric_limits::infinity() if the problem is infeasible. In the latter case
     the value of the \f$ x \f$ vector is not correct.
     *
     *  @author Luca Di Gaspero, DIEGM - University of Udine, Italy.
     *  @copyright GNU Lesser General Public License.
     *  @copyright 2007-2009 Luca Di Gaspero.
     *  @copyright 2007-2009 Luca Di Gaspero.
     */
    
    double solve_quadprog(const Eigen::MatrixXd& _G,  const Eigen::VectorXd& g0,
                        const Eigen::MatrixXd& _CE, const Eigen::VectorXd& ce0,
                        const Eigen::MatrixXd& _CI, const Eigen::VectorXd& ci0,
                              Eigen::VectorXd& x);
}

#endif // #define _QUADPROGPP
