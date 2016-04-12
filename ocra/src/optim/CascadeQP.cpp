#include "ocra/optim/CascadeQP.h"

namespace ocra
{
  using namespace std;
  // Consutructeur :
  CascadeQP::CascadeQP()
  {
    initializeHierarchyLevel_barre();                      // Initialize the system of equalities and inequalities
  }



  // Destructeur :
  CascadeQP::~CascadeQP()
  {
    ;
  }




  //Add Hierarchy Level 1 :
  size_t CascadeQP::addHierarchyLevel(HierarchyLevel::Ptr h)
  {

    allHierarchyLevel.push_back(std::move(h));

    if (allHierarchyLevel.size() > allSolution.size())  // NOTE: we assume the difference can be only 1
    {
      allSolution.push_back( Solution::Ptr());
      allHierarchyLevel_barre.push_back( HierarchyLevel_barre::Ptr());
      allMatrixPQ.push_back( MatrixPQ::Ptr());
      allEqualitiesConstraints.push_back( EqualitiesConstraints::Ptr());
      allInequalitiesConstraints.push_back( InequalitiesConstraints::Ptr());
    }

    return (allHierarchyLevel.size()-1);
  }


//   Add Hierarchy Level 2 :
  size_t CascadeQP::addHierarchyLevel(const std::vector<HierarchyLevel::Ptr>& v)
  {
    for(size_t j=0 ; j < v.size() ; j++)
    {
      addHierarchyLevel(v[j]);
    }
    return (allHierarchyLevel.size()-1);
  }

  // Compute Matrix PQ

  ////////////////////////////////////////////
  //       At*A  0                At*b      //
  // P = [         ]      q = - [      ]    //
  //        0   I                  0        //
  ////////////////////////////////////////////
  void CascadeQP::computeMatrixPQ(int i)
  {
//     //MatrixPQ m;
//     //MatrixPQ* m=new MatrixPQ;
//     HierarchyLevel& h = *allHierarchyLevel[i];
//     MatrixPQ* m = allMatrixPQ[i];
//
//     //cout << "size = " << endl << max(h.A.cols()(),h.C.cols()()) << endl;
//     //cout << "size1 = " << endl << max(allHierarchyLevel[0]->A.cols()(),allHierarchyLevel[0]->C.cols()()) << endl;
//
//     int size = max(h.A.cols()(),h.C.cols()());
//
//     /////////  compute P ///////////
//     m->P.resize(size + h.C.rows()(), size + h.C.rows()());
//     m->P.setZero();
//     //cout << "P = " << endl << m->P<< endl;
//     // P_sub1 = At*A
//     SubMatrix P_sub1(m->P);
//     P_sub1.rescope(size, size, 0, 0);
//     if (h.A.rows()() == 0)
//     {
//       P_sub1.setZero();
//     }
//     else
//     {
//       CML_gemm<'t','n'>(1, h.A, h.A, 0, P_sub1);                // 'n': normale  , 't':transposee
//     }
//     //cout << "P_sub1 = " << endl << P_sub1 << endl;
//     //cout << "P = " << endl <<  m->P << endl;
//     // P_sub2 = I
//     cmlDSubDenseMatrix P_sub2(m->P);
//     P_sub2.rescope(h.C.rows()(), h.C.rows()(), size, size);
//     P_sub2.setToIdentity();
//     //cout << "P_sub2 = " << endl << P_sub2 << endl;
//     //cout << "P = " << endl <<  m->P << endl;
//
//     /////////  compute Q ///////////
//     m->q.resize(size + h.C.rows()());
//     m->q.setZero();
//
//     // q_sub1 = At*b
//     cmlDSubDenseVector q_sub1(m->q);
//     q_sub1.rescope(size,0);
//     if (h.A.rows()() == 0)
//     {
//       q_sub1.setZero();
//     }
//     else
//     {
//       CML_gemv<'t'>(1, h.A, h.b, 0, q_sub1);
//     }
//
//     // q_sub2 = 0
//     cmlDSubDenseVector q_sub2(m->q);
//     q_sub2.rescope(h.C.rows()(),size);
//     q_sub2.setZero();
//     CML_scal(-1,m->q);
//     //cout << "q_sub2 = " << endl << q_sub2 << endl;
//
//     //cout << "P = " << endl <<  m->P << endl;
//     //cout << "q = " << endl <<  m->q << endl;
//
//     //add m in Matrix QP vector
//     //allMatrixPQ.push_back(m);
//
//     //return (allMatrixPQ.size()-1);
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////
  //                              Initialize Hierarchy Level barre                               //
  /////////////////////////////////////////////////////////////////////////////////////////////////

  size_t CascadeQP::initializeHierarchyLevel_barre()
  {
     allHierarchyLevel_barre.push_back(HierarchyLevel_barre::Ptr());
     return (allHierarchyLevel_barre.size()-1);
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////
  //                                 Compute Equalities Constraints                                //
  /////////////////////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////
  //                                        //
  // M = [ A_barre ]      n = [ b_barre ]   //
  //                                        //
  ////////////////////////////////////////////
  void CascadeQP::computeEqualitiesConstraints(int i)
  {
    auto e = allEqualitiesConstraints[i];

    e->M.resize(allHierarchyLevel_barre[i]->A_barre.rows(),
                allHierarchyLevel_barre[i]->A_barre.cols());
    e->M = allHierarchyLevel_barre[i]->A_barre;

    e->n.resize(allHierarchyLevel_barre[i]->A_barre.rows());
    e->n = allHierarchyLevel_barre[i]->b_barre;

  }


  /////////////////////////////////////////////////////////////////////////////////////////////////
  //                                Compute Inequalities Constraints                               //
  /////////////////////////////////////////////////////////////////////////////////////////////////

  /////////////////////////////////////////////
  //       C_barre              d_barre      //
  // R = [         ]      s = [         ]    //
  //       C    -I                 d         //
  /////////////////////////////////////////////
  void CascadeQP::computeInequalitiesConstraints(int i)
  {
    InequalitiesConstraints::Ptr ie = allInequalitiesConstraints[i];


    ///// R /////
    ie->R.setZero(allHierarchyLevel_barre[i]->C_barre.rows() + allHierarchyLevel[i]->C.rows(),
                  std::max(allHierarchyLevel_barre[i]->C_barre.cols(),
                           allHierarchyLevel[i]->C.cols() + allHierarchyLevel[i]->C.rows()));


    ie->R.block(0,0,allHierarchyLevel_barre[i]->C_barre.rows(),
                allHierarchyLevel_barre[i]->C_barre.cols()) = allHierarchyLevel_barre[i]->C_barre;

    ie->R.block(allHierarchyLevel_barre[i]->C_barre.rows(),0,
                       allHierarchyLevel[i]->C.rows(),allHierarchyLevel[i]->C.cols()) = allHierarchyLevel[i]->C;

    ie->R.block(allHierarchyLevel_barre[i]->C_barre.rows(),allHierarchyLevel[i]->C.cols(),
                allHierarchyLevel[i]->C.rows(),allHierarchyLevel[i]->C.rows()).setIdentity() *=-1.0;

    /////// s /////
    ie->s.setZero(allHierarchyLevel_barre[i]->C_barre.rows() + allHierarchyLevel[i]->C.rows());

    ie->s.head(allHierarchyLevel_barre[i]->C_barre.rows()) = allHierarchyLevel_barre[i]->d_barre;

    ie->s.tail(allHierarchyLevel_barre[i]->C_barre.rows()) = allHierarchyLevel[i]->d;

  }


//   /////////////////////////////////////////////////////////////////////////////////////////////////
//   //                                         Solve QLD                                           //
//   /////////////////////////////////////////////////////////////////////////////////////////////////
//
//   int CascadeQP::solveQLD(int i)
//   {
// //     // TAILLE DU PROBLEME
// //     int n = allMatrixPQ[i]->P.cols()() ;                         // NUMBER OF VARIABLES
// //     int n1= max(allEqualitiesConstraints[i]->M.cols()() , allInequalitiesConstraints[i]->R.cols()());
// //     int nmax = n;                                                   // ROW DIMENSION OF P
// //     int me = allEqualitiesConstraints[i]->M.rows()() ;           // NUMBER OF Equalities CONSTRAINTS
// //     int mi = allInequalitiesConstraints[i]->R.rows()() ;         // NUMBER OF Inequalities CONSTRAINTS
// //     int m = me + mi;                                                // TOTAL NUMBER OF CONSTRAINTS
// //     int mmax = m;                                                   // ROW DIMENSION OF A
// //     int mnn = m+n+n;                                                // MUST BE EQUAL TO M + N + N
// //
// //     // SOLUTION
// //     //Solution* s = new Solution;
// //     Solution* s = allSolution[i];
// //     s->y.resize(n);
// //     s->y.setZero();
// //
// //     // BORNES
// //     cmlDenseVector<double> Yl(n);
// //     Yl.fill(-1.e10) ;
// //     cmlDenseVector<double> Yu(n);
// //     Yu.fill(1.e10) ;
// //
// //
// //
// //     // MATRICES DE CONTRAINTES :
// //
// //     ///////////////////
// //     //        M      //
// //     // MR = [   ]    //
// //     //        R      //
// //     ///////////////////
// //
// //     cmlDenseMatrix<double> MR(m,n);
// //     //cmlDenseMatrix<double> MR;
// //     //MR.resize( allEqualitiesConstraints[i]->M.rows()() +  allInequalitiesConstraints[i]->R.rows()(), max(allEqualitiesConstraints[i]->M.cols()() ,  allHierarchyLevel[i]->C.rows()())) ;
// //     //MR.setZero();
// //     int size = max(allHierarchyLevel[i]->A.cols()(),allHierarchyLevel[i]->C.cols()());
// //     //cout << "Matrix P= " << endl << n << endl;
// //     //cout << " Constraints = " << endl << n1 << endl;
// //     //cout << " size = " << endl << size << endl;
// //
// //
// //
// //     // MR_sub1 = M
// //     cmlDSubDenseMatrix MR_sub1(MR);
// //     MR_sub1.rescope( me , allEqualitiesConstraints[i]->M.cols()() , 0 , 0 );
// //     //MR_sub1.rescope( me , n , 0 , 0 );
// //     MR_sub1.copyValuesFrom(allEqualitiesConstraints[i]->M);
// //
// //
// //     // MR_sub2 = R
// //     cmlDSubDenseMatrix MR_sub2(MR);
// //     MR_sub2.rescope( mi , n , me, 0 );
// //     MR_sub2.copyValuesFrom(allInequalitiesConstraints[i]->R);
// //     MR_sub2.scalarMultInPlace(-1);
// //
// //
// //     ///////////////////
// //     //        N      //
// //     // NS = [   ]    //
// //     //        S      //
// //     ///////////////////
// //
// //     cmlDenseVector<double> NS(m);
// //     //cmlDenseVector<double> NS;
// //     //NS.resize( allEqualitiesConstraints[i]->M.rows()() +  allInequalitiesConstraints[i]->R.rows()()) ;
// //     //NS.setZero();
// //
// //     // NS_sub1 = N
// //     cmlDSubDenseVector NS_sub1(NS);
// //     NS_sub1.rescope( me , 0 );
// //     CML_axpy(-1, allEqualitiesConstraints[i]->n, NS_sub1);
// //
// //     // NS_sub2 = S
// //     cmlDSubDenseVector NS_sub2(NS);
// //     NS_sub2.rescope( mi ,me);
// //     NS_sub2.copyValuesFrom(allInequalitiesConstraints[i]->s);
// //
// //
// //
// //     // SOLVE :
// //     /*cout << "MR_sub1 = " << endl << MR_sub1 << endl;
// //     cout << "MR_sub2 = " << endl << MR_sub2 << endl;
// //     cout << "M= " << endl << allEqualitiesConstraints[i]->M << endl;
// //     cout << "R= " << endl << allInequalitiesConstraints[i]->R << endl;
// //     cout << "P= " << endl << allMatrixPQ[i]->P << endl;
// //     cout << "q= " << endl << allMatrixPQ[i]->q << endl;
// //     cout << "MR = " << endl << MR << endl;
// //     cout << "NS = " << endl << NS << endl;*/
// //
// //     //cout << "MR = " << endl << MR.cols()() << endl;
// //     //cout << "NS = " << endl << NS.cols()() << endl
// //
// //     int r = solver.solve(allMatrixPQ[i]->P, allMatrixPQ[i]->q , MR, NS, me, s->y, Yl, Yu, false);
// //
// //     //for (size_t k=0; k<s->y.getSize(); ++k)
// //     //cout << s->y[k] << endl;
// //
// //     //allSolution.push_back(s);
// //     //cout << " SOLUTION : Y = "  << allSolution[i]->y << endl;
// //
// //
// // /*    cout << "***********LEVEL " << i << "*************" << endl;
// //     cout << "equalities" << endl;
// //     for (int j=0; j<me; ++j)
// //       cout << j << " : " << (solver.getLagrangeMultipliers())[j] << endl;
// //
// //     cout << "inequalities" << endl;
// //     for (int j=0; j<mi; ++j)
// //       cout << j << " : " << (solver.getLagrangeMultipliers())[me+j] << endl;
// //
// //     cout << "lower bound" << endl;
// //     for (int j=0; j<n; ++j)
// //       cout << j << " : " << (solver.getLagrangeMultipliers())[me+mi+j] << endl;
// //
// //     cout << "upper bound" << endl;
// //     for (int j=0; j<n; ++j)
// //       cout << j << " : " << (solver.getLagrangeMultipliers())[me+mi+n+j] << endl;
// // */
// //     return r;
//   }


  /////////////////////////////////////////////////////////////////////////////////////////////////
  //                               Add Hierarchy Level_barre                                     //
  /////////////////////////////////////////////////////////////////////////////////////////////////

  void CascadeQP::addHierarchyLevel_barre(int i)
  {
//     //HierarchyLevel_barre* h_b = new HierarchyLevel_barre;
//     HierarchyLevel_barre* h_b = allHierarchyLevel_barre[i+1];
//
//     ////////////////////// calcul du nombre de contraintes viol�es : compteur ///////////////////
//
//     // X : solution du niveau hierarchique k
//     cmlDSubDenseVector X(allSolution[i]->y);
//     X.rescope(max(allHierarchyLevel[i]->A.cols()(),allHierarchyLevel[i]->C.cols()()),0);
//
//
//     // Cj : ligne de la matrice C du niveau hierarchique k
//     cmlDSubDenseMatrix Cj(allHierarchyLevel[i]->C);
//     cmlDSubDenseVector dj(allHierarchyLevel[i]->d);
//
//     // L : Cj*X
//     cmlDenseVector<double> L;
//     L.resize(allHierarchyLevel[i]->C.rows()());
//
//     // compteur :
//     int compteurConstraintsViolated = 0;
//     int compteurConstraintsNotViolated = 0;
//     indexConstraintsNotViolated.clear();
//     indexConstraintsViolated.clear();
//
//     if (allHierarchyLevel[i]->C.rows()() == 0)
//     {
//       //throw runtime_error("[QLDSolver::addLinearEqualitiesConstraint] added Constraint is not an equalities" );
//     }
//     else
//     {
//       CML_gemv<'n'>(1,allHierarchyLevel[i]->C,X,0,L);
//       for (size_t j=0; j<allHierarchyLevel[i]->C.rows()(); ++j)
//       {
//         //Cj.rescope(1,allHierarchyLevel[i]->C.cols()(),j,0);
//         //cout << "X = " << endl << X << endl;
//         //cout << "Cj = " << endl << Cj << endl;
//         //CML_gemv<'n'>(1,Cj,X,0,L);
//         //cout << " L = " << endl << L << endl;
//
//         if ( L(j) <= allHierarchyLevel[i]->d(j) )
//         {
//           compteurConstraintsNotViolated = compteurConstraintsNotViolated + 1 ;
//           indexConstraintsNotViolated.push_back(j);
//         }
//         else
//         {
//           compteurConstraintsViolated = compteurConstraintsViolated + 1 ;
//           indexConstraintsViolated.push_back(j);
//         }
//       }
//     }
//
//
//
//
//     /////////////////////////////////////
//     //                  A_barre(k)     //
//     // A_barre(k+1) = [   A(k)   ]     //
//     //                     cj          //
//     /////////////////////////////////////
//
//     h_b->A_barre.resize(allHierarchyLevel_barre[i]->A_barre.rows()() + allHierarchyLevel[i]->A.rows()()+ compteurConstraintsViolated, max(allHierarchyLevel_barre[i]->A_barre.cols()(),max(allHierarchyLevel[i]->A.cols()(), allHierarchyLevel[i]->C.cols()())));
//     h_b->A_barre.setZero();
//     //cout << "A_barre = " << endl << h_b->A_barre << endl;
//     // A_sub1 = A_barre(k)
//     cmlDSubDenseMatrix A_sub1(h_b->A_barre);
//     A_sub1.rescope(allHierarchyLevel_barre[i]->A_barre.rows()(), allHierarchyLevel_barre[i]->A_barre.cols()(), 0, 0);
//     A_sub1.copyValuesFrom(allHierarchyLevel_barre[i]->A_barre);
//     //cout << "A_sub1 = " << endl << A_sub1 << endl;
//
//     // A_sub2 = A(k)
//     cmlDSubDenseMatrix A_sub2(h_b->A_barre);
//     A_sub2.rescope(allHierarchyLevel[i]->A.rows()(), allHierarchyLevel[i]->A.cols()(), allHierarchyLevel_barre[i]->A_barre.rows()(),0);
//     A_sub2.copyValuesFrom(allHierarchyLevel[i]->A);
//     //cout << "A_sub2 = " << endl << A_sub2 << endl;
//
//     // A_sub3 = cj
//     cmlDSubDenseMatrix A_sub3(h_b->A_barre);
//     for (int j=0; j<compteurConstraintsViolated; ++j)
//     {
//       Cj.rescope(1,allHierarchyLevel[i]->C.cols()(),indexConstraintsViolated[j],0);
//       A_sub3.rescope(1, max(allHierarchyLevel[i]->A.cols()(), allHierarchyLevel[i]->C.cols()()), allHierarchyLevel_barre[i]->A_barre.rows()() + allHierarchyLevel[i]->A.rows()() + j, 0);
//       A_sub3.copyValuesFrom(Cj);
//     }
//     //cout << "A_sub3 = " << endl << A_sub3 << endl;
//
//     //cout << "A_barre = " << endl << h_b->A_barre << endl;
//
//
//     /////////////////////////////////////
//     //                  b_barre(k)     //
//     // b_barre(k+1) = [  A(k)x(k)  ]   //
//     //                    cj x(k)      //
//     /////////////////////////////////////
//
//     h_b->b_barre.resize(allHierarchyLevel_barre[i]->A_barre.rows()() + allHierarchyLevel[i]->A.rows()() + compteurConstraintsViolated );
//     h_b->b_barre.setZero();
//
//     // b_sub1 = b_barre(k)
//     cmlDSubDenseVector b_sub1(h_b->b_barre);
//     b_sub1.rescope(allHierarchyLevel_barre[i]->A_barre.rows()(),0);
//     b_sub1.copyValuesFrom(allHierarchyLevel_barre[i]->b_barre);
//     //cout << "b_sub1 = " << endl << b_sub1 << endl;
//
//
//     // b_sub2 = A(k)x(k)
//     cmlDSubDenseVector b_sub2(h_b->b_barre);
//     b_sub2.rescope(allHierarchyLevel[i]->A.rows()(), allHierarchyLevel_barre[i]->A_barre.rows()());
//     if (allHierarchyLevel[i]->A.rows()() == 0)
//     {
//       ;
//     }
//     else
//     {
//       CML_gemv<'n'>(1, allHierarchyLevel[i]->A, X, 0, b_sub2);
//     }
//     //cout << "b_sub2 = " << endl << b_sub2 << endl;
//
//
//     // b_sub3 = cj x(k)
//     cmlDSubDenseVector b_sub3(h_b->b_barre);
//     b_sub3.rescope(compteurConstraintsViolated, allHierarchyLevel_barre[i]->A_barre.rows()() + allHierarchyLevel[i]->A.rows()());
//     cmlDenseVector<double> b (compteurConstraintsViolated);
//     b.setZero();
//     cmlDSubDenseVector bj(b);
//
//
//
//     for (int j=0; j<compteurConstraintsViolated; ++j)
//     {
//       b_sub3.rescope(1,  allHierarchyLevel_barre[i]->A_barre.rows()() + allHierarchyLevel[i]->A.rows()() + j );
//       Cj.rescope(1,allHierarchyLevel[i]->C.cols()(),indexConstraintsViolated[j],0);
//       CML_gemv<'n'>(1,Cj, X, 0, b_sub3);
//     }
//
//     //cout << "b_sub3 = " << endl << b_sub3 << endl;
//
//     //cout << "b_barre = " << endl << h_b->b_barre << endl;
//
//
//     /////////////////////////////////////
//     //                  C_barre(k)     //
//     // C_barre(k+1) = [            ]   //
//     //                     cj          //
//     /////////////////////////////////////
//
//     h_b->C_barre.resize(allHierarchyLevel_barre[i]->C_barre.rows()() + compteurConstraintsNotViolated , max(allHierarchyLevel_barre[i]->C_barre.cols()(),max(allHierarchyLevel[i]->A.cols()(), allHierarchyLevel[i]->C.cols()())) );
//     h_b->C_barre.setZero();
//
//     // C_sub1 = C_barre(k)
//     cmlDSubDenseMatrix C_sub1(h_b->C_barre);
//     C_sub1.rescope(allHierarchyLevel_barre[i]->C_barre.rows()(), allHierarchyLevel_barre[i]->C_barre.cols()(), 0, 0);
//     C_sub1.copyValuesFrom(allHierarchyLevel_barre[i]->C_barre);
//     //cout << "C_sub1 = " << endl << C_sub1 << endl;
//
//
//     // C_sub2 = cj
//     cmlDSubDenseMatrix C_sub2(h_b->C_barre);
//     for (int j=0; j<compteurConstraintsNotViolated; ++j)
//     {
//       Cj.rescope(1,allHierarchyLevel[i]->C.cols()(),indexConstraintsNotViolated[j],0);
//       C_sub2.rescope(1, max(allHierarchyLevel[i]->A.cols()(), allHierarchyLevel[i]->C.cols()()), allHierarchyLevel_barre[i]->C_barre.rows()() + j, 0);
//       C_sub2.copyValuesFrom(Cj);
//     }
//     //cout << "C_sub2 = " << endl << C_sub2 << endl;
//
//     //cout << "C_barre = " << endl << h_b->C_barre << endl;
//
//     /////////////////////////////////////
//     //                  d_barre(k)     //
//     // d_barre(k+1) = [            ]   //
//     //                    dj           //
//     /////////////////////////////////////
//
//     h_b->d_barre.resize(allHierarchyLevel_barre[i]->C_barre.rows()() + compteurConstraintsNotViolated );
//     h_b->d_barre.setZero();
//     //cout << "d_barre = " << endl << h_b->d_barre << endl;
//
//     // d_sub1 = d_barre(k)
//     cmlDSubDenseVector d_sub1(h_b->d_barre);
//     d_sub1.rescope(allHierarchyLevel_barre[i]->C_barre.rows()(),0);
//     d_sub1.copyValuesFrom(allHierarchyLevel_barre[i]->d_barre);
//     //cout << "d_sub1= " << endl << d_sub1 << endl;
//
//     // d_sub2 = dj
//     cmlDSubDenseVector d_sub2(h_b->d_barre);
//     d_sub2.rescope(compteurConstraintsNotViolated, allHierarchyLevel_barre[i]->C_barre.rows()() );
//     for (int j=0; j<compteurConstraintsNotViolated; ++j)
//     {
//       dj.rescope(1,indexConstraintsNotViolated[j]);
//       d_sub2.rescope(1, allHierarchyLevel_barre[i]->C_barre.rows() + j );
//       d_sub2.copyValuesFrom(dj);
//     }
//     //cout << "d_sub2= " << endl << d_sub2 << endl;
//
//     //cout << "d_barre = " << endl << h_b->d_barre << endl;
//
//     //allHierarchyLevel_barre.push_back(h_b);
//     //return (allHierarchyLevel_barre.size()-1);
  }



  /////////////////////////////////////////////////////////////////////////////////////////////////
  //                                           Clear                                             //
  /////////////////////////////////////////////////////////////////////////////////////////////////
  void CascadeQP::clear(void)
  {
    allHierarchyLevel.clear();
  }




  /////////////////////////////////////////////////////////////////////////////////////////////////
  //                                           SOLVE                                             //
  /////////////////////////////////////////////////////////////////////////////////////////////////

  const FinalSolution& CascadeQP::solveCascadeQP()
  {
    size_t nbLevel = (size_t)allHierarchyLevel.size();

    for(size_t i=0; i<nbLevel; i++)
    {
      computeMatrixPQ(i);                                    // Compute QP
      //cout << "P = " << endl << allMatrixPQ[i]->P << endl;
      //cout << "q = " << endl << allMatrixPQ[i]->q << endl;
      //writeInFile(allMatrixPQ[i]->P, "P_CASCADE_QP.txt", true);
      //writeInFile(allMatrixPQ[i]->q, "q_CASCADE_QP.txt", true);

      computeEqualitiesConstraints(i);                       // Compute Equalities Constraints
      //cout << "M = " << endl << allEqualitiesConstraints[i]->M << endl;
      //cout << "n = " << endl << allEqualitiesConstraints[i]->n << endl;
      //writeInFile(allEqualitiesConstraints[i]->M , "M_CASCADE_QP.txt", true);
      //writeInFile(allEqualitiesConstraints[i]->n , "n_CASCADE_QP.txt", true);

      computeInequalitiesConstraints(i);                     // Compute Inequalities Constraints
      //cout << "R = " << endl << allInequalitiesConstraints[i]->R << endl;
      //cout << "s = " << endl << allInequalitiesConstraints[i]->s << endl;
      //writeInFile(allInequalitiesConstraints[i]->R, "R_CASCADE_QP.txt", true);
      //writeInFile(allInequalitiesConstraints[i]->s, "s_CASCADE_QP.txt", true);

      ////f.r = solveQLD(i);                                     // Solve QLD
      //cout << "r = " << endl << f.r << endl;
      //cout << "solution = " << endl << f.yf << endl;
      //writeInFile(f.yf, "yf_CASCADE_QP.txt", true);

      addHierarchyLevel_barre(i);                            // Add Hierarchy Level_barre

      //printf("[return]"); getchar();
    }


    //   f.r = solveQLD(allHierarchyLevel.size()-1);
    //   f.yf.resize(max(allHierarchyLevel[nbLevel-1]->A.cols()(), allHierarchyLevel[nbLevel-1]->C.cols()()));
    //f.yf.resize(allSolution[nbLevel-1]->y.getSize());
    f.yf = allSolution[nbLevel-1]->y;


    return f;
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////
  //                                 get the final solution                                      //
  /////////////////////////////////////////////////////////////////////////////////////////////////
  const FinalSolution& CascadeQP::getSolution() const
  {
    return f;
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////
  //                                 get Solution Of Level                                       //
  /////////////////////////////////////////////////////////////////////////////////////////////////
  const Eigen::VectorXd& CascadeQP::getSolutionOfLevel(int i) const
  {
    return allSolution[i]->y;
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////
  //                                   get Hierarchy Level                                       //
  /////////////////////////////////////////////////////////////////////////////////////////////////
  HierarchyLevel::Ptr CascadeQP::getHierarchyLevel(int i) const
  {
    return allHierarchyLevel[i];
  }


  /////////////////////////////////////////////////////////////////////////////////////////////////
  //                               get Number Of Hierarchy Level                                 //
  /////////////////////////////////////////////////////////////////////////////////////////////////
  size_t CascadeQP::getNumberOfHierarchyLevel() const
  {
    return allHierarchyLevel.size();
  }


  /////////////////////////////////////////////////////////////////////////////////////////////////
  //                                   get All Hierarchy Level                                   //
  /////////////////////////////////////////////////////////////////////////////////////////////////
  //allHierarchyLevel CascadeQP::getAllHierarchyLevel() const
  //{
  //  return allHierarchyLevel;
  //}


  /////////////////////////////////////////////////////////////////////////////////////////////////
  //                                Affichage des structures                                     //
  /////////////////////////////////////////////////////////////////////////////////////////////////


  ostream & operator<<(ostream &out, const HierarchyLevel& h )
  {
    out << "A = " << endl << h.A  << endl << "b = " << endl << h.b  << endl << "C = " << endl << h.C  << endl << "d = " << endl << h.d  << endl << endl;
    return out;
  }

  ostream & operator<<(ostream &out, const HierarchyLevel_barre& h )
  {
    out << "A_barre = " << endl << h.A_barre  << endl << "b_barre = " << endl << h.b_barre  << endl << "C_barre = " << endl << h.C_barre  << endl << "d_barre = " << endl << h.d_barre  << endl << endl;
    return out;
  }


  ostream & operator<<(ostream &out, const MatrixPQ& m )
  {
    out << "P = " << endl << m.P  << endl << "q =" << endl << m.q  << endl << endl;
    return out;
  }


  ostream & operator<<(ostream &out, const EqualitiesConstraints& e )
  {
    out << "M = " << endl << e.M  << endl << "N =" << endl << e.n  << endl << endl;
    return out;
  }


  ostream & operator<<(ostream &out, const InequalitiesConstraints& ie )
  {
    out << "R = " << endl << ie.R  << endl << "S =" << endl << ie.s  << endl << endl;
    return out;
  }


  ostream & operator<<(ostream &out, const FinalSolution& sf )
  {
    out << "yf = " << endl << sf.yf  << endl << "r =" << endl << sf.r  << endl << endl;
    return out;
  }

}


// cmake:sourcegroup=toBeUpdated
