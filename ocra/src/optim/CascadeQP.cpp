#if 0

#include "ocra/optim/CascadeQP.h"
#include <stdio.h>
#include "ocra/optim/utilities.h"

using namespace xde;

namespace ocra
{

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
  size_t CascadeQP::addHierarchyLevel(HierarchyLevel *h)
  { 
    //taille = std::max(h->A.get_ncols(), h->C.get_ncols());
    //ASSERT

    allHierarchyLevel.push_back(h);

    //ensure other data structures have a sufficient size
    if (allHierarchyLevel.size() > allSolution.size())  //we assume the difference can be only 1
    {
      allSolution.push_back(new Solution());
      allHierarchyLevel_barre.push_back(new HierarchyLevel_barre());
      allMatrixPQ.push_back(new MatrixPQ());
      allEqualitiesConstraints.push_back(new EqualitiesConstraints());
      allInequalitiesConstraints.push_back(new InequalitiesConstraints());
    }

    return (allHierarchyLevel.size()-1);
  }


  //Add Hierarchy Level 2 :
  size_t CascadeQP::addHierarchyLevel(const std::vector<HierarchyLevel *>& v)
  {
    for(cfl_size_t j=0 ; j < v.size() ; j++)
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
    //MatrixPQ m;
    //MatrixPQ* m=new MatrixPQ; 
    HierarchyLevel& h = *allHierarchyLevel[i];
    MatrixPQ* m = allMatrixPQ[i];

    //std::cout << "size = " << std::endl << std::max(h.A.get_ncols(),h.C.get_ncols()) << std::endl;
    //std::cout << "size1 = " << std::endl << std::max(allHierarchyLevel[0]->A.get_ncols(),allHierarchyLevel[0]->C.get_ncols()) << std::endl;
   
    int size = std::max(h.A.get_ncols(),h.C.get_ncols());

    /////////  compute P ///////////
    m->P.resize(size + h.C.get_nrows(), size + h.C.get_nrows());
    m->P.setToZero();
    //std::cout << "P = " << std::endl << m->P<< std::endl;
    // P_sub1 = At*A
    SubMatrix P_sub1(m->P);
    P_sub1.rescope(size, size, 0, 0);  
    if (h.A.get_nrows() == 0)
    {
      P_sub1.setToZero();
    }
    else
    {
      CML_gemm<'t','n'>(1, h.A, h.A, 0, P_sub1);                // 'n': normale  , 't':transposee
    }
    //std::cout << "P_sub1 = " << std::endl << P_sub1 << std::endl;
    //std::cout << "P = " << std::endl <<  m->P << std::endl;
    // P_sub2 = I
    cmlDSubDenseMatrix P_sub2(m->P);
    P_sub2.rescope(h.C.get_nrows(), h.C.get_nrows(), size, size);
    P_sub2.setToIdentity();
    //std::cout << "P_sub2 = " << std::endl << P_sub2 << std::endl;
    //std::cout << "P = " << std::endl <<  m->P << std::endl;

    /////////  compute Q ///////////
    m->q.resize(size + h.C.get_nrows());
    m->q.setToZero();

    // q_sub1 = At*b
    cmlDSubDenseVector q_sub1(m->q);
    q_sub1.rescope(size,0);
    if (h.A.get_nrows() == 0)
    {
      q_sub1.setToZero();
    }
    else
    {
      CML_gemv<'t'>(1, h.A, h.b, 0, q_sub1);
    }

    // q_sub2 = 0
    cmlDSubDenseVector q_sub2(m->q);
    q_sub2.rescope(h.C.get_nrows(),size);
    q_sub2.setToZero();
    CML_scal(-1,m->q);
    //std::cout << "q_sub2 = " << std::endl << q_sub2 << std::endl;
    
    //std::cout << "P = " << std::endl <<  m->P << std::endl;
    //std::cout << "q = " << std::endl <<  m->q << std::endl;

    //add m in Matrix QP vector
    //allMatrixPQ.push_back(m);

    //return (allMatrixPQ.size()-1);
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////
  //                              Initialize Hierarchy Level barre                               //
  /////////////////////////////////////////////////////////////////////////////////////////////////

  size_t CascadeQP::initializeHierarchyLevel_barre()
  {
    // niveau 1
    HierarchyLevel_barre* hb1 = new HierarchyLevel_barre; 
    // add e in Matrix QP vector
    allHierarchyLevel_barre.push_back(hb1);

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
    //EqualitiesConstraints* e = new EqualitiesConstraints;
    EqualitiesConstraints* e = allEqualitiesConstraints[i];
    int n = allMatrixPQ[i]->P.get_ncols() ; 
    //std::cout << "n = " << std::endl <<  n<< std::endl;
    //std::cout << "n1 = " << std::endl <<  allHierarchyLevel_barre[i]->A_barre.get_ncols() << std::endl;
   
    //e->M.resize(allHierarchyLevel_barre[i]->A_barre.get_nrows(), n);
    //e->M.copyValuesFrom(allHierarchyLevel_barre[i]->A_barre);

    //e->n.resize(allHierarchyLevel_barre[i]->A_barre.get_nrows());
    //e->n.copyValuesFrom(allHierarchyLevel_barre[i]->b_barre);

    e->M.resize(allHierarchyLevel_barre[i]->A_barre.get_nrows(), allHierarchyLevel_barre[i]->A_barre.get_ncols());
    e->M.copyValuesFrom(allHierarchyLevel_barre[i]->A_barre);

    e->n.resize(allHierarchyLevel_barre[i]->A_barre.get_nrows());
    e->n.copyValuesFrom(allHierarchyLevel_barre[i]->b_barre);

    //std::cout << "M = " << std::endl <<  e->M << std::endl;
    //std::cout << "n = " << std::endl <<  e->n << std::endl;

    // add e in Matrix QP vector
    //allEqualitiesConstraints.push_back(e);
    //return (allEqualitiesConstraints.size()-1);
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
    //InequalitiesConstraints* ie = new InequalitiesConstraints; 
    InequalitiesConstraints* ie = allInequalitiesConstraints[i];


    ///// R /////
    ie->R.resize(allHierarchyLevel_barre[i]->C_barre.get_nrows() + allHierarchyLevel[i]->C.get_nrows(), std::max(allHierarchyLevel_barre[i]->C_barre.get_ncols(), allHierarchyLevel[i]->C.get_ncols()+ allHierarchyLevel[i]->C.get_nrows()));
    ie->R.setToZero();

    // R_sub1 = C_barre
    cmlDSubDenseMatrix R_sub1(ie->R);
    R_sub1.rescope(allHierarchyLevel_barre[i]->C_barre.get_nrows(),allHierarchyLevel_barre[i]->C_barre.get_ncols(),0,0);
    R_sub1.copyValuesFrom(allHierarchyLevel_barre[i]->C_barre);
    //std::cout << "R_sub1 " << std::endl << R_sub1 << std::endl;

    // R_sub2 = C
    cmlDSubDenseMatrix R_sub2(ie->R);
    R_sub2.rescope(allHierarchyLevel[i]->C.get_nrows(),allHierarchyLevel[i]->C.get_ncols(),allHierarchyLevel_barre[i]->C_barre.get_nrows(),0);   
    R_sub2.copyValuesFrom(allHierarchyLevel[i]->C);
    //std::cout << "R_sub2 " << std::endl << R_sub2 << std::endl;

    // R_sub3 = I
    cmlDSubDenseMatrix R_sub3(ie->R);
    R_sub3.rescope(allHierarchyLevel[i]->C.get_nrows(),allHierarchyLevel[i]->C.get_nrows(),allHierarchyLevel_barre[i]->C_barre.get_nrows(),allHierarchyLevel[i]->C.get_ncols());   
    R_sub3.setToIdentity();
    R_sub3.scalarMultInPlace(-1);
    //std::cout << "R_sub3 " << std::endl << R_sub3 << std::endl;

    /////// s /////
    ie->s.resize(allHierarchyLevel_barre[i]->C_barre.get_nrows() + allHierarchyLevel[i]->C.get_nrows());
    ie->s.setToZero();

    // s_sub1 = d_barre
    cmlDSubDenseVector s_sub1(ie->s);
    s_sub1.rescope(allHierarchyLevel_barre[i]->C_barre.get_nrows() , 0 );
    s_sub1.copyValuesFrom(allHierarchyLevel_barre[i]->d_barre);

    // s_sub2 = d
    cmlDSubDenseVector s_sub2(ie->s);
    s_sub2.rescope(allHierarchyLevel[i]->C.get_nrows() , allHierarchyLevel_barre[i]->C_barre.get_nrows() );
    s_sub2.copyValuesFrom(allHierarchyLevel[i]->d);

    // add e in Matrix QP vector
    //allInequalitiesConstraints.push_back(ie);
    //return (allInequalitiesConstraints.size()-1);
  }


  /////////////////////////////////////////////////////////////////////////////////////////////////
  //                                         Solve QLD                                           //
  /////////////////////////////////////////////////////////////////////////////////////////////////

  int CascadeQP::solveQLD(int i)
  {
    // TAILLE DU PROBLEME
    int n = allMatrixPQ[i]->P.get_ncols() ;                         // NUMBER OF VARIABLES 
    int n1= std::max(allEqualitiesConstraints[i]->M.get_ncols() , allInequalitiesConstraints[i]->R.get_ncols());
    int nmax = n;                                                   // ROW DIMENSION OF P
    int me = allEqualitiesConstraints[i]->M.get_nrows() ;           // NUMBER OF Equalities CONSTRAINTS
    int mi = allInequalitiesConstraints[i]->R.get_nrows() ;         // NUMBER OF Inequalities CONSTRAINTS
    int m = me + mi;                                                // TOTAL NUMBER OF CONSTRAINTS
    int mmax = m;                                                   // ROW DIMENSION OF A
    int mnn = m+n+n;                                                // MUST BE EQUAL TO M + N + N

    // SOLUTION
    //Solution* s = new Solution;
    Solution* s = allSolution[i];
    s->y.resize(n);
    s->y.setToZero();

    // BORNES
    cmlDenseVector<double> Yl(n);
    Yl.fill(-1.e10) ;
    cmlDenseVector<double> Yu(n);
    Yu.fill(1.e10) ;



    // MATRICES DE CONTRAINTES :

    ///////////////////
    //        M      //
    // MR = [   ]    //
    //        R      //
    ///////////////////

    cmlDenseMatrix<double> MR(m,n);
    //cmlDenseMatrix<double> MR;
    //MR.resize( allEqualitiesConstraints[i]->M.get_nrows() +  allInequalitiesConstraints[i]->R.get_nrows(), std::max(allEqualitiesConstraints[i]->M.get_ncols() ,  allHierarchyLevel[i]->C.get_nrows())) ;
    //MR.setToZero();
    int size = std::max(allHierarchyLevel[i]->A.get_ncols(),allHierarchyLevel[i]->C.get_ncols());
    //std::cout << "Matrix P= " << std::endl << n << std::endl;
    //std::cout << " Constraints = " << std::endl << n1 << std::endl;
    //std::cout << " size = " << std::endl << size << std::endl;

   
    
    // MR_sub1 = M
    cmlDSubDenseMatrix MR_sub1(MR);
    MR_sub1.rescope( me , allEqualitiesConstraints[i]->M.get_ncols() , 0 , 0 );
    //MR_sub1.rescope( me , n , 0 , 0 );
    MR_sub1.copyValuesFrom(allEqualitiesConstraints[i]->M);


    // MR_sub2 = R
    cmlDSubDenseMatrix MR_sub2(MR);
    MR_sub2.rescope( mi , n , me, 0 );
    MR_sub2.copyValuesFrom(allInequalitiesConstraints[i]->R);
    MR_sub2.scalarMultInPlace(-1);


    ///////////////////
    //        N      //
    // NS = [   ]    //
    //        S      //
    ///////////////////

    cmlDenseVector<double> NS(m);
    //cmlDenseVector<double> NS;
    //NS.resize( allEqualitiesConstraints[i]->M.get_nrows() +  allInequalitiesConstraints[i]->R.get_nrows()) ;
    //NS.setToZero();

    // NS_sub1 = N
    cmlDSubDenseVector NS_sub1(NS);
    NS_sub1.rescope( me , 0 );
    CML_axpy(-1, allEqualitiesConstraints[i]->n, NS_sub1);

    // NS_sub2 = S
    cmlDSubDenseVector NS_sub2(NS);
    NS_sub2.rescope( mi ,me);
    NS_sub2.copyValuesFrom(allInequalitiesConstraints[i]->s);



    // SOLVE :
    /*std::cout << "MR_sub1 = " << std::endl << MR_sub1 << std::endl;
    std::cout << "MR_sub2 = " << std::endl << MR_sub2 << std::endl;
    std::cout << "M= " << std::endl << allEqualitiesConstraints[i]->M << std::endl;
    std::cout << "R= " << std::endl << allInequalitiesConstraints[i]->R << std::endl;
    std::cout << "P= " << std::endl << allMatrixPQ[i]->P << std::endl;
    std::cout << "q= " << std::endl << allMatrixPQ[i]->q << std::endl;
    std::cout << "MR = " << std::endl << MR << std::endl;
    std::cout << "NS = " << std::endl << NS << std::endl;*/

    //std::cout << "MR = " << std::endl << MR.get_ncols() << std::endl;
    //std::cout << "NS = " << std::endl << NS.get_ncols() << std::endl

    int r = solver.solve(allMatrixPQ[i]->P, allMatrixPQ[i]->q , MR, NS, me, s->y, Yl, Yu, false);

    //for (cfl_size_t k=0; k<s->y.getSize(); ++k)   
    //std::cout << s->y[k] << std::endl;

    //allSolution.push_back(s);
    //std::cout << " SOLUTION : Y = "  << allSolution[i]->y << std::endl;


/*    std::cout << "***********LEVEL " << i << "*************" << std::endl;
    std::cout << "equalities" << std::endl;
    for (int j=0; j<me; ++j)
      std::cout << j << " : " << (solver.getLagrangeMultipliers())[j] << std::endl;

    std::cout << "inequalities" << std::endl;
    for (int j=0; j<mi; ++j)
      std::cout << j << " : " << (solver.getLagrangeMultipliers())[me+j] << std::endl;

    std::cout << "lower bound" << std::endl;
    for (int j=0; j<n; ++j)
      std::cout << j << " : " << (solver.getLagrangeMultipliers())[me+mi+j] << std::endl;

    std::cout << "upper bound" << std::endl;
    for (int j=0; j<n; ++j)
      std::cout << j << " : " << (solver.getLagrangeMultipliers())[me+mi+n+j] << std::endl;
*/
    return r;
  }


  /////////////////////////////////////////////////////////////////////////////////////////////////
  //                               Add Hierarchy Level_barre                                     //
  /////////////////////////////////////////////////////////////////////////////////////////////////

  void CascadeQP::addHierarchyLevel_barre(int i)
  {
    //HierarchyLevel_barre* h_b = new HierarchyLevel_barre; 
    HierarchyLevel_barre* h_b = allHierarchyLevel_barre[i+1]; 

    ////////////////////// calcul du nombre de contraintes violées : compteur ///////////////////

    // X : solution du niveau hierarchique k 
    cmlDSubDenseVector X(allSolution[i]->y);
    X.rescope(std::max(allHierarchyLevel[i]->A.get_ncols(),allHierarchyLevel[i]->C.get_ncols()),0);


    // Cj : ligne de la matrice C du niveau hierarchique k
    cmlDSubDenseMatrix Cj(allHierarchyLevel[i]->C);
    cmlDSubDenseVector dj(allHierarchyLevel[i]->d);

    // L : Cj*X
    cmlDenseVector<double> L;
    L.resize(allHierarchyLevel[i]->C.get_nrows());

    // compteur :
    int compteurConstraintsViolated = 0;
    int compteurConstraintsNotViolated = 0;
    indexConstraintsNotViolated.clear();
    indexConstraintsViolated.clear();

    if (allHierarchyLevel[i]->C.get_nrows() == 0)
    {
      //throw std::runtime_error("[QLDSolver::addLinearEqualitiesConstraint] added Constraint is not an equalities" );
    }
    else
    {
      CML_gemv<'n'>(1,allHierarchyLevel[i]->C,X,0,L);
      for (cfl_size_t j=0; j<allHierarchyLevel[i]->C.get_nrows(); ++j) 
      {
        //Cj.rescope(1,allHierarchyLevel[i]->C.get_ncols(),j,0);
        //std::cout << "X = " << std::endl << X << std::endl;
        //std::cout << "Cj = " << std::endl << Cj << std::endl;
        //CML_gemv<'n'>(1,Cj,X,0,L);
        //std::cout << " L = " << std::endl << L << std::endl;

        if ( L(j) <= allHierarchyLevel[i]->d(j) )
        {
          compteurConstraintsNotViolated = compteurConstraintsNotViolated + 1 ;
          indexConstraintsNotViolated.push_back(j);
        }
        else
        {
          compteurConstraintsViolated = compteurConstraintsViolated + 1 ;
          indexConstraintsViolated.push_back(j);
        }
      }
    }




    /////////////////////////////////////
    //                  A_barre(k)     //
    // A_barre(k+1) = [   A(k)   ]     //
    //                     cj          //
    /////////////////////////////////////

    h_b->A_barre.resize(allHierarchyLevel_barre[i]->A_barre.get_nrows() + allHierarchyLevel[i]->A.get_nrows()+ compteurConstraintsViolated, std::max(allHierarchyLevel_barre[i]->A_barre.get_ncols(),std::max(allHierarchyLevel[i]->A.get_ncols(), allHierarchyLevel[i]->C.get_ncols())));
    h_b->A_barre.setToZero();
    //std::cout << "A_barre = " << std::endl << h_b->A_barre << std::endl;
    // A_sub1 = A_barre(k)
    cmlDSubDenseMatrix A_sub1(h_b->A_barre);
    A_sub1.rescope(allHierarchyLevel_barre[i]->A_barre.get_nrows(), allHierarchyLevel_barre[i]->A_barre.get_ncols(), 0, 0);
    A_sub1.copyValuesFrom(allHierarchyLevel_barre[i]->A_barre);
    //std::cout << "A_sub1 = " << std::endl << A_sub1 << std::endl;

    // A_sub2 = A(k)
    cmlDSubDenseMatrix A_sub2(h_b->A_barre);
    A_sub2.rescope(allHierarchyLevel[i]->A.get_nrows(), allHierarchyLevel[i]->A.get_ncols(), allHierarchyLevel_barre[i]->A_barre.get_nrows(),0);    
    A_sub2.copyValuesFrom(allHierarchyLevel[i]->A);
    //std::cout << "A_sub2 = " << std::endl << A_sub2 << std::endl;

    // A_sub3 = cj
    cmlDSubDenseMatrix A_sub3(h_b->A_barre); 
    for (int j=0; j<compteurConstraintsViolated; ++j) 
    {
      Cj.rescope(1,allHierarchyLevel[i]->C.get_ncols(),indexConstraintsViolated[j],0);
      A_sub3.rescope(1, std::max(allHierarchyLevel[i]->A.get_ncols(), allHierarchyLevel[i]->C.get_ncols()), allHierarchyLevel_barre[i]->A_barre.get_nrows() + allHierarchyLevel[i]->A.get_nrows() + j, 0);  
      A_sub3.copyValuesFrom(Cj);
    }
    //std::cout << "A_sub3 = " << std::endl << A_sub3 << std::endl;

    //std::cout << "A_barre = " << std::endl << h_b->A_barre << std::endl;


    /////////////////////////////////////
    //                  b_barre(k)     //
    // b_barre(k+1) = [  A(k)x(k)  ]   //
    //                    cj x(k)      //
    /////////////////////////////////////

    h_b->b_barre.resize(allHierarchyLevel_barre[i]->A_barre.get_nrows() + allHierarchyLevel[i]->A.get_nrows() + compteurConstraintsViolated );
    h_b->b_barre.setToZero();

    // b_sub1 = b_barre(k)
    cmlDSubDenseVector b_sub1(h_b->b_barre);
    b_sub1.rescope(allHierarchyLevel_barre[i]->A_barre.get_nrows(),0);
    b_sub1.copyValuesFrom(allHierarchyLevel_barre[i]->b_barre);
    //std::cout << "b_sub1 = " << std::endl << b_sub1 << std::endl;


    // b_sub2 = A(k)x(k)
    cmlDSubDenseVector b_sub2(h_b->b_barre);
    b_sub2.rescope(allHierarchyLevel[i]->A.get_nrows(), allHierarchyLevel_barre[i]->A_barre.get_nrows());
    if (allHierarchyLevel[i]->A.get_nrows() == 0)
    {
      ;
    }
    else
    {
      CML_gemv<'n'>(1, allHierarchyLevel[i]->A, X, 0, b_sub2);
    }
    //std::cout << "b_sub2 = " << std::endl << b_sub2 << std::endl;


    // b_sub3 = cj x(k)
    cmlDSubDenseVector b_sub3(h_b->b_barre);
    b_sub3.rescope(compteurConstraintsViolated, allHierarchyLevel_barre[i]->A_barre.get_nrows() + allHierarchyLevel[i]->A.get_nrows());
    cmlDenseVector<double> b (compteurConstraintsViolated);
    b.setToZero();
    cmlDSubDenseVector bj(b);



    for (int j=0; j<compteurConstraintsViolated; ++j) 
    {
      b_sub3.rescope(1,  allHierarchyLevel_barre[i]->A_barre.get_nrows() + allHierarchyLevel[i]->A.get_nrows() + j );
      Cj.rescope(1,allHierarchyLevel[i]->C.get_ncols(),indexConstraintsViolated[j],0);
      CML_gemv<'n'>(1,Cj, X, 0, b_sub3);  
    }

    //std::cout << "b_sub3 = " << std::endl << b_sub3 << std::endl;

    //std::cout << "b_barre = " << std::endl << h_b->b_barre << std::endl;


    /////////////////////////////////////
    //                  C_barre(k)     //
    // C_barre(k+1) = [            ]   //
    //                     cj          //
    /////////////////////////////////////

    h_b->C_barre.resize(allHierarchyLevel_barre[i]->C_barre.get_nrows() + compteurConstraintsNotViolated , std::max(allHierarchyLevel_barre[i]->C_barre.get_ncols(),std::max(allHierarchyLevel[i]->A.get_ncols(), allHierarchyLevel[i]->C.get_ncols())) );
    h_b->C_barre.setToZero();

    // C_sub1 = C_barre(k)
    cmlDSubDenseMatrix C_sub1(h_b->C_barre);
    C_sub1.rescope(allHierarchyLevel_barre[i]->C_barre.get_nrows(), allHierarchyLevel_barre[i]->C_barre.get_ncols(), 0, 0);
    C_sub1.copyValuesFrom(allHierarchyLevel_barre[i]->C_barre);
    //std::cout << "C_sub1 = " << std::endl << C_sub1 << std::endl;


    // C_sub2 = cj
    cmlDSubDenseMatrix C_sub2(h_b->C_barre);
    for (int j=0; j<compteurConstraintsNotViolated; ++j) 
    {
      Cj.rescope(1,allHierarchyLevel[i]->C.get_ncols(),indexConstraintsNotViolated[j],0);
      C_sub2.rescope(1, std::max(allHierarchyLevel[i]->A.get_ncols(), allHierarchyLevel[i]->C.get_ncols()), allHierarchyLevel_barre[i]->C_barre.get_nrows() + j, 0);  
      C_sub2.copyValuesFrom(Cj);
    }
    //std::cout << "C_sub2 = " << std::endl << C_sub2 << std::endl;

    //std::cout << "C_barre = " << std::endl << h_b->C_barre << std::endl;

    /////////////////////////////////////
    //                  d_barre(k)     //
    // d_barre(k+1) = [            ]   //
    //                    dj           //
    /////////////////////////////////////

    h_b->d_barre.resize(allHierarchyLevel_barre[i]->C_barre.get_nrows() + compteurConstraintsNotViolated );
    h_b->d_barre.setToZero();
    //std::cout << "d_barre = " << std::endl << h_b->d_barre << std::endl;

    // d_sub1 = d_barre(k)
    cmlDSubDenseVector d_sub1(h_b->d_barre);
    d_sub1.rescope(allHierarchyLevel_barre[i]->C_barre.get_nrows(),0);
    d_sub1.copyValuesFrom(allHierarchyLevel_barre[i]->d_barre);
    //std::cout << "d_sub1= " << std::endl << d_sub1 << std::endl;

    // d_sub2 = dj
    cmlDSubDenseVector d_sub2(h_b->d_barre);
    d_sub2.rescope(compteurConstraintsNotViolated, allHierarchyLevel_barre[i]->C_barre.get_nrows() );
    for (int j=0; j<compteurConstraintsNotViolated; ++j) 
    {
      dj.rescope(1,indexConstraintsNotViolated[j]);
      d_sub2.rescope(1, allHierarchyLevel_barre[i]->C_barre.get_nrows() + j );
      d_sub2.copyValuesFrom(dj);
    }
    //std::cout << "d_sub2= " << std::endl << d_sub2 << std::endl;

    //std::cout << "d_barre = " << std::endl << h_b->d_barre << std::endl;

    //allHierarchyLevel_barre.push_back(h_b);
    //return (allHierarchyLevel_barre.size()-1);
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
    cfl_size_t nbLevel = (cfl_size_t)allHierarchyLevel.size();

    for(cfl_size_t i=0; i<nbLevel; i++)
    {
      computeMatrixPQ(i);                                    // Compute QP
      //std::cout << "P = " << std::endl << allMatrixPQ[i]->P << std::endl;
      //std::cout << "q = " << std::endl << allMatrixPQ[i]->q << std::endl;
      //writeInFile(allMatrixPQ[i]->P, "P_CASCADE_QP.txt", true);
      //writeInFile(allMatrixPQ[i]->q, "q_CASCADE_QP.txt", true);
      
      computeEqualitiesConstraints(i);                       // Compute Equalities Constraints 
      //std::cout << "M = " << std::endl << allEqualitiesConstraints[i]->M << std::endl;
      //std::cout << "n = " << std::endl << allEqualitiesConstraints[i]->n << std::endl;
      //writeInFile(allEqualitiesConstraints[i]->M , "M_CASCADE_QP.txt", true);
      //writeInFile(allEqualitiesConstraints[i]->n , "n_CASCADE_QP.txt", true);

      computeInequalitiesConstraints(i);                     // Compute Inequalities Constraints
      //std::cout << "R = " << std::endl << allInequalitiesConstraints[i]->R << std::endl;
      //std::cout << "s = " << std::endl << allInequalitiesConstraints[i]->s << std::endl;
      //writeInFile(allInequalitiesConstraints[i]->R, "R_CASCADE_QP.txt", true);
      //writeInFile(allInequalitiesConstraints[i]->s, "s_CASCADE_QP.txt", true);

      f.r = solveQLD(i);                                     // Solve QLD 
      //std::cout << "r = " << std::endl << f.r << std::endl;
      //std::cout << "solution = " << std::endl << f.yf << std::endl;
      //writeInFile(f.yf, "yf_CASCADE_QP.txt", true);

      addHierarchyLevel_barre(i);                            // Add Hierarchy Level_barre

      //printf("[return]"); getchar();
    }


    //   f.r = solveQLD(allHierarchyLevel.size()-1);
    //   f.yf.resize(std::max(allHierarchyLevel[nbLevel-1]->A.get_ncols(), allHierarchyLevel[nbLevel-1]->C.get_ncols()));
    f.yf.resize(allSolution[nbLevel-1]->y.getSize());
    f.yf.copyValuesFrom(allSolution[nbLevel-1]->y);


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
  const cmlDenseVector<double>& CascadeQP::getSolutionOfLevel(int i) const
  {
    return allSolution[i]->y;
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////
  //                                   get Hierarchy Level                                       //
  /////////////////////////////////////////////////////////////////////////////////////////////////
  const HierarchyLevel& CascadeQP::getHierarchyLevel(int i) const
  {
    return (*allHierarchyLevel[i]);
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


  std::ostream & operator<<(std::ostream &out, const HierarchyLevel& h )
  {
    out << "A = " << std::endl << h.A  << std::endl << "b = " << std::endl << h.b  << std::endl << "C = " << std::endl << h.C  << std::endl << "d = " << std::endl << h.d  << std::endl << std::endl; 
    return out;
  }

  std::ostream & operator<<(std::ostream &out, const HierarchyLevel_barre& h )
  {
    out << "A_barre = " << std::endl << h.A_barre  << std::endl << "b_barre = " << std::endl << h.b_barre  << std::endl << "C_barre = " << std::endl << h.C_barre  << std::endl << "d_barre = " << std::endl << h.d_barre  << std::endl << std::endl; 
    return out;
  }


  std::ostream & operator<<(std::ostream &out, const MatrixPQ& m )
  {
    out << "P = " << std::endl << m.P  << std::endl << "q =" << std::endl << m.q  << std::endl << std::endl; 
    return out;
  }


  std::ostream & operator<<(std::ostream &out, const EqualitiesConstraints& e )
  {
    out << "M = " << std::endl << e.M  << std::endl << "N =" << std::endl << e.n  << std::endl << std::endl; 
    return out;
  }


  std::ostream & operator<<(std::ostream &out, const InequalitiesConstraints& ie )
  {
    out << "R = " << std::endl << ie.R  << std::endl << "S =" << std::endl << ie.s  << std::endl << std::endl; 
    return out;
  }


  std::ostream & operator<<(std::ostream &out, const FinalSolution& sf )
  {
    out << "yf = " << std::endl << sf.yf  << std::endl << "r =" << std::endl << sf.r  << std::endl << std::endl; 
    return out;
  }

}

#endif

// cmake:sourcegroup=toBeUpdated
