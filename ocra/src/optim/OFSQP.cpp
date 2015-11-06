#include "OFSQP.h"
#include "ObjQLD.h"
#include <algorithm>    //for min and max
#include <assert.h>

namespace ocra
{
  OFSQPProblem::OFSQPProblem()
    :x_is_new(true), glob_grd(0x0), glob_prnt(0x0)
  {
  }

  void OFSQPProblem::gradob(int nparam,int j,double *x,double *gradf,void *cd)
  {
    assert(glob_grd != 0x0 && glob_prnt != 0x0 && "this default implementation is made so far to be called within fsqp routine only");

    int i;
    double xi,delta;

    for (i=0; i<=nparam-1; i++) {
      xi=x[i];
      delta=std::max(glob_grd->udelta,
          glob_grd->rteps*std::max(1.e0,fabs(xi)));
      if (xi<0.e0) delta=-delta;
      if (!(glob_prnt->ipd==1 || j != 1 || glob_prnt->iprint<3)) {
        /*  formats are not set yet...  */
        if (i==0) fprintf(glob_prnt->io,"\tdelta(i)\t %22.14f\n",delta);
        if (i!=0) fprintf(glob_prnt->io,"\t\t\t %22.14f\n",delta);
      }
      x[i]=xi+delta;
      invalidateX();
      obj(nparam,j,x,&gradf[i],cd);
      gradf[i]=(gradf[i]-glob_grd->valnom)/delta;
      x[i]=xi;
      invalidateX();
    }
    return;
  }


  void OFSQPProblem::gradcn(int nparam,int j,double *x,double *gradg,void *cd)
  {
    assert(glob_grd != 0x0 && glob_prnt != 0x0 && "this default implementation is made so far to be called within fsqp routine only");

    int i;
    double xi,delta;

    for (i=0; i<=nparam-1; i++) {
      xi=x[i];
      delta=std::max(glob_grd->udelta,
          glob_grd->rteps*std::max(1.e0,fabs(xi)));
      if (xi<0.e0) delta=-delta;
      if (!(j != 1 || glob_prnt->iprint<3)) {
        /*  formats are not set yet...  */
        if (i==0) fprintf(glob_prnt->io,"\tdelta(i)\t %22.14f\n",delta);
        if (i!=0) fprintf(glob_prnt->io,"\t\t\t %22.14f\n",delta);
        glob_prnt->ipd=1;
      }
      x[i]=xi+delta;
      invalidateX();
      constr(nparam,j,x,&gradg[i],cd);
      gradg[i]=(gradg[i]-glob_grd->valnom)/delta;
      x[i]=xi;
      invalidateX();
    }
    return;
  }

  void OFSQPProblem::invalidateX()
  {
    x_is_new = true;
  }

  void OFSQPProblem::validateX()
  {
    x_is_new = false;
  }

  bool OFSQPProblem::isXNew()
  {
    return x_is_new;
  }

  void OFSQPProblem::setFSQPstruct(fsqpDetails::Grd& grd, fsqpDetails::Prnt& prnt)
  {
    glob_grd = &grd;
    glob_prnt = &prnt;
  }

  void OFSQPProblem::resetFSQPstruct()
  {
    glob_grd = 0x0;
    glob_prnt = 0x0;
  }


  OFSQP::OFSQP(void) 
    : cfsqp_version("CFSQP 2.5d")
  {
    //this variables needs to be initialized for optimized version
    objeps = 0;
    objrep = 0;
    gLgeps = 0;
    nstop = 0;
  }


  OFSQP::~OFSQP(void)
  {
  }


  void
    OFSQP::cfsqp(int nparam,int nf,int nfsr,int nineqn,int nineq,int neqn,
        int neq,int ncsrl,int ncsrn,int *mesh_pts,
        int mode,int iprint,int miter,int *inform,double bigbnd,
        double eps,double epseqn,double udelta,double *bl,double *bu,
        double *x,double *f,double *g,double *lambda, OFSQPProblem& problem, void *cd)

    /*---------------------------------------------------------------------
     * Brief specification of various arrays and parameters in the calling
     * sequence. See manual for a more detailed description.
     *
     * nparam : number of variables
     * nf     : number of objective functions (count each set of sequentially
     *          related objective functions once)
     * nfsr   : number of sets of sequentially related objectives (possibly
     *          zero)
     * nineqn : number of nonlinear inequality constraints
     * nineq  : total number of inequality constraints
     * neqn   : number of nonlinear equality constraints
     * neq    : total number of equality constraints
     * ncsrl  : number of sets of linear sequentially related inequality
     *          constraints
     * ncsrn  : number of sets of nonlinear sequentially related inequality
     *          constraints
     * mesh_pts : array of integers giving the number of actual objectives/
     *            constraints in each sequentially related objective or
     *            constraint set. The order is as follows:
     *            (i) objective sets, (ii) nonlinear constraint sets,
     *            (iii) linear constraint sets. If one or no sequentially
     *            related constraint or objectives sets are present, the
     *            user may simply pass the address of an integer variable
     *            containing the appropriate number (possibly zero).
     * mode   : mode=CBA specifies job options as described below:
     *          A = 0 : ordinary minimax problems
     *            = 1 : ordinary minimax problems with each individual
     *                  function replaced by its absolute value, ie,
     *                  an L_infty problem
     *          B = 0 : monotone decrease of objective function
     *                  after each iteration
     *            = 1 : monotone decrease of objective function after
     *                  at most four iterations
     *          C = 1 : default operation.
     *            = 2 : requires that constraints always be evaluated
     *                  before objectives during the line search.
     * iprint : print level indicator with the following options-
     *          iprint=0: no normal output, only error information
     *                    (this option is imposed during phase 1)
     *          iprint=1: a final printout at a local solution
     *          iprint=2: a brief printout at the end of each iteration
     *          iprint=3: detailed infomation is printed out at the end
     *                    of each iteration (for debugging purposes)
     *          For iprint=2 or 3, the information may be printed at
     *          iterations that are multiples of 10, instead of every
     *          iteration. This may be done by adding the desired number
     *          of iterations to skip printing to the desired iprint value
     *          as specified above. e.g., sending iprint=23 would give
     *          the iprint=3 information once every 20 iterations.
     * miter  : maximum number of iterations allowed by the user to solve
     *          the problem
     * inform : status report at the end of execution
     *          inform= 0:normal termination
     *          inform= 1:no feasible point found for linear constraints
     *          inform= 2:no feasible point found for nonlinear constraints
     *          inform= 3:no solution has been found in miter iterations
     *          inform= 4:stepsize smaller than machine precision before
     *                    a successful new iterate is found
     *          inform= 5:failure in attempting to construct d0
     *          inform= 6:failure in attempting to construct d1
     *          inform= 7:inconsistent input data
     *          inform= 8:new iterate essentially identical to previous
     *                    iterate, though stopping criterion not satisfied.
     *          inform= 9:penalty parameter too large, unable to satisfy
     *                    nonlinear equality constraint
     * bigbnd : plus infinity
     * eps    : stopping criterion. Execution stopped when the norm of the
     *          Newton direction vector is smaller than eps
     * epseqn : tolerance of the violation of nonlinear equality constraints
     *          allowed by the user at an optimal solution
    * udelta : perturbation size in computing gradients by finite
    *          difference. The actual perturbation is determined by
    *          sign(x_i) X max{udelta, rteps X max{1, |x_i|}} for each
    *          component of x, where rteps is the square root of machine
    *          precision.
    * bl     : array of dimension nparam,containing lower bound of x
    * bu     : array of dimension nparam,containing upper bound of x
    * x      : array of dimension nparam,containing initial guess in input
    *          and final iterate at the end of execution
    * f      : array of dimension sufficient enough to hold the value of
    *          all regular objective functions and the value of all
    *          members of the sequentially related objective sets.
    *          (dimension must be at least 1)
    * g      : array of dimension sufficient enough to hold the value of
    *          all regular constraint functions and the value of all
    *          members of the sequentially related constraint sets.
    *          (dimension must be at least 1)
    * lambda : array of dimension nparam+dim(f)+dim(g), containing
    *          Lagrange multiplier values at x in output. (A concerns the
        *          mode, see above). The first nparam positions contain the
    *          multipliers associated with the simple bounds, the next
    *          dim(g) positions contain the multipliers associated with
    *          the constraints. The final dim(f) positions contain the
    *          multipliers associated with the objective functions. The
    *          multipliers are in the order they were specified in the
    *          user-defined objective and constraint functions.
    * obj    : Pointer to function that returns the value of objective
    *          functions, one upon each call
    * constr : Pointer to function that returns the value of constraints
    *          one upon each call
    * gradob : Pointer to function that computes gradients of f,
    *          alternatively it can be replaced by grobfd to compute
      *          finite difference approximations
      * gradcn : Pointer to function that computes gradients of g,
    *          alternatively it can be replaced by grcnfd to compute
      *          finite difference approximations
      * cd     : Void pointer that may be used by the user for the passing of
      *          "client data" (untouched by CFSQP)
      *
      *----------------------------------------------------------------------
      *
      *
      *                       CFSQP  Version 2.5d
      *
      *                  Craig Lawrence, Jian L. Zhou
      *                         and Andre Tits
      *                  Institute for Systems Research
      *                               and
      *                Electrical Engineering Department
      *                     University of Maryland
      *                     College Park, Md 20742
      *
      *                         February, 1998
      *
      *
      *  The purpose of CFSQP is to solve general nonlinear constrained
      *  minimax optimization problems of the form
      *
      *   (A=0 in mode)     minimize    max_i f_i(x)   for i=1,...,n_f
      *                        or
      *   (A=1 in mode)     minimize    max_j |f_i(x)|   for i=1,...,n_f
      *                       s.t.      bl   <= x <=  bu
      *                                 g_j(x) <= 0,   for j=1,...,nineqn
      *                                 A_1 x - B_1 <= 0
      *
      *                                 h_i(x)  = 0,   for i=1,...,neqn
      *                                 A_2 x - B_2  = 0
      *
      * CFSQP is also able to efficiently handle problems with large sets of
      * sequentially related objectives or constraints, see the manual for
      * details.
      *
      *
      *                  Conditions for External Use
      *                  ===========================
      *
      *   1. The CFSQP routines may not be distributed to third parties.
      *      Interested parties shall contact AEM Design directly.
      *   2. If modifications are performed on the routines, these
      *      modifications shall be communicated to AEM Design.  The
      *      modified routines will remain the sole property of the authors.
      *   3. Due acknowledgment shall be made of the use of the CFSQP
      *      routines in research reports or publications. Whenever
      *      such reports are released for public access, a copy shall
      *      be forwarded to AEM Design.
      *   4. The CFSQP routines may only be used for research and
      *      development, unless it has been agreed otherwise with AEM
      *      Design in writing.
      *
      * Copyright (c) 1993-1998 by Craig T. Lawrence, Jian L. Zhou, and
      *                         Andre L. Tits
      * All Rights Reserved.
      *
      *
      * Enquiries should be directed to:
      *
      *      AEM Design
      *      3754 LaVista Rd., Suite 250
      *      Tucker, GA 30084
      *      U. S. A.
      *
      *      Phone : 678-990-1121
      *      Fax   : 678-990-1122
      *      E-mail: info@aemdesign.com
      *
      *  References:
      *  [1] E. Panier and A. Tits, `On Combining Feasibility, Descent and
      *      Superlinear Convergence In Inequality Constrained Optimization',
    *      Mathematical Programming, Vol. 59(1993), 261-276.
      *  [2] J. F. Bonnans, E. Panier, A. Tits and J. Zhou, `Avoiding the
      *      Maratos Effect by Means of a Nonmonotone Line search: II.
      *      Inequality Problems - Feasible Iterates', SIAM Journal on
      *      Numerical Analysis, Vol. 29, No. 4, 1992, pp. 1187-1202.
      *  [3] J.L. Zhou and A. Tits, `Nonmonotone Line Search for Minimax
      *      Problems', Journal of Optimization Theory and Applications,
    *      Vol. 76, No. 3, 1993, pp. 455-476.
      *  [4] C.T. Lawrence, J.L. Zhou and A. Tits, `User's Guide for CFSQP
      *      Version 2.5: A C Code for Solving (Large Scale) Constrained
      *      Nonlinear (Minimax) Optimization Problems, Generating Iterates
      *      Satisfying All Inequality Constraints,' Institute for
      *      Systems Research, University of Maryland,Technical Report
      *      TR-94-16r1, College Park, MD 20742, 1997.
      *  [5] C.T. Lawrence and A.L. Tits, `Nonlinear Equality Constraints
      *      in Feasible Sequential Quadratic Programming,' Optimization
      *      Methods and Software, Vol. 6, March, 1996, pp. 265-282.
      *  [6] J.L. Zhou and A.L. Tits, `An SQP Algorithm for Finely
      *      Discretized Continuous Minimax Problems and Other Minimax
      *      Problems With Many Objective Functions,' SIAM Journal on
      *      Optimization, Vol. 6, No. 2, May, 1996, pp. 461--487.
      *  [7] C. T. Lawrence and A. L. Tits, `Feasible Sequential Quadratic
      *      Programming for Finely Discretized Problems from SIP,'
      *      To appear in R. Reemtsen, J.-J. Ruckmann (eds.): Semi-Infinite
                                                              *      Programming, in the series Nonconcex Optimization and its
                                                              *      Applications. Kluwer Academic Publishers, 1998.
                                                              *
                                                              ***********************************************************************
                                                              */
  {
    pb = &problem;
    pb->invalidateX();
    pb->setFSQPstruct(glob_grd, glob_prnt);

    int  i,ipp,j,ncnstr,nclin,nctotl,nob,nobL,modem,nn,
         nppram,nrowa,ncsipl1,ncsipn1,nfsip1;
    bool feasbl,feasb,prnt,Linfty;
    int  *indxob,*indxcn,*mesh_pts1;
    double *signeq;
    double xi,gi,gmax,dummy,epskt;
    struct _constraint *cs;      /* pointer to array of constraints */
    struct _objective  *ob;      /* pointer to array of objectives  */
    struct _parameter  *param;   /* pointer to parameter structure  */
    struct _parameter  _param;

    /*     Make adjustments to parameters for SIP constraints       */
    glob_info.tot_actf_sip = glob_info.tot_actg_sip = 0;
    mesh_pts=mesh_pts-1;
    glob_info.nfsip=nfsr;
    glob_info.ncsipl=ncsrl;
    glob_info.ncsipn=ncsrn;
    nf=nf-nfsr;
    nfsip1=nfsr;
    nfsr=0;
    for (i=1; i<=nfsip1; i++)
      nfsr=nfsr+mesh_pts[i];
    nf=nf+nfsr;
    nineqn=nineqn-ncsrn;
    nineq=nineq-ncsrl-ncsrn;
    ncsipl1=ncsrl;
    ncsipn1=ncsrn;
    ncsrl=0;
    ncsrn=0;
    if (ncsipn1)
      for (i=1; i<=ncsipn1; i++)
        ncsrn=ncsrn+mesh_pts[nfsip1+i];
    if (ncsipl1)
      for (i=1; i<=ncsipl1; i++)
        ncsrl=ncsrl+mesh_pts[nfsip1+ncsipn1+i];
    nineqn=nineqn+ncsrn;
    nineq=nineq+ncsrn+ncsrl;
    /* Create array of constraint structures               */
    cs=(struct _constraint *)calloc(nineq+neq+1,
        sizeof(struct _constraint));
    for (i=1; i<=nineq+neq; i++) {
      cs[i].grad=make_dv(nparam);
      cs[i].act_sip=false;
      cs[i].d1bind=false;
    }
    /* Create parameter structure                          */
    _param.x=make_dv(nparam+1);
    _param.bl=make_dv(nparam);
    _param.bu=make_dv(nparam);
    _param.mult=make_dv(nparam+1);
    param=&_param;

    /*   Initialize, compute the machine precision, etc.   */
    bl=bl-1; bu=bu-1; x=x-1;
    for (i=1; i<=nparam; i++) {
      param->x[i]=x[i];
      param->bl[i]=bl[i];
      param->bu[i]=bu[i];
    }
    param->cd=cd;    /* Initialize client data */
    dummy=0.e0;
    f=f-1; g=g-1; lambda=lambda-1;
    glob_prnt.iter=0;
    nstop=1;
    nn=nineqn+neqn;
    glob_grd.epsmac=small();
    tolfea=glob_grd.epsmac*1.e2;
    bgbnd=bigbnd;
    glob_grd.rteps=sqrt(glob_grd.epsmac);
    glob_grd.udelta=udelta;
    glob_log.rhol_is1=false;
    glob_log.get_ne_mult=false;
    signeq=make_dv(neqn);

    nob=0;
    gmax=-bgbnd;
    glob_prnt.info=0;
    glob_prnt.iprint=iprint%10;
    ipp=iprint;
    glob_prnt.iter_mod=std::max(iprint-iprint%10,1);
    glob_prnt.io=stdout;
    ncnstr=nineq+neq;
    glob_info.nnineq=nineq;
    if (glob_prnt.iprint>0) {
      fprintf(glob_prnt.io,
          "\n\n  CFSQP Version 2.5d (Released February 1998) \n");
      fprintf(glob_prnt.io,
          "          Copyright (c) 1993 --- 1998       \n");
      fprintf(glob_prnt.io,
          "           C.T. Lawrence, J.L. Zhou         \n");
      fprintf(glob_prnt.io,
          "                and A.L. Tits               \n");
      fprintf(glob_prnt.io,
          "             All Rights Reserved            \n\n");
    }
    /*-----------------------------------------------------*/
    /*   Check the input data                              */
    /*-----------------------------------------------------*/
    check(nparam,nf,nfsr,&Linfty,nineq,nineqn,neq,neqn,
        ncsrl,ncsrn,mode,&modem,eps,bgbnd,param);
    if (glob_prnt.info==7) {
      *inform=glob_prnt.info;
      pb->resetFSQPstruct();
      return;
    }

    maxit=std::max(std::max(miter,10*std::max(nparam,ncnstr)),1000);
    feasb=true;
    feasbl=true;
    prnt=false;
    nppram=nparam+1;

    /*-----------------------------------------------------*/
    /*   Check whether x is within bounds                  */
    /*-----------------------------------------------------*/
    for (i=1; i<=nparam; i++) {
      xi=param->x[i];
      if (param->bl[i]<=xi && param->bu[i]>=xi) continue;
      feasbl=false;
      break;
    }
    nclin=ncnstr-nn;
    /*-----------------------------------------------------*/
    /*   Check whether linear constraints are feasbile     */
    /*-----------------------------------------------------*/
    if (nclin!=0) {
      for (i=1; i<=nclin; i++) {
        j=i+nineqn;
        if (j<=nineq) {
          pb->constr(nparam,j,(param->x)+1,&gi,param->cd);
          if (gi>glob_grd.epsmac) feasbl=false;
        } else {
          pb->constr(nparam,j+neqn,(param->x)+1,&gi,param->cd);
          if (fabs(gi)>glob_grd.epsmac) feasbl=false;
        }
        cs[j].val=gi;
      }
    }
    /*-------------------------------------------------------*/
    /*   Generate a new point if infeasible                  */
    /*-------------------------------------------------------*/
    if (!feasbl) {
      if (glob_prnt.iprint>0) {
        fprintf(glob_prnt.io,
            " The given initial point is infeasible for inequality\n");
        fprintf(glob_prnt.io,
            " constraints and linear equality constraints:\n");
        sbout1(glob_prnt.io,nparam,"                    ",dummy,
            param->x,2,1);
        prnt=true;
      }
      nctotl=nparam+nclin;
      lenw=2*nparam*nparam+10*nparam+2*nctotl+1;
      leniw=std::max(2*nparam+2*nctotl+3,2*nclin+2*nparam+6);
      /*-----------------------------------------------------*/
      /*   Attempt to generate a point satisfying all linear */
      /*   constraints.                                      */
      /*-----------------------------------------------------*/
      nrowa=std::max(nclin,1);
      iw=make_iv(leniw);
      w=make_dv(lenw);
      initpt(nparam,nineqn,neq,neqn,nclin,nctotl,nrowa,param,
          &cs[nineqn]);
      free_iv(iw);
      free_dv(w);
      if (glob_prnt.info!=0) {
        *inform=glob_prnt.info;
        pb->resetFSQPstruct();
        return;
      }
    }
    indxob=make_iv(std::max(nineq+neq,nf));
    indxcn=make_iv(nineq+neq);
L510:
    if (glob_prnt.info!=-1) {
      for (i=1; i<=nineqn; i++) {
        pb->constr(nparam,i,(param->x)+1,&(cs[i].val),param->cd);
        if (cs[i].val>0.e0) feasb=false;
      }
      glob_info.ncallg=nineqn;
      if (!feasb) {
        /* Create array of objective structures for Phase 1  */
        ob=(struct _objective *)calloc(nineqn+1,
            sizeof(struct _objective));
        for (i=1; i<=nineqn; i++) {
          ob[i].grad=make_dv(nparam);
          ob[i].act_sip=false;
        }
        for (i=1; i<=nineqn; i++) {
          nob++;
          indxob[nob]=i;
          ob[nob].val=cs[i].val;
          gmax=std::max(gmax,ob[nob].val);
        }
        for (i=1; i<=nineq-nineqn; i++)
          indxcn[i]=nineqn+i;
        for (i=1; i<=neq-neqn; i++)
          indxcn[i+nineq-nineqn]=nineq+neqn+i;
        goto L605;
      }
    }

    /* Create array of objective structures for Phase 2 and      */
    /* initialize.                                               */
    ob=(struct _objective *)calloc(nf+1,sizeof(struct _objective));
    for (i=1; i<=nf; i++) {
      ob[i].grad=make_dv(nparam);
      ob[i].act_sip=false;
    }
    for (i=1; i<=nineqn; i++) {
      indxcn[i]=i;
    }
    for (i=1; i<=neq-neqn; i++)
      cs[i+nineq+neqn].val=cs[i+nineq].val;
    for (i=1; i<=neqn; i++) {
      j=i+nineq;
      pb->constr(nparam,j,(param->x)+1,&(cs[j].val),param->cd);
      indxcn[nineqn+i]=j;
    }
    for (i=1; i<=nineq-nineqn; i++)
      indxcn[i+nn]=nineqn+i;
    for (i=1; i<=neq-neqn; i++)
      indxcn[i+nineq+neqn]=nineq+neqn+i;
    glob_info.ncallg+=neqn;

L605:
    if (glob_prnt.iprint>0 && feasb && !prnt) {
      fprintf(glob_prnt.io,
          "The given initial point is feasible for inequality\n");
      fprintf(glob_prnt.io,
          "         constraints and linear equality constraints:\n");
      sbout1(glob_prnt.io,nparam,"                    ",dummy,
          param->x,2,1);
      prnt=true;
    }
    if (nob==0) {
      if (glob_prnt.iprint>0) {
        if (glob_prnt.info!=0) {
          fprintf(glob_prnt.io,
              "To generate a feasible point for nonlinear inequality\n");
          fprintf(glob_prnt.io,
              "constraints and linear equality constraints, ");
          fprintf(glob_prnt.io,"ncallg = %10d\n",glob_info.ncallg);
          if (ipp==0)
            fprintf(glob_prnt.io," iteration           %26d\n",
                glob_prnt.iter);
          if (ipp>0)
            fprintf(glob_prnt.io," iteration           %26d\n",
                glob_prnt.iter-1);
          if (ipp==0) glob_prnt.iter++;
        }
        if (feasb && !feasbl) {
          fprintf(glob_prnt.io,
              "Starting from the generated point feasible for\n");
          fprintf(glob_prnt.io,
              "inequality constraints and linear equality constraints:\n");
          sbout1(glob_prnt.io,nparam,"                    ",
              dummy,param->x,2,1);

        }
        if (glob_prnt.info!=0 || !prnt || !feasb) {
          fprintf(glob_prnt.io,
              "Starting from the generated point feasible for\n");
          fprintf(glob_prnt.io,
              "inequality constraints and linear equality constraints:\n");
          sbout1(glob_prnt.io,nparam,"                    ",
              dummy,param->x,2,1);
        }
      }
      feasb=true;
      feasbl=true;
    }
    if (ipp>0 && !feasb && !prnt) {
      fprintf(glob_prnt.io,
          " The given initial point is infeasible for inequality\n");
      fprintf(glob_prnt.io,
          " constraints and linear equality constraints:\n");
      sbout1(glob_prnt.io,nparam,"                    ",dummy,
          param->x,2,1);
      prnt=true;
    }
    if (nob==0) nob=1;
    if (feasb) {
      nob=nf;
      glob_prnt.info=0;
      glob_prnt.iprint=iprint%10;
      ipp=iprint;
      glob_prnt.iter_mod=std::max(iprint-iprint%10,1);
      glob_info.mode=modem;
      epskt=eps;
      if (Linfty) nobL=2*nob;
      else nobL=nob;
      if (nob!=0 || neqn!=0) goto L910;
      fprintf(glob_prnt.io,
          "current feasible iterate with no objective specified\n");
      *inform=glob_prnt.info;
      for (i=1; i<=nineq+neq; i++)
        g[i]=cs[i].val;
      dealloc(nineq,neq,signeq,indxcn,indxob,cs,param);
      free((char *) ob);
      pb->resetFSQPstruct();
      return;
    }
    ipp=0;
    glob_info.mode=0;
    nobL=nob;
    glob_prnt.info=-1;
    epskt=1.e-10;
L910:
    nctotl=nppram+ncnstr+std::max(nobL,1);
    leniw=2*(ncnstr+std::max(nobL,1))+2*nppram+6;
    lenw=2*nppram*nppram+10*nppram+6*(ncnstr+std::max(nobL,1)+1);
    glob_info.M=4;
    if (modem==1 && nn==0) glob_info.M=3;

    param->x[nparam+1]=gmax;
    if (feasb) {
      for (i=1; i<=neqn; i++) {
        if (cs[i+nineq].val>0.e0) signeq[i]=-1.e0;
        else signeq[i]=1.e0;
      }
    }
    if (!feasb) {
      ncsipl1=ncsrl;
      ncsipn1=0;
      nfsip1=ncsrn;
      mesh_pts1=&mesh_pts[glob_info.nfsip];
    } else {
      ncsipl1=ncsrl;
      ncsipn1=ncsrn;
      nfsip1=nfsr;
      mesh_pts1=mesh_pts;
    }
    /*---------------------------------------------------------------*/
    /*    either attempt to generate a point satisfying all          */
    /*    constraints or try to solve the original problem           */
    /*---------------------------------------------------------------*/
    nrowa=std::max(ncnstr+std::max(nobL,1),1);
    w=make_dv(lenw);
    iw=make_iv(leniw);

    cfsqp1(miter,nparam,nob,nobL,nfsip1,nineqn,neq,neqn,ncsipl1,ncsipn1,
        mesh_pts1,ncnstr,nctotl,nrowa,feasb,epskt,epseqn,indxob,
        indxcn,param,cs,ob,signeq);

    free_iv(iw);
    free_dv(w);
    if (glob_prnt.info==-1) { /* Successful phase 1 termination  */
      for (i=1; i<=nob; i++)
        cs[i].val=ob[i].val;
      nob=0;
      for (i=1; i<=nineqn; i++)
        free_dv(ob[i].grad);
      free((char *) ob);
      goto L510;
    }
    if (glob_prnt.info!=0) {
      if (feasb) {
        for (i=1; i<=nparam; i++)
          x[i]=param->x[i];
        for (i=1; i<=nineq+neq; i++)
          g[i]=cs[i].val;
        *inform=glob_prnt.info;
        dealloc(nineq,neq,signeq,indxcn,indxob,cs,param);
        for (i=1; i<=nf; i++) {
          f[i]=ob[i].val;
          free_dv(ob[i].grad);
        }
        free((char *) ob);
        pb->resetFSQPstruct();
        return;
      }
      glob_prnt.info=2;
      fprintf(glob_prnt.io,
          "Error: No feasible point is found for nonlinear inequality\n");
      fprintf(glob_prnt.io,
          "constraints and linear equality constraints\n");
      *inform=glob_prnt.info;
      dealloc(nineq,neq,signeq,indxcn,indxob,cs,param);
      for (i=1; i<=nineqn; i++)
        free_dv(ob[i].grad);
      free((char *) ob);
      pb->resetFSQPstruct();
      return;
    }
    /* Successful phase 2 termination                            */
    *inform=glob_prnt.info;
    for (i=1; i<=nparam; i++) {
      x[i]=param->x[i];
      lambda[i]=param->mult[i];
    }
    for (i=1; i<=nineq+neq; i++) {
      g[i]=cs[i].val;
      lambda[i+nparam]=cs[i].mult;
    }
    for (i=1; i<=nf; i++) {
      f[i]=ob[i].val;
      lambda[i+nparam+nineq+neq]=ob[i].mult;
      free_dv(ob[i].grad);
    }
    /* If just one objective, set multiplier=1 */
    if (nf==1) lambda[1+nparam+nineq+neq]=1.e0;
    free((char *) ob);
    dealloc(nineq,neq,signeq,indxcn,indxob,cs,param);
    pb->resetFSQPstruct();
    return;
  }


  /***************************************************************/
  /*     Free allocated memory                                   */
  /***************************************************************/


  void
    OFSQP::dealloc(int nineq,int neq,double *signeq,int *indxob,
        int *indxcn,struct _constraint *cs,struct _parameter *param)
    {
      int i;

      free_dv(param->x);
      free_dv(param->bl);
      free_dv(param->bu);
      free_dv(param->mult);
      free_dv(signeq);
      free_iv(indxob);
      free_iv(indxcn);
      for (i=1; i<=nineq+neq; i++)
        free_dv(cs[i].grad);
      free((char *) cs);
    }


  void
    OFSQP::cfsqp1(int miter,int nparam,int nob,int nobL,int nfsip,int nineqn,
        int neq,int neqn,int ncsipl,int ncsipn,int *mesh_pts,int ncnstr,
        int nctotl,int nrowa,int feasb,double epskt,double epseqn,
        int *indxob,int *indxcn,struct _parameter *param,
        struct _constraint *cs, struct _objective *ob,double *signeq)
    {
      int   i,iskp,nfs,ncf,ncg,nn,nstart,nrst,ncnst1;
      int   *iact,*iskip,*istore;
      double Cbar,Ck,dbar,fmax,fM,fMp,steps,d0nm,dummy,
             sktnom,scvneq,grdftd,psf;
      double *di,*d,*gm,*grdpsf,*penp,*bl,*bu,*clamda,
             *cvec,*psmu,*span,*backup;
      double **hess,**hess1,**a;
      double *tempv;
      struct _violation *viol;
      struct _violation _viol;

      /*   Allocate memory                              */

      hess=make_dm(nparam,nparam);
      hess1=make_dm(nparam+1,nparam+1);
      a=make_dm(nrowa,nparam+2);
      di=make_dv(nparam+1);
      d=make_dv(nparam+1);
      gm=make_dv(4*neqn);
      grdpsf=make_dv(nparam);
      penp=make_dv(neqn);
      bl=make_dv(nctotl);
      bu=make_dv(nctotl);
      clamda=make_dv(nctotl+nparam+1);
      cvec=make_dv(nparam+1);
      psmu=make_dv(neqn);
      span=make_dv(4);
      backup=make_dv(nob+ncnstr);
      iact=make_iv(nob+nineqn+neqn);
      iskip=make_iv(glob_info.nnineq+1);
      istore=make_iv(nineqn+nob);

      viol=&_viol;
      viol->index=0;
      viol->type=NONE;

      glob_prnt.initvl=1;
      glob_log.first=true;
      nrst=glob_prnt.ipd=0;
      dummy=0.e0;
      scvneq=0.e0;
      steps=0.e0;
      sktnom=0.e0;
      d0nm=0.e0;
      if (glob_prnt.iter==0) diagnl(nparam,1.e0,hess);
      if (feasb) {
        glob_log.first=true;
        if (glob_prnt.iter>0) glob_prnt.iter--;
        if (glob_prnt.iter!=0) diagnl(nparam,1.e0,hess);
      }
      Ck=Cbar=1.e-2;
      dbar=5.e0;
      nstart=1;
      glob_info.ncallf=0;
      nstop=1;
      nfs=0;
      if (glob_info.mode!=0)
        nfs=glob_info.M;
      if (feasb) {
        nn=nineqn+neqn;
        ncnst1=ncnstr;
      } else {
        nn=0;
        ncnst1=ncnstr-nineqn-neqn;
      }
      scvneq=0.e0;
      for (i=1; i<=ncnst1; i++) {
        glob_grd.valnom=cs[indxcn[i]].val;
        backup[i]=glob_grd.valnom;
        if (feasb && i>nineqn && i<=nn) {
          gm[i-nineqn]=glob_grd.valnom*signeq[i-nineqn];
          scvneq=scvneq+fabs(glob_grd.valnom);
        }
        if (feasb && i<=nn) {
          iact[i]=indxcn[i];
          if (i<=nineqn) istore[i]=0;
          if (i>nineqn) penp[i-nineqn]=2.e0;
        }
        pb->gradcn(nparam,indxcn[i],(param->x)+1,(cs[indxcn[i]].grad)+1,param->cd);
      }
      nullvc(nparam,grdpsf);
      psf=0.e0;
      if (feasb && neqn!=0)
        resign(nparam,neqn,&psf,grdpsf,penp,cs,signeq,12,12);
      fmax=-bgbnd;
      for (i=1; i<=nob; i++) {
        if (!feasb) {
          glob_grd.valnom=ob[i].val;
          iact[i]=i;
          istore[i]=0;
          pb->gradcn(nparam,indxob[i],(param->x)+1,(ob[i].grad)+1,param->cd);
        } else {
          iact[nn+i]=i;
          istore[nineqn+i]=0;
          pb->obj(nparam,i,(param->x)+1,&(ob[i].val),param->cd);
          glob_grd.valnom=ob[i].val;
          backup[i+ncnst1]=glob_grd.valnom;
          pb->gradob(nparam,i,(param->x)+1,(ob[i].grad)+1,param->cd);
          glob_info.ncallf++;
          if (nobL!=nob) fmax=std::max(fmax,-ob[i].val);
        }
        fmax=std::max(fmax,ob[i].val);
      }
      if (feasb && nob==0) fmax=0.e0;
      fM=fmax;
      fMp=fmax-psf;
      span[1]=fM;

      if (glob_prnt.iprint>=3 && glob_log.first) {
        for (i=1; i<=nob; i++) {
          if (feasb) {
            if (nob>1) {
              tempv=ob[i].grad;
              sbout2(glob_prnt.io,nparam,i,"gradf(j,",")",tempv);
            }
            if (nob==1) {
              tempv=ob[1].grad;
              sbout1(glob_prnt.io,nparam,"gradf(j)            ",
                  dummy,tempv,2,2);
            }
            continue;
          }
          tempv=ob[i].grad;
          sbout2(glob_prnt.io,nparam,indxob[i],"gradg(j,",")",tempv);
        }
        if (ncnstr!=0) {
          for (i=1; i<=ncnst1; i++) {
            tempv=cs[indxcn[i]].grad;
            sbout2(glob_prnt.io,nparam,indxcn[i],"gradg(j,",")",tempv);
          }
          if (neqn!=0) {
            sbout1(glob_prnt.io,nparam,"grdpsf(j)           ",dummy,
                grdpsf,2,2);
            sbout1(glob_prnt.io,neqn,"P                   ",dummy,
                penp,2,2);
          }
        }
        for (i=1; i<=nparam; i++) {
          tempv=colvec(hess,i,nparam);
          sbout2(glob_prnt.io,nparam,i,"hess (j,",")",tempv);
          free_dv(tempv);
        }
      }

      /*----------------------------------------------------------*
       *              Main loop of the algorithm                  *
       *----------------------------------------------------------*/

      nstop=1;
      for (;;) {
        out(miter,nparam,nob,nobL,nfsip,nineqn,nn,nineqn,ncnst1,
            ncsipl,ncsipn,mesh_pts,param->x,cs,ob,fM,fmax,steps,
            sktnom,d0nm,feasb);
        if (nstop==0) {
          if (!feasb) {
            dealloc1(nparam,nrowa,a,hess,hess1,di,d,gm,
                grdpsf,penp,bl,bu,clamda,cvec,psmu,span,backup,
                iact,iskip,istore);
            return;
          }
          for (i=1; i<=ncnst1; i++) cs[i].val=backup[i];
          for (i=1; i<=nob; i++) ob[i].val=backup[i+ncnst1];
          for (i=1; i<=neqn; i++)
            cs[glob_info.nnineq+i].mult = signeq[i]*psmu[i];
          dealloc1(nparam,nrowa,a,hess,hess1,di,d,gm,
              grdpsf,penp,bl,bu,clamda,cvec,psmu,span,backup,
              iact,iskip,istore);
          return;
        }
        if (!feasb && glob_prnt.iprint==0) glob_prnt.iter++;
        /*   Update the SIP constraint set Omega_k  */
        if ((ncsipl+ncsipn)!=0 || nfsip)
          update_omega(nparam,ncsipl,ncsipn,mesh_pts,nineqn,nob,nobL,
              nfsip,steps,fmax,cs,ob,param->x,viol,param->cd,feasb);
        /*   Compute search direction               */
        dir(nparam,nob,nobL,nfsip,nineqn,neq,neqn,nn,ncsipl,ncsipn,
            ncnst1,feasb,&steps,epskt,epseqn,&sktnom,&scvneq,Ck,&d0nm,
            &grdftd,indxob,indxcn,iact,&iskp,iskip,istore,param,di,d,
            cs,ob,&fM,&fMp,&fmax,&psf,grdpsf,penp,a,bl,bu,clamda,cvec,
            hess,hess1,backup,signeq,viol);
        if (nstop==0 && !glob_log.get_ne_mult) continue;
        glob_log.first=false;
        if (!glob_log.update && !glob_log.d0_is0) {
          /*   Determine step length                                */
          step1(nparam,nob,nobL,nfsip,nineqn,neq,neqn,nn,ncsipl,ncsipn,
              ncnst1,&ncg,&ncf,indxob,indxcn,iact,&iskp,iskip,istore,
              feasb,grdftd,ob,&fM,&fMp,&fmax,&psf,penp,&steps,&scvneq,
              bu,param->x,di,d,cs,backup,signeq,viol,param->cd);
          if (nstop==0) continue;
        }
        /*   Update the Hessian                                      */
        hessian(nparam,nob,nfsip,nobL,nineqn,neq,neqn,nn,ncsipn,ncnst1,
            nfs,&nstart,feasb,bu,param,ob,fmax,&fM,&fMp,&psf,grdpsf,
            penp,cs,gm,indxob,indxcn,bl,clamda,di,hess,d,steps,&nrst,
            signeq,span,hess1,cvec,psmu,viol);
        if (nstop==0 || glob_info.mode==0) continue;
        if (d0nm>dbar) Ck=std::max(0.5e0*Ck,Cbar);
        if (d0nm<=dbar && glob_log.dlfeas) Ck=Ck;
        if (d0nm<=dbar && !glob_log.dlfeas &&
            !glob_log.rhol_is1) Ck=10.e0*Ck;
      }
    }

  /*******************************************************************/
  /*    Free up memory used by CFSQP1                                */
  /*******************************************************************/

  void
    OFSQP::dealloc1(int nparam,int nrowa,double **a,double **hess,double **hess1,
        double *di,double *d,double *gm,double *grdpsf,double *penp,
        double *bl,double *bu,double *clamda,double *cvec,double *psmu,
        double *span,double *backup,int *iact,int *iskip,int *istore)
    {
      free_dm(a,nrowa);
      free_dm(hess,nparam);
      free_dm(hess1,nparam+1);
      free_dv(di);
      free_dv(d);
      free_dv(gm);
      free_dv(grdpsf);
      free_dv(penp);
      free_dv(bl);
      free_dv(bu);
      free_dv(clamda);
      free_dv(cvec);
      free_dv(psmu);
      free_dv(span);
      free_dv(backup);
      free_iv(iact);
      free_iv(iskip);
      free_iv(istore);
    }


  /************************************************************/
  /*   CFSQP - Check the input data                           */
  /************************************************************/

  void
    OFSQP::check(int nparam,int nf,int nfsip,bool *Linfty,int nineq,
        int nnl,int neq,int neqn,int ncsipl,int ncsipn,int mode,
        int *modem,double eps,double bigbnd,struct _parameter *param)
    {
      int i;
      double bli,bui;

      if (nparam<=0)
        error("nparam should be positive!                ",
            &glob_prnt.info);
      if (nf<0)
        error("nf should not be negative!                ",
            &glob_prnt.info);
      if (nineq<0)
        error("nineq should not be negative!             ",
            &glob_prnt.info);
      if (nineq>=0 && nnl>=0 && nineq<nnl)
        error("nineq should be no smaller than nnl!     ",
            &glob_prnt.info);
      if (neqn<0)
        error("neqn should not be negative!              ",
            &glob_prnt.info);
      if (neq<neqn)
        error("neq should not be smaller than neqn      ",
            &glob_prnt.info);
      if (nf<nfsip)
        error("nf should not be smaller than nfsip      ",
            &glob_prnt.info);
      if (nineq<ncsipn+ncsipl)
        error("ncsrl+ncsrn should not be larger than nineq",
            &glob_prnt.info);
      if (nparam<=neq-neqn)
        error("Must have nparam > number of linear equalities",
            &glob_prnt.info);
      if (glob_prnt.iprint<0 || glob_prnt.iprint>3)
        error("iprint mod 10 should be 0,1,2 or 3!       ",
            &glob_prnt.info);
      if (eps<=glob_grd.epsmac) {
        error("eps should be bigger than epsmac!         ",
            &glob_prnt.info);
        fprintf(glob_prnt.io,
            "epsmac = %22.14e which is machine dependent.\n",
            glob_grd.epsmac);
      }
      if (!(mode==100 || mode==101 || mode==110 || mode==111
            || mode==200 || mode==201 || mode==210 || mode==211))
        error("mode is not properly specified!           ",
            &glob_prnt.info);
      if (glob_prnt.info!=0) {
        fprintf(glob_prnt.io,
            "Error: Input parameters are not consistent.\n");
        return;
      }
      for (i=1; i<=nparam; i++) {
        bli=param->bl[i];
        bui=param->bu[i];
        if (bli>bui) {
          fprintf(glob_prnt.io,
              "lower bounds should be smaller than upper bounds\n");
          glob_prnt.info=7;
        }
        if (glob_prnt.info!=0) return;
        if (bli<(-bigbnd)) param->bl[i]=-bigbnd;
        if (bui>bigbnd) param->bu[i]=bigbnd;
      }
      if (mode >= 200) {
        i=mode-200;
        glob_info.modec=2;
      } else {
        i=mode-100;
        glob_info.modec=1;
      }
      if (i<10) *modem=0;
      else {
        *modem=1;
        i-=10;
      }
      if (!i) *Linfty=false;
      else *Linfty=true;
    }

  /****************************************************************/
  /*    CFSQP : Generate a feasible point satisfying simple       */
  /*            bounds and linear constraints.                    */
  /****************************************************************/

  void
    OFSQP::initpt(int nparam,int nnl,int neq,int neqn,int nclin,int nctotl,
        int nrowa,struct _parameter *param,struct _constraint *cs)
    {
      int i,j,infoql,mnn,temp1,iout,zero;
      double x0i,*atemp,*htemp;
      double *x,*bl,*bu,*cvec,*clamda,*bj;
      double **a,**hess;

      hess=make_dm(nparam,nparam);
      a=make_dm(nrowa,nparam);
      x=make_dv(nparam);
      bl=make_dv(nctotl);
      bu=make_dv(nctotl);
      cvec=make_dv(nparam);
      clamda=make_dv(nctotl+nparam+1);
      bj=make_dv(nclin);

      glob_prnt.info=1;
      for (i=1; i<=nclin; i++) {
        glob_grd.valnom=cs[i].val;
        j=i+nnl;
        if (j<=glob_info.nnineq)
          pb->gradcn(nparam,j,(param->x)+1,cs[i].grad+1,param->cd);
        else pb->gradcn(nparam,j+neqn,(param->x)+1,cs[i].grad+1,param->cd);
      }
      for (i=1; i<=nparam; i++) {
        x0i=param->x[i];
        bl[i]=param->bl[i]-x0i;
        bu[i]=param->bu[i]-x0i;
        cvec[i]=0.e0;
      }
      for (i=nclin; i>=1; i--)
        bj[nclin-i+1]=-cs[i].val;
      for (i=nclin; i>=1; i--)
        for (j=1; j<=nparam; j++)
          a[nclin-i+1][j]=-cs[i].grad[j];
      diagnl(nparam,1.e0,hess);
      nullvc(nparam,x);

      iout=6;
      zero=0;
      mnn=nrowa+2*nparam;
      iw[1]=1;
      temp1=neq-neqn;
      htemp=convert(hess,nparam,nparam);
      atemp=convert(a,nrowa,nparam);

      ObjQLD::ql0001_(&nclin,&temp1,&nrowa,&nparam,&nparam,&mnn,(htemp+1),
          (cvec+1),(atemp+1),(bj+1),(bl+1),(bu+1),(x+1),(clamda+1),
          &iout,&infoql,&zero,(w+1),&lenw,(iw+1),&leniw,
          &glob_grd.epsmac);

      free_dv(htemp);
      free_dv(atemp);
      if (infoql==0) {
        for (i=1; i<=nparam; i++)
          param->x[i]=param->x[i]+x[i];
        pb->invalidateX();
        for (i=1; i<=nclin; i++) {
          j=i+nnl;
          if (j<=glob_info.nnineq) pb->constr(nparam,j,(param->x)+1,
              &(cs[i].val),param->cd);
          else pb->constr(nparam,j+neqn,(param->x)+1,&(cs[i].val),param->cd);
        }
        glob_prnt.info=0;
      }
      if (glob_prnt.info==1 && glob_prnt.iprint!=0) {
        fprintf(glob_prnt.io,
            "\n Error: No feasible point is found for the");
        fprintf(glob_prnt.io," linear constraints.\n");
      }
      free_dm(a,nrowa);
      free_dm(hess,nparam);
      free_dv(x);
      free_dv(bl);
      free_dv(bu);
      free_dv(cvec);
      free_dv(clamda);
      free_dv(bj);
      return;
    }

  /****************************************************************/
  /*   CFSQP : Update the SIP "active" objective and constraint   */
  /*           sets Omega_k and Xi_k.                             */
  /****************************************************************/

  void
    OFSQP::update_omega(int nparam,int ncsipl,int ncsipn,int *mesh_pts,
        int nineqn,int nob,int nobL,int nfsip,double steps,
        double fmax,struct _constraint *cs,struct _objective *ob,
        double *x,struct _violation *viol,void *cd,int feasb)
    {
      int i,j,i_max,index,offset,nineq;
      bool display;
      double epsilon,g_max,fprev,fnow,fnext,fmult;

      epsilon=1.e0;
      glob_info.tot_actf_sip=glob_info.tot_actg_sip=0;
      nineq=glob_info.nnineq;
      if (glob_prnt.iter%glob_prnt.iter_mod) display=false;
      else display=true;
      /* Clear previous constraint sets                   */
      for (i=1; i<=ncsipl; i++)
        cs[nineq-ncsipl+i].act_sip=false;
      for (i=1; i<=ncsipn; i++)
        cs[nineqn-ncsipn+i].act_sip=false;
      /* Clear previous objective sets                    */
      for (i=nob-nfsip+1; i<=nob; i++)
        ob[i].act_sip=false;

      /*--------------------------------------------------*/
      /* Update Constraint Sets Omega_k                   */
      /*--------------------------------------------------*/

      if (ncsipn != 0) {
        offset=nineqn-ncsipn;
        for (i=1; i<=glob_info.ncsipn; i++) {
          for (j=1; j<=mesh_pts[glob_info.nfsip+i]; j++) {
            offset++;
            if (j==1) {
              if (cs[offset].val >= -epsilon &&
                  cs[offset].val>=cs[offset+1].val) {
                cs[offset].act_sip=true;
                glob_info.tot_actg_sip++;
                if (cs[offset].mult==0.e0 && !glob_log.first) {
                  glob_grd.valnom=cs[offset].val;
                  pb->gradcn(nparam,offset,x+1,cs[offset].grad+1,cd);
                }
                continue;
              }
            } else if (j==mesh_pts[glob_info.nfsip+i]) {
              if (cs[offset].val >= -epsilon &&
                  cs[offset].val>cs[offset-1].val) {
                cs[offset].act_sip=true;
                glob_info.tot_actg_sip++;
                if (cs[offset].mult==0.e0 && !glob_log.first) {
                  glob_grd.valnom=cs[offset].val;
                  pb->gradcn(nparam,offset,x+1,cs[offset].grad+1,cd);
                }
                continue;
              }
            } else {
              if (cs[offset].val >= -epsilon && cs[offset].val >
                  cs[offset-1].val && cs[offset].val >=
                  cs[offset+1].val) {
                cs[offset].act_sip=true;
                glob_info.tot_actg_sip++;
                if (cs[offset].mult==0.e0 && !glob_log.first) {
                  glob_grd.valnom=cs[offset].val;
                  pb->gradcn(nparam,offset,x+1,cs[offset].grad+1,cd);
                }
                continue;
              }
            }
            if (cs[offset].val >= -glob_grd.epsmac) {
              cs[offset].act_sip=true;
              glob_info.tot_actg_sip++;
              if (cs[offset].mult==0.e0 && !glob_log.first) {
                glob_grd.valnom=cs[offset].val;
                pb->gradcn(nparam,offset,x+1,cs[offset].grad+1,cd);
              }
              continue;
            }
            if (cs[offset].mult>0.e0) {
              cs[offset].act_sip=true;
              glob_info.tot_actg_sip++;
            }
            /* Add if binding for d1  */
            if (cs[offset].d1bind) {
              cs[offset].act_sip=true;
              glob_info.tot_actg_sip++;
              if (cs[offset].mult==0.e0 && !glob_log.first) {
                glob_grd.valnom=cs[offset].val;
                pb->gradcn(nparam,offset,x+1,cs[offset].grad+1,cd);
              }
            }

          }
        }
      }
      if (ncsipl != 0) {
        /* Don't need to get gradients */
        offset=nineq-ncsipl;
        for (i=1; i<=glob_info.ncsipl; i++) {
          if (feasb) index=glob_info.nfsip+glob_info.ncsipn+i;
          else index=glob_info.ncsipn+i;
          for (j=1; j<=mesh_pts[index]; j++) {
            offset++;
            if (j==1) {
              if (cs[offset].val >= -epsilon &&
                  cs[offset].val>=cs[offset+1].val) {
                cs[offset].act_sip=true;
                glob_info.tot_actg_sip++;
                continue;
              }
            } else
              if (j==mesh_pts[index]) {
                if (cs[offset].val >= -epsilon &&
                    cs[offset].val>cs[offset-1].val) {
                  cs[offset].act_sip=true;
                  glob_info.tot_actg_sip++;
                  continue;
                }
              } else {
                if (cs[offset].val >= -epsilon && cs[offset].val >
                    cs[offset-1].val && cs[offset].val >=
                    cs[offset+1].val) {
                  cs[offset].act_sip=true;
                  glob_info.tot_actg_sip++;
                  continue;
                }
              }
            if (cs[offset].val >= -glob_grd.epsmac ||
                cs[offset].mult>0.e0 || cs[offset].d1bind) {
              cs[offset].act_sip=true;
              glob_info.tot_actg_sip++;
            }
          }
        }
      }
      /* Include some extra points during 1st iteration        */
      /* (gradients are already evaluated for first iteration) */
      /* Current heuristics: maximizers and end-points.        */
      if (glob_log.first) {
        if (feasb) {
          offset=nineqn-ncsipn;
          for (i=1; i<=glob_info.ncsipn; i++) {
            i_max= ++offset;
            g_max=cs[i_max].val;
            if (!cs[i_max].act_sip) { /* add first point       */
              cs[i_max].act_sip=true;
              glob_info.tot_actg_sip++;
            }
            for (j=2;j<=mesh_pts[glob_info.nfsip+i];j++) {
              offset++;
              if (cs[offset].val>g_max) {
                i_max=offset;
                g_max=cs[i_max].val;
              }
            }
            if (!cs[i_max].act_sip) {
              cs[i_max].act_sip=true;
              glob_info.tot_actg_sip++;
            }
            if (!cs[offset].act_sip) { /* add last point          */
              cs[offset].act_sip=true;
              glob_info.tot_actg_sip++;
            }
          }
        }
        offset=nineq-ncsipl;
        for (i=1; i<=glob_info.ncsipl; i++) {
          i_max= ++offset;
          g_max=cs[i_max].val;
          if (!cs[i_max].act_sip) { /* add first point       */
            cs[i_max].act_sip=true;
            glob_info.tot_actg_sip++;
          }
          if (feasb) index=glob_info.nfsip+glob_info.ncsipn+i;
          else index=glob_info.ncsipn+i;
          for (j=2;j<=mesh_pts[index]; j++) {
            offset++;
            if (cs[offset].val>g_max) {
              i_max=offset;
              g_max=cs[i_max].val;
            }
          }
          if (!cs[i_max].act_sip) {
            cs[i_max].act_sip=true;
            glob_info.tot_actg_sip++;
          }
          if (!cs[offset].act_sip) { /* add last point          */
            cs[offset].act_sip=true;
            glob_info.tot_actg_sip++;
          }
        }
      }

      /* If necessary, append xi_bar                              */
      if (steps<1.e0 && viol->type==CONSTR) {
        i=viol->index;
        if (!cs[i].act_sip) {
          cs[i].act_sip=true;
          glob_info.tot_actg_sip++;
        }
      }
      if (glob_prnt.iprint>=2 && display)
        fprintf(glob_prnt.io," |Xi_k| for g       %26d\n",
            glob_info.tot_actg_sip);

      for (i=1; i<=ncsipl; i++)
        cs[nineq-ncsipl+i].d1bind=false;
      for (i=1; i<=ncsipn; i++)
        cs[nineqn-ncsipn+i].d1bind=false;

      /*---------------------------------------------------------*/
      /* Update Objective Set Omega_k                                    */
      /*---------------------------------------------------------*/

      if (nfsip) {
        offset=nob-nfsip;
        if (feasb) index=glob_info.nfsip;
        else index=glob_info.ncsipn;
        for (i=1; i<=index; i++) {
          for (j=1; j<=mesh_pts[i]; j++) {
            offset++;
            if (nobL>nob) {
              fnow=fabs(ob[offset].val);
              fmult=std::max(fabs(ob[offset].mult),
                  fabs(ob[offset].mult_L));
            } else {
              fnow=ob[offset].val;
              fmult=ob[offset].mult;
            }
            if (j==1) {
              if (nobL>nob) fnext=fabs(ob[offset+1].val);
              else fnext=ob[offset+1].val;
              if ((fnow>=fmax-epsilon)&& fnow>=fnext) {
                ob[offset].act_sip=true;
                glob_info.tot_actf_sip++;
                if (fmult==0.e0 && !glob_log.first) {
                  glob_grd.valnom=ob[offset].val;
                  if (feasb) pb->gradob(nparam,offset,x+1,ob[offset].grad+1,cd);
                  else pb->gradcn(nparam,offset,x+1,ob[offset].grad+1,cd);
                }
                continue;
              }
            } else if (j==mesh_pts[i]) {
              if (nobL>nob) fprev=fabs(ob[offset-1].val);
              else fprev=ob[offset-1].val;
              if ((fnow>=fmax-epsilon)&& fnow>fprev) {
                ob[offset].act_sip=true;
                glob_info.tot_actf_sip++;
                if (fmult==0.e0 && !glob_log.first) {
                  glob_grd.valnom=ob[offset].val;
                  if (feasb) pb->gradob(nparam,offset,x+1,ob[offset].grad+1,cd);
                  else pb->gradcn(nparam,offset,x+1,ob[offset].grad+1,cd);
                }
                continue;
              }
            } else {
              if (nobL>nob) {
                fprev=fabs(ob[offset-1].val);
                fnext=fabs(ob[offset+1].val);
              } else {
                fprev=ob[offset-1].val;
                fnext=ob[offset+1].val;
              }
              if ((fnow>=fmax-epsilon)&& fnow>fprev &&
                  fnow>=fnext) {
                ob[offset].act_sip=true;
                glob_info.tot_actf_sip++;
                if (fmult==0.e0 && !glob_log.first) {
                  glob_grd.valnom=ob[offset].val;
                  if (feasb) pb->gradob(nparam,offset,x+1,
                      ob[offset].grad+1,cd);
                  else pb->gradcn(nparam,offset,x+1,ob[offset].grad+1,cd);
                }
                continue;
              }
            }
            if (fnow>= fmax-glob_grd.epsmac && !ob[offset].act_sip) {
              ob[offset].act_sip=true;
              glob_info.tot_actf_sip++;
              if (fmult==0.e0 && !glob_log.first) {
                glob_grd.valnom=ob[offset].val;
                if (feasb) pb->gradob(nparam,offset,x+1,
                    ob[offset].grad+1,cd);
                else pb->gradcn(nparam,offset,x+1,ob[offset].grad+1,cd);
              }
              continue;
            }
            if (fmult!=0.e0 && !ob[offset].act_sip) {
              ob[offset].act_sip=true;
              glob_info.tot_actf_sip++;
              continue;
            }
          }
        }
        /* Addition of objectives for first iteration.          */
        /* Current heuristics: maximizers and end-points        */
        if (glob_log.first) {
          offset=nob-nfsip;
          if (feasb) index=glob_info.nfsip;
          else index=glob_info.ncsipn;
          for (i=1; i<=index; i++) {
            i_max= ++offset;
            if (nobL==nob) g_max=ob[i_max].val;
            else g_max=fabs(ob[i_max].val);
            if (!ob[i_max].act_sip) { /* add first point       */
              ob[i_max].act_sip=true;
              glob_info.tot_actf_sip++;
            }
            for (j=2;j<=mesh_pts[i];j++) {
              offset++;
              if (nobL==nob) fnow=ob[offset].val;
              else fnow=fabs(ob[offset].val);
              if (fnow>g_max) {
                i_max=offset;
                g_max=fnow;
              }
            }
            if (!ob[i_max].act_sip) {
              ob[i_max].act_sip=true;
              glob_info.tot_actf_sip++;
            }
            if (!ob[offset].act_sip) { /* add last point          */
              ob[offset].act_sip=true;
              glob_info.tot_actf_sip++;
            }
          }
        }

        /* If necessary, append omega_bar                          */
        if (steps<1.e0 && viol->type==OBJECT) {
          i=viol->index;
          if (!ob[i].act_sip) {
            ob[i].act_sip=true;
            glob_info.tot_actf_sip++;
          }
        }
        if (glob_prnt.iprint>=2 && display)
          fprintf(glob_prnt.io," |Omega_k| for f    %26d\n",
              glob_info.tot_actf_sip);
      }
      viol->type=NONE;
      viol->index=0;
      return;
    }


  /*******************************************************************/
  /*   CFSQP : Computation of the search direction                   */
  /*******************************************************************/


  void
    OFSQP::dir(int nparam,int nob,int nobL,int nfsip,int nineqn,int neq,int neqn,
        int nn,int ncsipl,int ncsipn,int ncnstr,
        int feasb,double *steps,double epskt,double epseqn,
        double *sktnom,double *scvneq,double Ck,double *d0nm,
        double *grdftd,int *indxob,int *indxcn,int *iact,int *iskp,
        int *iskip,int *istore,struct _parameter *param,double *di,
        double *d,struct _constraint *cs,struct _objective *ob,
        double *fM,double *fMp,double *fmax,double *psf,double *grdpsf,
        double *penp,double **a,double *bl,double *bu,double *clamda,
        double *cvec,double **hess,double **hess1,
        double *backup,double *signeq,struct _violation *viol)
    {
      int  i,j,k,kk,ncg,ncf,nqprm0,nclin0,nctot0,infoqp,nqprm1,ncl,
      nclin1,ncc,nff,nrowa0,nrowa1,ninq,nobb,nobbL,
      nncn,ltem1,ltem2;
      bool display,need_d1;
      double fmxl,vv,dx,dmx,dnm1,dnm,v0,v1,vk,temp1,temp2,theta,
             rhol,rhog,rho,grdfd0,grdfd1,dummy,grdgd0,grdgd1,thrshd,
             sign,*adummy,dnmtil,*tempv;

      theta = 0;  //AE: initialization to avoid VS crash in debug for mode B=0

      ncg=ncf=*iskp=0;
      ncl=glob_info.nnineq-nineqn;
      glob_log.local=glob_log.update=false;
      glob_log.rhol_is1=false;
      thrshd=tolfea;
      adummy=make_dv(1);
      adummy[1]=0.e0;
      dummy=0.e0;
      temp1=temp2=0.e0;
      if (glob_prnt.iter%glob_prnt.iter_mod) display=false;
      else display=true;
      need_d1=true;

      if (nobL<=1) {
        nqprm0=nparam;
        nclin0=ncnstr;
      } else {
        nqprm0=nparam+1;
        nclin0=ncnstr+nobL;
      }
      nctot0=nqprm0+nclin0;
      vv=0.e0;
      nrowa0=std::max(nclin0,1);
      for (i=1; i<=ncnstr; i++) {
        if (feasb) {
          if (i>nineqn && i<=glob_info.nnineq)
            iskip[glob_info.nnineq+2-i]=i;
          iw[i]=i;
        } else {
          if (i<=ncl) iskip[ncl+2-i]=nineqn+i;
          if (i<=ncl) iw[i]=nineqn+i;
          if (i>ncl) iw[i]=nineqn+neqn+i;
        }
      }
      for (i=1; i<=nob; i++)
        iw[ncnstr+i]=i;
      nullvc(nparam, cvec);
      glob_log.d0_is0=false;
      dqp(nparam,nqprm0,nob,nobL,nfsip,nineqn,neq,neqn,nn,ncsipl,ncsipn,
          ncnstr,nctot0,nrowa0,nineqn,&infoqp,param,di,feasb,ob,
          *fmax,grdpsf,cs,a,cvec,bl,bu,clamda,hess,hess1,di,vv,0);
      if (infoqp!=0) {
        glob_prnt.info=5;
        if (!feasb) glob_prnt.info=2;
        nstop=0;
        free_dv(adummy);
        return;
      }
      /*-------------------------------------------------------------*/
      /*    Reorder indexes of constraints & objectives              */
      /*-------------------------------------------------------------*/
      if (nn>1) {
        j=1;
        k=nn;
        for (i=nn; i>=1; i--) {
          if (fuscmp(cs[indxcn[i]].mult,thrshd)) {
            iact[j]=indxcn[i];
            j++;
          } else {
            iact[k]=indxcn[i];
            k--;
          }
        }
      }
      if (nobL>1) {
        j=nn+1;
        k=nn+nob;
        for (i=nob; i>=1; i--) {
          kk=nqprm0+ncnstr;
          ltem1=fuscmp(ob[i].mult,thrshd);
          ltem2=(nobL!=nob) && (fuscmp(ob[i].mult_L,thrshd));
          if (ltem1 || ltem2) {
            iact[j]=i;
            j++;
          } else {
            iact[k]=i;
            k--;
          }
        }
      }
      if (nob>0) vv=ob[iact[nn+1]].val;
      *d0nm=sqrt(scaprd(nparam,di,di));
      if (glob_log.first && nclin0==0) {
        dx=sqrt(scaprd(nparam,param->x,param->x));
        dmx=std::max(dx,1.e0);
        if (*d0nm>dmx) {
          for (i=1; i<=nparam; i++)
            di[i]=di[i]*dmx/(*d0nm);
          *d0nm=dmx;
        }
      }
      matrvc(nparam,nparam,hess,di,w);
      if (nn==0) *grdftd = -scaprd(nparam,w,di);
      *sktnom=sqrt(scaprd(nparam,w,w));
      if (((*d0nm<=epskt)||((gLgeps>0.e0)&&(*sktnom<=gLgeps)))&&
          (neqn==0 || *scvneq<=epseqn)) {
        /* We are finished! */
        nstop=0;
        if (feasb && glob_log.first && neqn!=0) {
          /*  Finished, but still need to estimate nonlinear equality
              constraint multipliers   */
          glob_log.get_ne_mult = true;
          glob_log.d0_is0 = true;
        }
        if (!feasb) glob_prnt.info=2;
        free_dv(adummy);
        if (glob_prnt.iprint<3 || !display) return;
        if (nobL<=1) nff=1;
        if (nobL>1) nff=2;
        sbout1(glob_prnt.io,nparam,"multipliers for x   ",dummy,
            param->mult,2,2);
        if (ncnstr!=0) {
          fprintf(glob_prnt.io,"\t\t\t %s\t %22.14e\n",
              "           for g    ",cs[1].mult);
          for (j=2; j<=ncnstr; j++)
            fprintf(glob_prnt.io," \t\t\t\t\t\t %22.14e\n",cs[j].mult);
        }
        if (nobL>1) {
          fprintf(glob_prnt.io,"\t\t\t %s\t %22.14e\n",
              "           for f    ",ob[1].mult);
          for (j=2; j<=nob; j++)
            fprintf(glob_prnt.io," \t\t\t\t\t\t %22.14e\n",ob[j].mult);
        }
        return;
      }
      if (glob_prnt.iprint>=3 && display) {
        sbout1(glob_prnt.io,nparam,"d0                  ",dummy,di,2,2);
        sbout1(glob_prnt.io,0,"d0norm              ",*d0nm,adummy,1,2);
        sbout1(glob_prnt.io,0,"ktnorm              ",*sktnom,adummy,1,2);
      }
      if (neqn!=0 && *d0nm<=std::min(0.5e0*epskt,(0.1e-1)*glob_grd.rteps)
          && *scvneq>epseqn) {
        /* d0 is "zero", but equality constraints not satisfied  */
        glob_log.d0_is0 = true;
        return;
      }
      /*--------------------------------------------------------------*/
      /*     Single objective without nonlinear constraints requires  */
      /*     no d1 and dtilde; multi-objectives without nonlinear     */
      /*     constraints requires no d1.                              */
      /*--------------------------------------------------------------*/
      if (nn!=0) *grdftd=slope(nob,nobL,neqn,nparam,feasb,ob,grdpsf,
          di,d,*fmax,dummy,0,adummy,0);

      if (nn==0 && nobL<=1) {
        for (i=1; i<=nparam; i++) d[i]=0.e0;
        dnmtil=0.e0;
        free_dv(adummy);
        return;
      }
      if (nn==0) {
        dnm=*d0nm;
        rho=0.e0;
        rhog=0.e0;
        goto L310;
      }
      /*-------------------------------------------------------------*/
      /*     compute modified first order direction d1               */
      /*-------------------------------------------------------------*/

      /* First check that it is necessary */
      if (glob_info.mode==1) {
        vk=std::min(Ck*(*d0nm)*(*d0nm),*d0nm);
        need_d1=false;
        for (i=1; i<=nn; i++) {
          tempv=cs[indxcn[i]].grad;
          grdgd0=scaprd(nparam,tempv,di);
          temp1=vk+cs[indxcn[i]].val+grdgd0;
          if (temp1>0.e0) {
            need_d1=true;
            break;
          }
        }
      }
      if (need_d1) {
        nqprm1=nparam+1;
        if (glob_info.mode==0) nclin1=ncnstr+std::max(nobL,1);
        if (glob_info.mode==1) nclin1=ncnstr;
        nrowa1=std::max(nclin1,1);
        ninq=glob_info.nnineq;
        di1(nparam,nqprm1,nob,nobL,nfsip,nineqn,neq,neqn,ncnstr,
            ncsipl,ncsipn,nrowa1,&infoqp,glob_info.mode,
            param,di,ob,*fmax,grdpsf,cs,cvec,bl,bu,clamda,
            hess1,d,*steps);
        if (infoqp!=0) {
          glob_prnt.info=6;
          if (!feasb) glob_prnt.info=2;
          nstop=0;
          free_dv(adummy);
          return;
        }
        dnm1=sqrt(scaprd(nparam,d,d));
        if (glob_prnt.iprint>=3 && display) {
          sbout1(glob_prnt.io,nparam,"d1                  ",dummy,d,2,2);
          sbout1(glob_prnt.io,0,"d1norm              ",dnm1,adummy,1,2);
        }
      } else {
        dnm1=0.e0;
        for (i=1; i<=nparam; i++) d[i]=0.e0;
      }
      if (glob_info.mode!=1) {
        v0=pow(*d0nm,2.1);
        v1=std::max(0.5e0,pow(dnm1,2.5));
        rho=v0/(v0+v1);
        rhog=rho;
      } else {
        rhol=0.e0;
        if (need_d1) {
          for (i=1; i<=nn; i++) {
            tempv=cs[indxcn[i]].grad;
            grdgd0=scaprd(nparam,tempv,di);
            grdgd1=scaprd(nparam,tempv,d);
            temp1=vk+cs[indxcn[i]].val+grdgd0;
            temp2=grdgd1-grdgd0;
            if (temp1<=0.e0) continue;
            if (fabs(temp2)<glob_grd.epsmac) {
              rhol=1.e0;
              glob_log.rhol_is1=true;
              break;
            }
            rhol=std::max(rhol,-temp1/temp2);
            if (temp2<0.e0 && rhol<1.e0) continue;
            rhol=1.e0;
            glob_log.rhol_is1=true;
            break;
          }
        }
        theta=0.2e0;
        if (rhol==0.e0) {
          rhog=rho=0.e0;
          dnm=*d0nm;
          goto L310;
        }
        if (nobL>1) {
          rhog=slope(nob,nobL,neqn,nparam,feasb,ob,grdpsf,
              di,d,*fmax,theta,glob_info.mode,adummy,0);
          rhog=std::min(rhol,rhog);
        } else {
          grdfd0= *grdftd;
          if (nob==1) grdfd1=scaprd(nparam,ob[1].grad,d);
          else grdfd1=0.e0;
          grdfd1=grdfd1-scaprd(nparam,grdpsf,d);
          temp1=grdfd1-grdfd0;
          temp2=(theta-1.e0)*grdfd0/temp1;
          if(temp1<=0.e0) rhog=rhol;
          else rhog=std::min(rhol,temp2);
        }
        rho=rhog;
        if (*steps==1.e0 && rhol<0.5e0) rho=rhol;
      }
      for (i=1; i<=nparam; i++) {
        if (rho!=rhog) cvec[i]=di[i];
        di[i]=(1.e0-rho)*di[i]+rho*d[i];
      }
      dnm=sqrt(scaprd(nparam,di,di));
      if (!(glob_prnt.iprint<3 || glob_info.mode==1 || nn==0)&&display) {
        sbout1(glob_prnt.io,0,"rho                 ",rho,adummy,1,2);
        sbout1(glob_prnt.io,nparam,"d                   ",dummy,di,2,2);
        sbout1(glob_prnt.io,0,"dnorm               ",dnm,adummy,1,2);
      }
L310:
      for (i=1; i<=nob; i++) bl[i]=ob[i].val;
      if (rho!=1.e0) {
        if (!(glob_prnt.iprint!=3 || glob_info.mode==0 || nn==0)
            && display) {
          sbout1(glob_prnt.io,0,"Ck                  ",Ck,adummy,1,2);
          sbout1(glob_prnt.io,0,"rhol                ",rho,adummy,1,2);
          sbout1(glob_prnt.io,nparam,"dl                  ",dummy,di,2,2);
          sbout1(glob_prnt.io,0,"dlnorm              ",dnm,adummy,1,2);
        }
        if (glob_info.mode!=0) {
          glob_log.local=true;
          step1(nparam,nob,nobL,nfsip,nineqn,neq,neqn,nn,ncsipl,ncsipn,
              ncnstr,&ncg,&ncf,indxob,indxcn,iact,iskp,iskip,istore,
              feasb,*grdftd,ob,fM,fMp,fmax,psf,penp,steps,scvneq,bu,
              param->x,di,d,cs,backup,signeq,viol,param->cd);
          if (!glob_log.update) nstop=1;
          else {
            free_dv(adummy);
            return;
          }
          glob_log.local=false;
          if (rho!=rhog && nn!=0)
            for (i=1; i<=nparam; i++)
              di[i]=(1-rhog)*cvec[i]+rhog*d[i];
          dnm=sqrt(scaprd(nparam,di,di));
        }
      }
      if (!(glob_prnt.iprint<3 || glob_info.mode==0 || nn==0) &&
          display) {
        sbout1(glob_prnt.io,0,"rhog                ",rhog,adummy,1,2);
        sbout1(glob_prnt.io,nparam,"dg                  ",dummy,di,2,2);
        sbout1(glob_prnt.io,0,"dgnorm              ",dnm,adummy,1,2);
      }
      if (rho !=0.e0) *grdftd=slope(nob,nobL,neqn,nparam,feasb,ob,
          grdpsf,di,d,*fmax,theta,0,bl,1);
      if (glob_info.mode!=1 || rho!=rhog)
        for (i=1; i<=nparam; i++)
          bu[i]=param->x[i]+di[i];
      pb->invalidateX();
      if (rho!=rhog) ncg=0;
      ncc=ncg+1;
      fmxl=-bgbnd;
      ninq=nncn=ncg;
      j=0;
      /*--------------------------------------------------------------*/
      /*   iskip[1]-iskip[iskp] store the indexes of linear inequality*/
      /*   constraints that are not to be used to compute d~          */
      /*   iskip[nnineq-nineqn+1]-iskip[nnineq-ncn+1-iskp] store      */
      /*   those that are to be used to compute d~                    */
      /*--------------------------------------------------------------*/
      for (i=ncc; i<=ncnstr; i++) {
        if (i<=nn) kk=iact[i];
        else kk=indxcn[i];
        if (kk>nineqn && kk<=glob_info.nnineq) {
          iskip[ncl+1-j]=kk;
          j++;
        }
        if (kk<=glob_info.nnineq) {
          tempv=cs[kk].grad;
          temp1=dnm*sqrt(scaprd(nparam,tempv,tempv));
          temp2=cs[kk].mult;
        }
        if (temp2!=0.e0 || cs[kk].val >= (-0.2e0*temp1) ||
            kk>glob_info.nnineq) {
          ninq++;
          iw[ninq]=kk;
          if (feasb && kk<=nineqn) istore[kk]=1;
          pb->constr(nparam,kk,bu+1,&(cs[kk].val),param->cd);
          if (!feasb || (feasb && (kk>glob_info.nnineq+neqn))) continue;
          if (kk<=nineqn) nncn=ninq;
          fmxl=std::max(fmxl,cs[kk].val);
          if (feasb &&(kk<=nineqn || (kk>glob_info.nnineq
                  && kk<=(glob_info.nnineq+neqn)))) glob_info.ncallg++;
          if (fabs(fmxl)>bgbnd) {
            for (i=1; i<=nparam; i++) d[i]=0.e0;
            dnmtil=0.e0;
            nstop=1;
            free_dv(adummy);
            return;
          }
          continue;
        }
        if (kk<=nineqn) continue;
        (*iskp)++;
        iskip[*iskp]=kk;
        j--;
      }
      if ((neqn!=0)&&(feasb))
        resign(nparam,neqn,psf,grdpsf,penp,cs,signeq,10,20);
      ninq-=neq;
      /*  if (!feasb) ninq+=neqn;   BUG???   */
      if (ncg!=0) for (i=1; i<=ncg; i++) {
        iw[i]=iact[i];
        if (iact[i]<=nineqn) istore[iact[i]]=1;
        fmxl=std::max(fmxl,cs[iact[i]].val);
        if (fabs(fmxl)>bgbnd) {
          for (i=1; i<=nparam; i++) d[i]=0.e0;
          dnmtil=0.e0;
          nstop=1;
          free_dv(adummy);
          return;
        }
      }
      if (nobL<=1) {
        iw[1+ninq+neq]=1;
        nobb=nob;
        goto L1110;
      }
      if (rho!=rhog) ncf=0;
      nff=ncf+1;
      nobb=ncf;
      sign=1.e0;
      fmxl=-bgbnd;
      if (ob[iact[nn+1]].mult<0.e0) sign=-1.e0;
      for (i=nff; i<=nob; i++) {
        kk=iact[nn+i];
        if (!feasb) kk=iact[i];
        if (feasb) k=nn+1;
        if (!feasb) k=1;
        for (j=1; j<=nparam; j++)
          w[nparam+j]=sign*ob[iact[k]].grad[j]-ob[kk].grad[j];
        temp1=fabs(ob[kk].val-sign*vv);
        temp2=dnm*sqrt(scaprd(nparam,&w[nparam],&w[nparam]));
        if (temp1!=0.e0 && temp2!=0.e0) {
          temp1=temp1/temp2;
          temp2=ob[kk].mult;
          if (temp2==0.e0 && temp1>0.2e0) continue;
        }
        nobb++;
        iw[nobb+ninq+neq]=kk;
        if (feasb) istore[nineqn+kk]=1;
        else istore[kk]=1;
        if (!feasb) {
          pb->constr(nparam,indxob[kk],bu+1,&(ob[kk].val),param->cd);
          glob_info.ncallg++;
        } else {
          pb->obj(nparam,kk,bu+1,&(ob[kk].val),param->cd);
          glob_info.ncallf++;
          if (nobL!=nob) fmxl=std::max(fmxl, -ob[kk].val);
        }
        fmxl=std::max(fmxl,ob[kk].val);
        if (fabs(fmxl)>bgbnd) {
          for (i=1; i<=nparam; i++) d[i]=0.e0;
          dnmtil=0.e0;
          nstop=1;
          free_dv(adummy);
          return;
        }
      }
      if (ncf != 0) {
        for (i=1; i<=ncf; i++) {
          iw[ninq+neq+i]=iact[i+nn];
          istore[nineqn+iact[i+nn]]=1;
          fmxl=std::max(fmxl,ob[iact[i+nn]].val);
          if (nobL != nob) fmxl=std::max(fmxl,-ob[iact[i+nn]].val);
          if (fabs(fmxl)>bgbnd) {
            for (i=1; i<=nparam; i++) d[i]=0.e0;
            dnmtil=0.e0;
            nstop=1;
            free_dv(adummy);
            return;
          }
        }
      }
L1110:
      matrvc(nparam,nparam,hess,di,cvec);
      vv=-std::min(0.01e0*dnm,pow(dnm,2.5));
      /*--------------------------------------------------------------*/
      /*    compute a correction dtilde to d=(1-rho)d0+rho*d1         */
      /*--------------------------------------------------------------*/
      if (nobL!=nob) nobbL=2*nobb;
      if (nobL==nob) nobbL=nobb;
      if (nobbL<=1) {
        nqprm0=nparam;
        nclin0=ninq+neq;
      } else {
        nqprm0=nparam+1;
        nclin0=ninq+neq+nobbL;
      }
      nctot0=nqprm0+nclin0;
      nrowa0=std::max(nclin0,1);
      i=ninq+neq;
      nstop=1;
      dqp(nparam,nqprm0,nobb,nobbL,nfsip,nncn,neq,neqn,nn,ncsipl,ncsipn,i,
          nctot0,nrowa0,nineqn,&infoqp,param,di,feasb,ob,fmxl,
          grdpsf,cs,a,cvec,bl,bu,clamda,hess,hess1,d,vv,1);
      dnmtil=sqrt(scaprd(nparam,d,d));
      if (infoqp!=0 || dnmtil>dnm) {
        for (i=1; i<=nparam; i++) d[i]=0.e0;
        dnmtil=0.e0;
        nstop=1;
        free_dv(adummy);
        return;
      }
      if (dnmtil!=0.e0)
        for (i=1; i<=nineqn+nob; i++)
          istore[i]=0;
      if (glob_prnt.iprint<3 || !display) {
        free_dv(adummy);
        return;
      }
      sbout1(glob_prnt.io,nparam,"dtilde              ",dummy,d,2,2);
      sbout1(glob_prnt.io,0,"dtnorm              ",dnmtil,adummy,1,2);
      free_dv(adummy);
      return;
    }


  /*******************************************************************/
  /*     job=0:     compute d0                                       */
  /*     job=1:     compute d~                                       */
  /*******************************************************************/

  void
    OFSQP::dqp(int nparam,int nqpram,int nob,int nobL,int nfsip,int nineqn,
        int neq,int neqn,int nn,int ncsipl,int ncsipn,int ncnstr,
        int nctotl,int nrowa,int nineqn_tot,int *infoqp,
        struct _parameter *param,double *di,int feasb,struct _objective *ob,
        double fmax,double *grdpsf,struct _constraint *cs,
        double **a,double *cvec,double *bl,double *bu,double *clamda,
        double **hess,double **hess1,double *x,
        double vv,int job)
    {
      int i,ii,j,jj,ij,k,iout,mnn,nqnp,zero,temp1,temp2,ncnstr_used,
      numf_used;
      int *iw_hold;
      double x0i,xdi,*bj,*htemp,*atemp;

      iout=6;
      bj=make_dv(nrowa);
      iw_hold=make_iv(nrowa);
      for (i=1; i<=nparam; i++) {
        x0i=param->x[i];
        if (job==1) xdi=di[i];
        if (job==0) xdi=0.e0;
        bl[i]=param->bl[i]-x0i-xdi;
        bu[i]=param->bu[i]-x0i-xdi;
        cvec[i]=cvec[i]-grdpsf[i];
      }
      if (nobL>1) {
        bl[nqpram]=-bgbnd;
        bu[nqpram]=bgbnd;
      }
      ii=ncnstr-nn;
      /*---------------------------------------------------------------*/
      /*     constraints are assigned to a in reverse order            */
      /*---------------------------------------------------------------*/
      k=0;
      for (i=1; i<=ncnstr; i++) {
        jj=iw[ncnstr+1-i];
        if ((jj>glob_info.nnineq)||(jj<=nineqn_tot-ncsipn)||
            ((jj>nineqn_tot)&&(jj<=glob_info.nnineq-ncsipl))||
            cs[jj].act_sip) {
          k++;
          x0i=vv;
          if (i<=(neq-neqn) || (i>neq && i<=(ncnstr-nineqn)))
            x0i=0.e0;
          if (!feasb) x0i=0.e0;
          bj[k]=x0i-cs[jj].val;
          for (j=1; j<=nparam; j++)
            a[k][j]=-cs[jj].grad[j];
          if (nobL>1) a[k][nqpram]=0.e0;
          iw_hold[k]=jj;
        }
      }
      ncnstr_used=k;
      /*---------------------------------------------------------------*/
      /* Assign objectives for QP                                      */
      /*---------------------------------------------------------------*/
      if (nobL==1) {
        for (i=1; i<=nparam; i++)
          cvec[i]=cvec[i]+ob[1].grad[i];
      } else if (nobL>1) {
        numf_used=nob-nfsip+glob_info.tot_actf_sip;
        if (job&&nfsip) {     /* compute # objectives used for dtilde */
          numf_used=0;
          for (i=1; i<=nob; i++)
            if (ob[iw[ncnstr+i]].act_sip) numf_used++;
        }
        for (i=1; i<=nob; i++) {
          ij=ncnstr+i;
          if ((i<=nob-nfsip) || ob[iw[ij]].act_sip) {
            k++;
            iw_hold[k]=iw[ij];  /* record which are used */
            bj[k]=fmax-ob[iw[ij]].val;
            if (nobL>nob) bj[k+numf_used]=fmax+ob[iw[ij]].val;
            for (j=1; j<=nparam; j++) {
              a[k][j]=-ob[iw[ij]].grad[j];
              if (nobL>nob) a[k+numf_used][j]=ob[iw[ij]].grad[j];
            }
            a[k][nqpram]=1.e0;
            if (nobL>nob) a[k+numf_used][nqpram]=1.e0;
          }
        }
        cvec[nqpram]=1.e0;
        if (nobL>nob) k=k+numf_used;  /* k=# rows for a         */
      }                                /*  =# constraints for QP */
      matrcp(nparam,hess,nparam+1,hess1);
      nullvc(nqpram,x);

      iw[1]=1;
      zero=0;
      temp1=neq-neqn;
      temp2=nparam+1;
      mnn=k+2*nqpram;
      htemp=convert(hess1,nparam+1,nparam+1);
      atemp=convert(a,nrowa,nqpram);

      ObjQLD::ql0001_(&k,&temp1,&nrowa,&nqpram,&temp2,&mnn,(htemp+1),
          (cvec+1),(atemp+1),(bj+1),(bl+1),(bu+1),(x+1),
          (clamda+1),&iout,infoqp,&zero,(w+1),&lenw,(iw+1),&leniw,
          &glob_grd.epsmac);

      free_dv(htemp);
      free_dv(atemp);
      if (*infoqp!=0 || job==1) {
        free_iv(iw_hold);
        free_dv(bj);
        return;
      }

      /*---------------------------------------------------------------*/
      /*  Save multipliers from the computation of d0                  */
      /*---------------------------------------------------------------*/
      nullvc(nqpram,param->mult);
      if (ncsipl+ncsipn)
        for (i=1; i<=ncnstr; i++)
          cs[i].mult=0.e0;
      if (nfsip)
        for (i=1; i<=nob; i++) {
          ob[i].mult=0.e0;
          ob[i].mult_L=0.e0;
        }
      for (i=1; i<=nqpram; i++) {
        ii=k+i;
        if (clamda[ii]==0.e0 && clamda[ii+nqpram]==0.e0) continue;
        else if (clamda[ii]!=0.e0) clamda[ii]=-clamda[ii];
        else clamda[ii]=clamda[ii+nqpram];
      }
      nqnp=nqpram+ncnstr;
      for (i=1; i<=nqpram; i++)                  /* Simple bounds */
        param->mult[i]=clamda[k+i];
      if (nctotl>nqnp) {                         /* Objectives    */
        for (i=1; i<=numf_used; i++) {
          ij=ncnstr_used+i;
          if (nobL!=nob) {
            ii=k-2*numf_used+i;
            ob[iw_hold[ij]].mult=clamda[ii]-clamda[ii+numf_used];
            ob[iw_hold[ij]].mult_L=clamda[ii+numf_used];
          } else {
            ii=k-numf_used+i;
            ob[iw_hold[ij]].mult=clamda[ii];
          }
        }
      }
      for (i=1; i<=ncnstr_used; i++)             /* Constraints   */
        cs[iw_hold[i]].mult=clamda[i];
      free_iv(iw_hold);
      free_dv(bj);
      return;
    }


  /****************************************************************/
  /*    Computation of first order direction d1                   */
  /****************************************************************/

  void
    OFSQP::di1(int nparam,int nqpram,int nob,int nobL,int nfsip,int nineqn,
        int neq,int neqn,int ncnstr,int ncsipl,int ncsipn,
        int nrowa,int *infoqp,int mode,struct _parameter *param,
        double *d0,struct _objective *ob,double fmax,double
        *grdpsf,struct _constraint *cs,double *cvec,double *bl,double *bu,
        double *clamda,double **hess1,double *x,double steps)
    {
      int i,k,ii,jj,iout,j,mnn,zero,temp1,temp3,ncnstr_used,numf_used;
      int *iw_hold;
      double x0i,eta,*atemp,*htemp,**a,*bj;

      if ((ncsipl+ncsipn)!=0)
        nrowa=nrowa-(ncsipl+ncsipn)+glob_info.tot_actg_sip;
      if (nfsip) {
        if (nobL>nob) nrowa=nrowa-2*nfsip+2*glob_info.tot_actf_sip;
        else nrowa=nrowa-nfsip+glob_info.tot_actf_sip;
      }
      nrowa=std::max(nrowa,1);
      a=make_dm(nrowa,nqpram);
      bj=make_dv(nrowa);
      iw_hold=make_iv(nrowa);
      iout=6;
      if (mode==0) eta=0.1e0;
      if (mode==1) eta=3.e0;
      for (i=1; i<=nparam; i++) {
        x0i=param->x[i];
        bl[i]=param->bl[i]-x0i;
        bu[i]=param->bu[i]-x0i;
        if (mode==0) cvec[i]=-eta*d0[i];
        if (mode==1) cvec[i]=0.e0;
      }
      bl[nqpram]=-bgbnd;
      bu[nqpram]=bgbnd;
      cvec[nqpram]=1.e0;
      ii=ncnstr-nineqn;
      k=0;
      for (i=1; i<=ncnstr; i++) {
        jj=ncnstr+1-i;
        if ((jj>glob_info.nnineq)||(jj<=nineqn-ncsipn)||
            ((jj>nineqn)&&(jj<=glob_info.nnineq-ncsipl))||
            cs[jj].act_sip) {
          k++;
          bj[k]=-cs[jj].val;
          for (j=1; j<=nparam; j++)
            a[k][j]=-cs[jj].grad[j];
          a[k][nqpram]=0.e0;
          if ((i>(neq-neqn) && i<=neq) || i>ii) a[k][nqpram]=1.e0;
          iw_hold[k]=jj;
        }
      }
      ncnstr_used=k;

      if (mode!=1) {
        numf_used=nob-nfsip+glob_info.tot_actf_sip;
        for (i=1; i<=nob; i++) {
          if ((i<=nob-nfsip) || ob[i].act_sip) {
            k++;
            bj[k]=fmax-ob[i].val;
            for (j=1; j<=nparam; j++) {
              a[k][j]=-ob[i].grad[j]+grdpsf[j];
              if (nobL>nob) a[k+numf_used][j]=ob[i].grad[j]+grdpsf[j];
            }
            a[k][nqpram]=1.e0;
            if (nobL>nob) a[k+numf_used][nqpram]=1.e0;
          }
        }
        if (nob==0) {
          k++;
          bj[k]=fmax;
          for (j=1; j<=nparam; j++)
            a[k][j]=grdpsf[j];
          a[k][nqpram]=1.e0;
        }
      }
      diagnl(nqpram,eta,hess1);
      nullvc(nqpram,x);
      hess1[nqpram][nqpram]=0.e0;

      iw[1]=1;
      zero=0;
      temp1=neq-neqn;
      if (nobL>nob) temp3=k+numf_used;
      else temp3=k;
      mnn=temp3+2*nqpram;
      htemp=convert(hess1,nparam+1,nparam+1);
      atemp=convert(a,nrowa,nqpram);

      ObjQLD::ql0001_(&temp3,&temp1,&nrowa,&nqpram,&nqpram,&mnn,(htemp+1),
          (cvec+1),(atemp+1),(bj+1),(bl+1),(bu+1),(x+1),
          (clamda+1),&iout,infoqp,&zero,(w+1),&lenw,(iw+1),&leniw,
          &glob_grd.epsmac);

      free_dv(htemp);
      free_dv(atemp);
      free_dm(a,nrowa);
      free_dv(bj);
      /* Determine binding constraints */
      if (ncsipl+ncsipn) {
        for (i=1; i<=ncnstr_used; i++)
          if (clamda[i]>0.e0) cs[iw_hold[i]].d1bind=true;
      }
      free_iv(iw_hold);
      return;
    }


  /*****************************************************************/
  /*     CFSQP : Armijo or nonmonotone line search, with some      */
  /*             ad hoc strategies to decrease the number of       */
  /*             function evaluations as much as possible.         */
  /*****************************************************************/


  void
    OFSQP::step1(int nparam,int nob,int nobL,int nfsip,int nineqn,int neq,int neqn,
        int nn,int ncsipl,int ncsipn,int ncnstr,int *ncg,int *ncf,
        int *indxob,int *indxcn,int *iact,int *iskp,int *iskip,
        int *istore,int feasb,double grdftd,struct _objective *ob,
        double *fM,double *fMp,double *fmax,double *psf,double *penp,
        double *steps,double *scvneq,double *xnew,double *x,double *di,
        double *d,struct _constraint *cs,double *backup,double *signeq,
        struct _violation *sip_viol,void *cd)
    {
      int  i,ii,ij,jj,itry,ikeep,j,job,nlin,mnm,ltem1,ltem2;
      bool display,fbind,cdone,fdone,eqdone,reform,sipldone;
      double prod1,prod,dummy,fmax1,tolfe,ostep,temp,**adummy,fii;

      nlin=glob_info.nnineq-nineqn;
      itry=ii=jj=1;
      ostep=*steps=1.e0;
      fbind=cdone=fdone=eqdone=false;
      dummy=0.e0;
      sipldone=(ncsipl==0);
      if (glob_log.local) glob_log.dlfeas=false;
      ikeep=nlin-*iskp;
      prod1=(0.1e0)*grdftd;        /* alpha = 0.1e0  */
      tolfe=0.e0;                  /* feasibility tolerance */
      adummy=make_dm(1,1);
      adummy[1][1]=0.e0;
      if (glob_prnt.iter%glob_prnt.iter_mod) display=false;
      else display=true;
      if (glob_prnt.iprint >= 3 && display)
        sbout1(glob_prnt.io,0,"directional deriv   ",grdftd,*(adummy+1),
            1,2);
      w[1]=*fM;
      for (;;) {
        reform=true;
        if (glob_prnt.iprint >= 3 && display)
          fprintf(glob_prnt.io,"\t\t\t trial number            %22d\n",
              itry);
        prod=prod1*(*steps);
        if (!feasb || (nobL > 1)) prod=prod+tolfe;
        for (i=1; i<=nparam; i++) {
          if (glob_log.local) xnew[i]=x[i]+(*steps)*di[i];
          else xnew[i]=x[i]+(*steps)*di[i]+d[i]*(*steps)*(*steps);
        }
        pb->invalidateX();
        if (glob_prnt.iprint >= 3 && display) {
          sbout1(glob_prnt.io,0,"trial step          ",*steps,
              *(adummy+1),1,2);
          sbout1(glob_prnt.io,nparam,"trial point         ",
              dummy,xnew,2,2);
        }

        /* Generate an upper bound step size using the linear constraints
           not used in the computation of dtilde */
        if (*iskp != 0) {
          ostep=*steps;
          for (i=ii; i<=*iskp; i++) {
            ij=iskip[i];
            pb->constr(nparam,ij,xnew+1,&(cs[ij].val),cd);
            if (glob_prnt.iprint >= 3 && display) {
              if (i==1) fprintf(glob_prnt.io,
                  "\t\t\t trial constraints  %d \t %22.14e\n",ij,
                  cs[ij].val);
              if (i!=1) fprintf(glob_prnt.io,
                  "\t\t\t\t\t %d \t %22.14e\n",ij,cs[ij].val);
            }
            if (cs[ij].val<=tolfe) continue;
            ii=i;
            if (ncsipl && ii>glob_info.nnineq-ncsipl) {
              sip_viol->type = CONSTR;
              sip_viol->index = ij;
            } else {
              sip_viol->type = NONE; /* non-SIP constraint violated */
              sip_viol->index = 0;
            }
            goto L1120;
          }
          *iskp=0;
        }

        /* Refine the upper bound using the linear SI constraints not
           in Omega_k */
        if (!sipldone) {
          for (i=jj; i<=ncsipl; i++) {
            ij=glob_info.nnineq-ncsipl+i;
            if (cs[ij].act_sip||element(iskip,nlin-ikeep,ij))
              continue;
            pb->constr(nparam,ij,xnew+1,&(cs[ij].val),cd);
            if (glob_prnt.iprint >= 3 && display) {
              if (i==1) fprintf(glob_prnt.io,
                  "\t\t\t trial constraints  %d \t %22.14e\n",ij,
                  cs[ij].val);
              if (i!=1) fprintf(glob_prnt.io,
                  "\t\t\t\t\t %d \t %22.14e\n",ij,cs[ij].val);
            }
            if (cs[ij].val<=tolfe) continue;
            jj=i;
            sip_viol->type=CONSTR;
            sip_viol->index=ij;
            goto L1120;
          }
          sipldone=true;
        }
        if (nn==0) goto L310;

        /* Check nonlinear constraints                            */
        if (!glob_log.local && fbind) goto L315;
        do {
          for (i=1; i<=nn; i++) {
            *ncg=i;
            ii=iact[i];
            ij=glob_info.nnineq+neqn;
            if (!((ii<=nineqn && istore[ii]==1)||
                  (ii>glob_info.nnineq && ii<=ij && eqdone))) {
              temp=1.e0;
              if(ii>glob_info.nnineq && ii<=ij)
                temp=signeq[ii-glob_info.nnineq];
              pb->constr(nparam,ii,xnew+1,&(cs[ii].val),cd);
              cs[ii].val *= temp;
              glob_info.ncallg++;
            }
            if (glob_prnt.iprint>=3 && display) {
              if (i==1 && ikeep==nlin) fprintf(glob_prnt.io,
                  "\t\t\t trial constraints  %d \t %22.14e\n",ii,
                  cs[ii].val);
              if (i!=1 || ikeep!=nlin) fprintf(glob_prnt.io,
                  "\t\t\t\t\t %d \t %22.14e\n",ii,cs[ii].val);
            }
            if (!(glob_log.local || cs[ii].val<=tolfe)) {
              shift(nn,ii,iact);
              if (ncsipn && ii>nineqn-ncsipn) {
                sip_viol->type=CONSTR;
                sip_viol->index=ii;
              } else {
                sip_viol->type=NONE; /* non-SIP constraint violated */
                sip_viol->index=0;
              }
              goto L1110;
            }
            if (glob_log.local && cs[ii].val>tolfe) {
              if (ncsipn && ii>nineqn-ncsipn) {
                sip_viol->type=CONSTR;
                sip_viol->index=ii;
              } else {
                sip_viol->type=NONE; /* non-SIP constraint violated */
                sip_viol->index=0;
              }
              goto L1500;
            }
          }
L310:    cdone=eqdone=true;
         if (glob_log.local) glob_log.dlfeas=true; /* dl is feasible */
L315:    if (fdone) break;
         if (nob>0) fmax1=-bgbnd;
         else fmax1=0.e0;
         for (i=1; i<=nob; i++) {		//CHANGE HERE : i=1 instead of i=0, found by Olivier Stasse
           if (nob!=0 && i==0) continue;
           *ncf=i;
           ii=iact[nn+i];
           if (feasb) {
             if (!(eqdone || neqn==0)) {
               for (j=1; j<=neqn; j++)
                 pb->constr(nparam,glob_info.nnineq+j,xnew+1,
                     &(cs[glob_info.nnineq+j].val),cd);
               glob_info.ncallg+=neqn;
             }
             if (neqn != 0) {
               if (eqdone)    job=20;
               if (!eqdone)   job=10;
               resign(nparam,neqn,psf,*(adummy+1),penp,cs,signeq,
                   job,10);
             }
             if (istore[nineqn+ii]!=1 && i!=0) {
               pb->obj(nparam,ii,xnew+1,&(ob[ii].val),cd);
               glob_info.ncallf++;
             }
             if (i==0) fii=0.e0;
             else fii=ob[ii].val;
             if (i==0 && glob_prnt.iprint>=3 && display)
               fprintf(glob_prnt.io,
                   "\t\t\t trial penalty term \t %22.14e\n",-*psf);
             if (i==1 && glob_prnt.iprint>=3 && display)
               fprintf(glob_prnt.io,
                   "\t\t\t trial objectives   %d \t %22.14e\n",
                   ii,fii-*psf);
             if (i>1 && glob_prnt.iprint>=3 && display)
               fprintf(glob_prnt.io,
                   "\t\t\t\t\t %d \t %22.14e\n",ii,fii-*psf);
           } else {
             if (istore[ii]!=1) {
               pb->constr(nparam,indxob[ii],xnew+1,&(ob[ii].val),cd);
               glob_info.ncallg++;
             }
             if (ob[ii].val>tolfe) reform=false;
             if (i==1 && glob_prnt.iprint>2 && display)
               fprintf(glob_prnt.io,
                   "\t\t\t trial objectives   %d \t %22.14e\n",
                   indxob[ii],ob[ii].val);
             if (i!=1 && glob_prnt.iprint>2 && display)
               fprintf(glob_prnt.io,
                   "\t\t\t\t\t %d \t %22.14e\n",indxob[ii],
                   ob[ii].val);
             fii=ob[ii].val;
           }
           fmax1=std::max(fmax1,fii);
           if (nobL!=nob) fmax1=std::max(fmax1,-fii);
           if (!feasb && reform) continue;
           if (!glob_log.local) {
             if ((fii-*psf)>(*fMp+prod)) {
               fbind=true;
               shift(nob,ii,&iact[nn]);
               if (nfsip && ii>nob-nfsip) {
                 sip_viol->type=OBJECT;
                 sip_viol->index=ii;
               } else {
                 sip_viol->type=NONE;
                 sip_viol->index=0;
               }
               goto L1110;
             }
             if (nobL==nob || (-fii-*psf)<=(*fMp+prod))
               continue;
             fbind=true;
             shift(nob,ii,&iact[nn]);
             if (nfsip && ii>nob-nfsip) {
               sip_viol->type=OBJECT;
               sip_viol->index=ii;
             } else {
               sip_viol->type=NONE;
               sip_viol->index=0;
             }
             goto L1110;
           }
           ltem1=(fii-*psf)>(*fMp+prod);
           ltem2=(nobL!=nob)&&((-fii-*psf)>(*fMp+prod));
           if (ltem1 || ltem2) goto L1500;
         }
         fbind=false;
         fdone=eqdone=true;
        } while (!cdone);
        if (ostep==*steps) mnm=ikeep+neq-neqn;
        if (ostep!=*steps) mnm=ncnstr-nn;
        for (i=1; i<=mnm; i++) {
          ii=indxcn[i+nn];
          if (ikeep!=nlin && ostep==*steps) {
            if (i<= ikeep) ii=iskip[nlin+2-i];
            else ii=indxcn[nn+i-ikeep+nlin];
          }
          pb->constr(nparam,ii,xnew+1,&(cs[ii].val),cd);
        }
        *scvneq=0.e0;
        for (i=1; i<=ncnstr; i++) {
          if (i>glob_info.nnineq && i<=(glob_info.nnineq+neqn))
            *scvneq=*scvneq-cs[i].val;
          backup[i]=cs[i].val;
        }
        for (i=1; i<=nob; i++)
          backup[i+ncnstr]=ob[i].val;
        if (!feasb && reform) {
          for (i=1; i<=nparam; i++)
            x[i]=xnew[i];
          nstop=0;
          goto L1500;
        }
        if (glob_log.local) *ncg=ncnstr;
        if (glob_log.local) glob_log.update=true;
        *fM=fmax1;
        *fMp=fmax1-*psf;
        *fmax=fmax1;
        for (i=1; i<=nn; i++)
          iact[i]=indxcn[i];
        for (i=1; i<=nob; i++)
          iact[nn+i]=i;
        goto L1500;
L1110:
        cdone=fdone=eqdone=reform=false;
L1120:
        itry++;
        if (glob_info.modec==2) fbind=false;
        if (*steps >= 1.e0)
          for (i=1; i<=nob+nineqn; i++)
            istore[i]=0;
        *steps=*steps*0.5e0;
        if (*steps<glob_grd.epsmac) break;
      }
      glob_prnt.info=4;
      nstop=0;
L1500:
      free_dm(adummy,1);
      if (*steps<1.e0) return;
      for (i=1; i<=nob+nineqn; i++)
        istore[i]=0;
      return;
    }



  /******************************************************************/
  /*   CFSQP : Update the Hessian matrix using BFGS formula with    */
  /*           Powell's modification.                               */
  /******************************************************************/

  void
    OFSQP::hessian(int nparam,int nob,int nfsip,int nobL,int nineqn,int neq,
        int neqn,int nn,int ncsipn,int ncnstr,int nfs,int *nstart,
        int feasb,double *xnew,struct _parameter *param,
        struct _objective *ob,double fmax,double *fM,double *fMp,
        double *psf,double *grdpsf,double *penp,struct _constraint *cs,
        double *gm,int *indxob,int *indxcn,double *delta,double *eta,
        double *gamma,double **hess,double *hd,double steps,int *nrst,
        double *signeq,double *span,
        double **phess,double *psb,double *psmu,
        struct _violation *sip_viol)
    {
      int    i,j,k,ifail,np,mnm;
      bool   display,done;
      double dhd,gammd,etad,dummy,theta,signgj,psfnew,delta_s;
      double *tempv;

      /* Check to see whether user-accessible stopping criterion
         is satisfied. The check of gLgeps is made just after
         computing d0 */

      if (!glob_log.get_ne_mult) {
        if (feasb && nstop && !neqn)
          if ((fabs(w[1]-fmax) <= objeps) ||
              (fabs(w[1]-fmax) <= objrep*fabs(w[1]))) nstop=0;
        if (!nstop) {
          for (i=1; i<= nparam; i++)
            param->x[i]=xnew[i];
          pb->invalidateX();
          return;
        }
      }

      delta_s=glob_grd.rteps;  /* SIP */
      if (glob_prnt.iter%glob_prnt.iter_mod) display=false;
      else display=true;
      psfnew=0.e0;
      glob_prnt.ipd=0;
      done=false;
      dummy=0.e0;
      nullvc(nparam,delta);
      nullvc(nparam,eta);
      for (j=1; j<=2; j++) {
        nullvc(nparam, gamma);
        if (nobL>1) {
          for (i=1; i<=nparam; i++) {
            hd[i]=0.e0;
            for (k=1; k<=nob; k++)
              hd[i]=hd[i]+ob[k].grad[i]*ob[k].mult;
          }
        }
        if (feasb) {
          if (nineqn != 0) {
            for (i=1; i<=nparam; i++) {
              gamma[i]=0.e0;
              for (k=1; k<=nineqn; k++)
                gamma[i]=gamma[i]+cs[k].grad[i]*cs[k].mult;
            }
          }
          if (neqn != 0) {
            for (i=1; i<=nparam; i++) {
              eta[i]=0.e0;
              for (k=1; k<=neqn; k++)
                eta[i]=eta[i]+cs[glob_info.nnineq+k].grad[i]*
                  cs[glob_info.nnineq+k].mult;
            }
          }
        }
        for (i=1; i<=nparam; i++) {
          if (nobL>1) {
            if (done) psb[i]=hd[i]+param->mult[i]+gamma[i];
            gamma[i]=gamma[i]+hd[i]-grdpsf[i]+eta[i];
          } else if (nobL==1) {
            if (done) psb[i]=ob[1].grad[i]+param->mult[i]+gamma[i];
            gamma[i]=gamma[i]+ob[1].grad[i]-grdpsf[i]+eta[i];
          } else if (nobL==0) {
            if (done) psb[i]=param->mult[i]+gamma[i];
            gamma[i]=gamma[i]-grdpsf[i]+eta[i];
          }
          if (!done) delta[i]=gamma[i];
        }
        if (!done && !glob_log.d0_is0) {
          if (nn != 0) {
            for (i=1; i<=nn; i++) {
              if ((feasb) && (i>nineqn)) signgj=signeq[i-nineqn];
              if ((!feasb) || (i<=nineqn)) signgj=1.e0;
              if ((feasb) && (ncsipn) && (i>nineqn-ncsipn) &&
                  (cs[indxcn[i]].mult==0.e0)) continue;
              glob_grd.valnom=cs[indxcn[i]].val*signgj;
              pb->gradcn(nparam,indxcn[i],xnew+1,cs[indxcn[i]].grad+1,param->cd);
            }
            resign(nparam,neqn,psf,grdpsf,penp,cs,signeq,11,11);
          }
          for (i=1; i<=nob; i++) {
            glob_grd.valnom=ob[i].val;
            if ((i<=nob-nfsip)||(i>nob-nfsip &&
                  ((ob[i].mult !=0.e0)||(ob[i].mult_L !=0.e0)))) {
              if (feasb)
                pb->gradob(nparam,i,xnew+1,ob[i].grad+1,param->cd);
              else pb->gradcn(nparam,indxob[i],xnew+1,ob[i].grad+1,param->cd);
            }
          }
          done=true;
        }
        if (glob_log.d0_is0) done=true;
      }
      if (!glob_log.d0_is0) {
        if (!(feasb && steps<delta_s && ((sip_viol->type==OBJECT &&
                  !ob[sip_viol->index].act_sip)||(sip_viol->type==CONSTR &&
                    !cs[sip_viol->index].act_sip)))) {
          if (*nrst<(5*nparam) || steps>0.1e0) {
            (*nrst)++;
            for (i=1; i<=nparam; i++) {
              gamma[i]=gamma[i]-delta[i];
              delta[i]=xnew[i]-param->x[i];
            }
            matrvc(nparam,nparam,hess,delta,hd);
            dhd=scaprd(nparam,delta,hd);
            if (sqrt(scaprd(nparam,delta,delta)) <= glob_grd.epsmac) {
              /* xnew too close to x!! */
              nstop=0;
              glob_prnt.info=8;
              return;
            }
            gammd=scaprd(nparam,delta,gamma);
            if (gammd >= (0.2e0*dhd)) theta=1.e0;
            else theta=0.8e0*dhd/(dhd-gammd);
            for (i=1; i<=nparam; i++)
              eta[i]=hd[i]*(1.e0-theta)+theta*gamma[i];
            etad=theta*gammd+(1.e0-theta)*dhd;
            for (i=1; i<=nparam; i++) {
              for (j=i; j<=nparam; j++) {
                hess[i][j]=hess[i][j] - hd[i]*hd[j]/dhd +
                  eta[i]*eta[j]/etad;
                hess[j][i]=hess[i][j];
              }
            }
          } else {
            *nrst=0;
            diagnl(nparam,1.e0,hess);
          }
        }
        for (i=1; i<=nparam; i++)
          param->x[i]=xnew[i];
        pb->invalidateX();
      }
      if (neqn!=0 && (feasb)) {
        i=glob_info.nnineq-nineqn;
        if (i!=0) {
          for (j=1; j<=nparam; j++) {
            gamma[j]=0.e0;
            for (k=1; k<=i; k++)
              gamma[j]=gamma[j]+cs[k+nineqn].grad[j]*
                cs[nineqn+k].mult;
          }
          for (i=1; i<=nparam; i++)
            psb[i]=psb[i]+gamma[i];
        }
        i=neq-neqn;
        if (i!=0) {
          for (j=1; j<=nparam; j++) {
            gamma[j]=0.e0;
            for (k=1; k<=i; k++)
              gamma[j]=gamma[j]+cs[k+neqn+glob_info.nnineq].grad[j]*
                cs[glob_info.nnineq+neqn+k].mult;
          }
          for (i=1; i<=nparam; i++)
            psb[i]=psb[i]+gamma[i];
        }
        /* Update penalty parameters for nonlinear equality constraints */
        estlam(nparam,neqn,&ifail,bgbnd,phess,delta,eta,
            gamma,cs,psb,hd,xnew,psmu);
        if (glob_log.get_ne_mult) return;
        for (i=1; i<=neqn; i++) {
          if (ifail != 0 || glob_log.d0_is0) penp[i]=2.e0*penp[i];
          else {
            etad=psmu[i]+penp[i];
            if (etad < 1.e0)
              penp[i]=std::max((1.e0-psmu[i]),(2.e0*penp[i]));
          }
          if (penp[i] > bgbnd) {
            nstop=0;
            glob_prnt.info=9;
            return;
          }
        }
        resign(nparam,neqn,psf,grdpsf,penp,cs,signeq,20,12);
        *fMp=*fM-*psf;
      }
      if (nfs != 0) {
        (*nstart)++;
        np=indexs(*nstart,nfs);
        span[np]=fmax;
        for (i=1; i<=neqn; i++)
          gm[(np-1)*neqn+i]=cs[glob_info.nnineq+i].val;
        if (neqn != 0) {
          psfnew=0.e0;
          for (i=1; i<=neqn; i++)
            psfnew=psfnew+gm[i]*penp[i];
        }
        *fM=span[1];
        *fMp=span[1]-psfnew;
        mnm=std::min(*nstart,nfs);
        for (i=2; i<=mnm; i++) {
          if (neqn != 0) {
            psfnew=0.e0;
            for (j=1; j<=neqn; j++)
              psfnew=psfnew+gm[(i-1)*neqn +j]*penp[j];
          }
          *fM=std::max(*fM, span[i]);
          *fMp=std::max(*fMp,span[i]-psfnew);
        }
      }
      if (glob_prnt.iprint < 3 || !display) return;
      for (i=1; i<=nob; i++) {
        if (!feasb) {
          sbout2(glob_prnt.io,nparam,indxob[i],"gradg(j,",
              ")",ob[i].grad);
          continue;
        }
        if (nob>1) sbout2(glob_prnt.io,nparam,i,"gradf(j,",")",
            ob[i].grad);
        if (nob==1) sbout1(glob_prnt.io,nparam,"gradf(j)            ",
            dummy,ob[1].grad,2,2);
      }
      if (ncnstr != 0) {
        for (i=1; i<=ncnstr; i++) {
          tempv=cs[indxcn[i]].grad;
          sbout2(glob_prnt.io,nparam,indxcn[i],"gradg(j,",")",tempv);
        }
        if (neqn != 0) {
          sbout1(glob_prnt.io,nparam,"grdpsf(j)           ",
              dummy,grdpsf,2,2);
          sbout1(glob_prnt.io,neqn,"P                   ",dummy,
              penp,2,2);
          sbout1(glob_prnt.io,neqn,"psmu                ",dummy,
              psmu,2,2);
        }
      }
      sbout1(glob_prnt.io,nparam,"multipliers for x   ",dummy,
          param->mult,2,2);
      if (ncnstr != 0) {
        fprintf(glob_prnt.io,"\t\t\t %s\t %22.14e\n",
            "            for g   ",cs[1].mult);
        for (j=2; j<=ncnstr; j++)
          fprintf(glob_prnt.io," \t\t\t\t\t\t %22.14e\n",cs[j].mult);
      }
      if (nobL > 1) {
        fprintf(glob_prnt.io,"\t\t\t %s\t %22.14e\n",
            "            for f   ",ob[1].mult);
        for (j=2; j<=nob; j++)
          fprintf(glob_prnt.io," \t\t\t\t\t\t %22.14e\n",ob[j].mult);
      }
      for (i=1; i<=nparam; i++) {
        tempv=colvec(hess,i,nparam);
        sbout2(glob_prnt.io,nparam,i,"hess (j,",")",tempv);
        free_dv(tempv);
      }
      return;
    }


  /**************************************************************/
  /*   CFSQP : Output                                           */
  /**************************************************************/


  void
    OFSQP::out(int miter,int nparam,int nob,int nobL,int nfsip,int ncn,
        int nn,int nineqn,int ncnstr,int ncsipl,int ncsipn,
        int *mesh_pts,double *x,struct _constraint *cs,
        struct _objective *ob,double fM,double fmax,
        double steps,double sktnom,double d0norm,int feasb)
    {
      int i,j,index,offset;
      bool display;
      double SNECV,dummy,*adummy,gmax;

      adummy=make_dv(1);
      adummy[1]=0.e0;
      dummy=0.e0;
      if (glob_prnt.iter>=miter && nstop != 0) {
        glob_prnt.info=3;
        nstop=0;
        if (glob_prnt.iprint==0) goto L9000;
      }
      if (glob_prnt.iprint==0 && glob_prnt.iter<miter) {
        glob_prnt.iter++;
        goto L9000;
      }
      if ((glob_prnt.info>0 && glob_prnt.info<3) || glob_prnt.info==7)
        goto L120;
      if (glob_prnt.iprint==1 && nstop!=0) {
        glob_prnt.iter++;
        if (glob_prnt.initvl==0) goto L9000;
        if (feasb && nob>0) {
          fprintf(glob_prnt.io," objectives\n");
          for (i=1; i<=nob-nfsip; i++) {
            if (nob==nobL)
              fprintf(glob_prnt.io," \t\t\t %22.14e\n",ob[i].val);
            else
              fprintf(glob_prnt.io," \t\t\t %22.14e\n",fabs(ob[i].val));
          }
          if (nfsip) {
            offset=nob-nfsip;
            for (i=1; i<=glob_info.nfsip; i++) {
              if (nob==nobL) gmax=ob[++offset].val;
              else gmax=fabs(ob[++offset].val);
              for (j=2; j<=mesh_pts[i]; j++) {
                offset++;
                if (nob==nobL && ob[offset].val>gmax)
                  gmax=ob[offset].val;
                else if (nob!=nobL && fabs(ob[offset].val)>gmax)
                  gmax=fabs(ob[offset].val);
              }
              fprintf(glob_prnt.io," \t\t\t %22.14e\n",gmax);
            }
          }
        }
        if (glob_info.mode==1 && glob_prnt.iter>1 && feasb)
          sbout1(glob_prnt.io,0,"objective max4      ",fM,adummy,1,1);
        if (nob>1)
          sbout1(glob_prnt.io,0,"objmax              ",fmax,adummy,1,1);
        if (ncnstr==0) fprintf(glob_prnt.io,"\n");
        else {
          fprintf(glob_prnt.io," constraints\n");
          for (i=1; i<=nineqn-ncsipn; i++)
            fprintf(glob_prnt.io," \t\t\t %22.14e\n",cs[i].val);
          if (ncsipn) {
            offset=nineqn-ncsipn;
            for (i=1; i<=glob_info.ncsipn; i++) {
              gmax=cs[++offset].val;
              for (j=2; j<=mesh_pts[glob_info.nfsip+i]; j++) {
                offset++;
                if (cs[offset].val>gmax) gmax=cs[offset].val;
              }
              fprintf(glob_prnt.io," \t\t\t %22.14e\n",gmax);
            }
          }
          for (i=nineqn+1; i<=glob_info.nnineq-ncsipl; i++)
            fprintf(glob_prnt.io," \t\t\t %22.14e\n",cs[i].val);
          if (ncsipl) {
            offset=glob_info.nnineq-ncsipl;
            for (i=1; i<=glob_info.ncsipl; i++) {
              gmax=cs[++offset].val;
              if (feasb) index=glob_info.nfsip+glob_info.ncsipn+i;
              else index=glob_info.ncsipn+i;
              for (j=2; j<=mesh_pts[index]; j++) {
                offset++;
                if (cs[offset].val>gmax) gmax=cs[offset].val;
              }
              fprintf(glob_prnt.io," \t\t\t %22.14e\n",gmax);
            }
          }
          for (i=glob_info.nnineq+1; i<=ncnstr; i++)
            fprintf(glob_prnt.io," \t\t\t %22.14e\n",cs[i].val);
        }
        if (ncnstr!=0) fprintf(glob_prnt.io,"\n");
        goto L9000;
      }
      if (glob_prnt.iprint==1 && nstop==0)
        fprintf(glob_prnt.io," iteration           %26d\n",
            glob_prnt.iter);
      if (glob_prnt.iprint<=2 && nstop==0)
        fprintf(glob_prnt.io," inform              %26d\n",
            glob_prnt.info);
      if (glob_prnt.iprint==1 && nstop==0 && (ncsipl+ncsipn)!=0)
        fprintf(glob_prnt.io," |Xi_k|              %26d\n",
            glob_info.tot_actg_sip);
      if (glob_prnt.iprint==1 && nstop==0 && nfsip!=0)
        fprintf(glob_prnt.io," |Omega_k|           %26d\n",
            glob_info.tot_actf_sip);
      glob_prnt.iter++;
      if (!((glob_prnt.iter)%glob_prnt.iter_mod)) display=true;
      else display=(nstop==0);
      if (glob_prnt.iter_mod!=1 && display)
        fprintf(glob_prnt.io,"\n iteration           %26d\n",
            glob_prnt.iter-1);
      if (glob_prnt.initvl==0 && display)
        sbout1(glob_prnt.io,nparam,"x                   ",dummy,x,2,1);
      if (display) {
        if (nob>0) {
          fprintf(glob_prnt.io," objectives\n");
          for (i=1; i<=nob-nfsip; i++) {
            if (nob==nobL)
              fprintf(glob_prnt.io," \t\t\t %22.14e\n",ob[i].val);
            else
              fprintf(glob_prnt.io," \t\t\t %22.14e\n",
                  fabs(ob[i].val));
          }
        }
        if (nfsip) {
          offset=nob-nfsip;
          if (feasb) index=glob_info.nfsip;
          else index=glob_info.ncsipn;
          for (i=1; i<=index; i++) {
            if (nob==nobL) gmax=ob[++offset].val;
            else gmax=fabs(ob[++offset].val);
            for (j=2; j<=mesh_pts[i]; j++) {
              offset++;
              if (nob==nobL && ob[offset].val>gmax)
                gmax=ob[offset].val;
              else if (nob!=nobL && fabs(ob[offset].val)>gmax)
                gmax=fabs(ob[offset].val);
            }
            fprintf(glob_prnt.io," \t\t\t %22.14e\n",gmax);
          }
        }
      }
      if (glob_info.mode==1 && glob_prnt.iter>1 && display)
        sbout1(glob_prnt.io,0,"objective max4      ",fM,adummy,1,1);
      if (nob>1 && display)
        sbout1(glob_prnt.io,0,"objmax              ",fmax,adummy,1,1);
      if (ncnstr != 0 && display ) {
        fprintf(glob_prnt.io," constraints\n");
        for (i=1; i<=nineqn-ncsipn; i++)
          fprintf(glob_prnt.io," \t\t\t %22.14e\n",cs[i].val);
        if (ncsipn) {
          offset=nineqn-ncsipn;
          for (i=1; i<=glob_info.ncsipn; i++) {
            gmax=cs[++offset].val;
            for (j=2; j<=mesh_pts[glob_info.nfsip+i]; j++) {
              offset++;
              if (cs[offset].val>gmax) gmax=cs[offset].val;
            }
            fprintf(glob_prnt.io," \t\t\t %22.14e\n",gmax);
          }
        }
        for (i=nineqn+1; i<=glob_info.nnineq-ncsipl; i++)
          fprintf(glob_prnt.io," \t\t\t %22.14e\n",cs[i].val);
        if (ncsipl) {
          offset=glob_info.nnineq-ncsipl;
          for (i=1; i<=glob_info.ncsipl; i++) {
            gmax=cs[++offset].val;
            if (feasb) index=glob_info.nfsip+glob_info.ncsipn+i;
            else index=glob_info.ncsipn+i;
            for (j=2; j<=mesh_pts[index];
                j++) {
              offset++;
              if (cs[offset].val>gmax) gmax=cs[offset].val;
            }
            fprintf(glob_prnt.io," \t\t\t %22.14e\n",gmax);
          }
        }
        for (i=glob_info.nnineq+1; i<=ncnstr; i++)
          fprintf(glob_prnt.io," \t\t\t %22.14e\n",cs[i].val);
        if (feasb) {
          SNECV=0.e0;
          for (i=glob_info.nnineq+1; i<=glob_info.nnineq+nn-nineqn; i++)
            SNECV=SNECV+fabs(cs[i].val);
          if (glob_prnt.initvl==0 && (nn-nineqn)!=0)
            sbout1(glob_prnt.io,0,"SNECV               ",
                SNECV,adummy,1,1);
        }
      }
      if (glob_prnt.iter<=1 && display) {
        fprintf(glob_prnt.io," \n");
        fprintf(glob_prnt.io," iteration           %26d\n",
            glob_prnt.iter);
        goto L9000;
      }
      if (glob_prnt.iprint>=2 && glob_prnt.initvl==0 && display)
        sbout1(glob_prnt.io,0,"step                ",steps,adummy,1,1);
      if (glob_prnt.initvl==0 && display &&
          (nstop==0 || glob_prnt.info!=0 || glob_prnt.iprint==2)) {
        sbout1(glob_prnt.io,0,"d0norm              ",d0norm,adummy,1,1);
        sbout1(glob_prnt.io,0,"ktnorm              ",sktnom,adummy,1,1);
      }
      if (glob_prnt.initvl==0 && feasb && display)
        fprintf(glob_prnt.io," ncallf              %26d\n",
            glob_info.ncallf);
      if (glob_prnt.initvl==0 && (nn!=0 || !feasb) && display)
        fprintf(glob_prnt.io," ncallg              %26d\n",
            glob_info.ncallg);
      if (glob_prnt.iprint>=3 && glob_prnt.iter_mod!=1 && nstop!=0
          && !(glob_prnt.iter%glob_prnt.iter_mod))
        fprintf(glob_prnt.io,
            "\n The following was calculated during iteration %5d:\n",
            glob_prnt.iter);
      if (nstop != 0 && (glob_prnt.iter_mod==1))
        fprintf(glob_prnt.io,"\n iteration           %26d\n",
            glob_prnt.iter);
L120:
      if (nstop!=0 || glob_prnt.iprint==0) goto L9000;
      fprintf(glob_prnt.io,"\n");
      if (glob_prnt.iprint>=3)
        fprintf(glob_prnt.io," inform              %26d\n",
            glob_prnt.info);
      if (glob_prnt.info==0) fprintf(glob_prnt.io,
          "\nNormal termination: You have obtained a solution !!\n");
      if (glob_prnt.info==0 && sktnom>0.1e0) fprintf(glob_prnt.io,
          "Warning: Norm of Kuhn-Tucker vector is large !!\n");
      if (glob_prnt.info==3) {
        fprintf(glob_prnt.io,
            "\nWarning: Maximum iterations have been reached before\n");
        fprintf(glob_prnt.io,"obtaining a solution !!\n\n");
      }
      if (glob_prnt.info==4) {
        fprintf(glob_prnt.io,
            "\nError : Step size has been smaller than the computed\n");
        fprintf(glob_prnt.io,"machine precision !!\n\n");
      }
      if (glob_prnt.info==5) fprintf(glob_prnt.io,
          "\nError: Failure in constructing d0 !!\n\n");
      if (glob_prnt.info==6) fprintf(glob_prnt.io,
          "\nError: Failure in constructing d1 !!\n\n");
      if (glob_prnt.info==8) {
        fprintf(glob_prnt.io,
            "\nError: The new iterate is numerically equivalent to the\n");
        fprintf(glob_prnt.io,
            "previous iterate, though the stopping criterion is not \n");
        fprintf(glob_prnt.io,"satisfied\n");
      }
      if (glob_prnt.info==9) {
        fprintf(glob_prnt.io,
            "\nError: Could not satisfy nonlinear equality constraints -\n");
        fprintf(glob_prnt.io,"       Penalty parameter too large\n");
      }
      fprintf(glob_prnt.io,"\n");
L9000:
      free_dv(adummy);
      glob_prnt.initvl=0;
      return;
    }


  /************************************************************/
  /*    Utility functions used by CFSQP -                     */
  /*    Available functions:                                  */
  /*      diagnl        error         estlam                  */
  /*      colvec        scaprd        small                   */
  /*      fool          matrvc        matrcp                  */
  /*      nullvc        resign        sbout1                  */
  /*      sbout2        shift         slope                   */
  /*      fuscmp        indexs        element                 */
  /************************************************************/

  /************************************************************/
  /*    Set a=diag*I, a diagonal matrix                       */
  /************************************************************/


  void OFSQP::diagnl(int nrowa,double diag,double **a)
  {
    int i,j;

    for (i=1; i<=nrowa; i++) {
      for (j=i; j<=nrowa; j++) {
        a[i][j]=0.e0;
        a[j][i]=0.e0;
      }
      a[i][i]=diag;
    }
    return;
  }

  /***********************************************************/
  /*    Display error messages                               */
  /***********************************************************/


  void OFSQP::error(char string[],int *inform)
  {
    if (glob_prnt.iprint>0)
      fprintf(stderr,"%s\n",string);
    *inform=7;
    return;
  }


  /***********************************************************/
  /*    Compute an estimate of multipliers for updating      */
  /*    penalty parameter (nonlinear equality constraints)   */
  /***********************************************************/


  void
    OFSQP::estlam(int nparam,int neqn,int *ifail,double bigbnd,double **hess,
        double *cvec,double *a,double *b,struct _constraint *cs,
        double *psb,double *bl,double *bu,double *x)
    {
      int i,j,zero,one,lwar2,mnn,iout;
      double *ctemp;

      for (i=1; i<=neqn; i++) {
        bl[i]= (-bigbnd);
        bu[i]= bigbnd;
        cvec[i]=scaprd(nparam,cs[i+glob_info.nnineq].grad,psb);
        x[i]= 0.e0;
        for (j=i; j<=neqn; j++) {
          hess[i][j]=scaprd(nparam,cs[i+glob_info.nnineq].grad,
              cs[j+glob_info.nnineq].grad);
          hess[j][i]=hess[i][j];
        }
      }
      zero=0;
      one=1;
      iw[1]=1;
      mnn=2*neqn;
      ctemp=convert(hess,neqn,neqn);
      lwar2=lenw-1;
      iout=6;

      ObjQLD::ql0001_(&zero,&zero,&one,&neqn,&neqn,&mnn,(ctemp+1),(cvec+1),
          (a+1),(b+1),(bl+1),(bu+1),(x+1),(w+1),&iout,ifail,
          &zero,(w+3),&lwar2,(iw+1),&leniw,&glob_grd.epsmac);

      free_dv(ctemp);
      return;
    }


  /**************************************************************/
  /*   Extract a column vector from a matrix                    */
  /**************************************************************/


  double *OFSQP::colvec(double **a,int col,int nrows)
  {
    double *temp;
    int i;

    temp=make_dv(nrows);
    for (i=1;i<=nrows;i++)
      temp[i]=a[i][col];
    return temp;
  }


  /************************************************************/
  /*    Compute the scalar product z=x'y                      */
  /************************************************************/


  double OFSQP::scaprd(int n,double *x,double *y)
  {
    int i;
    double z;

    z=0.e0;
    for (i=1;i<=n;i++)
      z=z+x[i]*y[i];
    return z;
  }


  /***********************************************************/
  /*    Used by small()                                      */
  /***********************************************************/


  void OFSQP::fool(double x,double y,double *z)
  {
    *z=x*y+y;
    return;
  }


  /**********************************************************/
  /*    Computes the machine precision                      */
  /**********************************************************/


  double OFSQP::small()
  {
    double one,two,z,tsmall;

    one=1.e0;
    two=2.e0;
    tsmall=one;
    do {
      tsmall=tsmall/two;
      fool(tsmall,one,&z);
    } while (z>1.e0);
    return tsmall*two*two;
  }


  /**********************************************************/
  /*     Compares value with threshold to see if exceeds    */
  /**********************************************************/


  bool OFSQP::fuscmp(double val,double thrshd)
  {
    bool temp;

    if (fabs(val)<=thrshd) temp=false;
    else temp=true;
    return temp;
  }


  /**********************************************************/
  /*     Find the residue of i with respect to nfs          */
  /**********************************************************/


  int OFSQP::indexs(int i,int nfs)
  {
    int mm=i;

    while (mm>nfs) mm -= nfs;
    return mm;
  }


  /*********************************************************/
  /*     Copies matrix a to matrix b                       */
  /*********************************************************/


  void OFSQP::matrcp(int ndima,double **a,int ndimb,double **b)
  {
    int i,j;

    for (i=1; i<=ndima; i++)
      for (j=1; j<=ndima; j++)
        b[i][j]=a[i][j];
    if (ndimb<=ndima) return;
    for (i=1; i<=ndimb; i++) {
      b[ndimb][i]=0.e0;
      b[i][ndimb]=0.e0;
    }
    return;
  }

  /*******************************************************/
  /*     Computes y=ax                                   */
  /*******************************************************/


  void OFSQP::matrvc(int la,int na,double **a,double *x,double *y)
  {
    int i,j;
    double yi;

    for (i=1; i<=la; i++) {
      yi=0.e0;
      for (j=1; j<=na; j++)
        yi=yi + a[i][j]*x[j];
      y[i]=yi;
    }
    return;
  }

  /******************************************************/
  /*      Set x=0                                       */
  /******************************************************/


  void OFSQP::nullvc(int nparam,double *x)
  {
    int i;

    for (i=1; i<=nparam; i++)
      x[i]=0.e0;
    return;
  }

  /*********************************************************/
  /*   job1=10: g*signeq,   job1=11: gradg*signeq,         */
  /*                        job1=12: job1=10&11            */
  /*   job1=20: do not change sign                         */
  /*   job2=10: psf,        job2=11: grdpsf,               */
  /*                        job2=12: job2=10&11            */
  /*   job2=20: do not change sign                         */
  /*********************************************************/


  void
    OFSQP::resign(int n,int neqn,double *psf,double *grdpsf,double *penp,
        struct _constraint *cs,double *signeq,int job1,int job2)
    {
      int i,j,nineq;

      nineq=glob_info.nnineq;
      if (job2==10 || job2==12) *psf=0.e0;
      for (i=1; i<=neqn; i++) {
        if (job1==10 || job1==12) cs[i+nineq].val=
          signeq[i]*cs[i+nineq].val;
        if (job2==10 || job2==12) *psf=*psf+cs[i+nineq].val*penp[i];
        if (job1==10 || job1==20) continue;
        for (j=1; j<=n; j++)
          cs[i+nineq].grad[j]=cs[i+nineq].grad[j]*signeq[i];
      }
      if (job2==10 || job2==20) return;
      nullvc(n,grdpsf);
      for (i=1; i<=n; i++)
        for (j=1; j<=neqn; j++)
          grdpsf[i]=grdpsf[i]+cs[j+nineq].grad[i]*penp[j];
      return;
    }

  /**********************************************************/
  /*      Write output to file                              */
  /**********************************************************/


  void
    OFSQP::sbout1(FILE *io,int n,char *s1,double z,double *z1,int job,int level)
    {
      int j;

      if (job != 2) {
        if (level == 1) fprintf(io," %s\t %22.14e\n",s1,z);
        if (level == 2) fprintf(io,"\t\t\t %s\t %22.14e\n",s1,z);
        return;
      }
      if (n==0) return;
      if (level == 1) fprintf(io," %s\t %22.14e\n",s1,z1[1]);
      if (level == 2) fprintf(io,"\t\t\t %s\t %22.14e\n",s1,z1[1]);
      for (j=2; j<=n; j++) {
        if (level == 1) fprintf(io," \t\t\t %22.14e\n",z1[j]);
        if (level == 2) fprintf(io," \t\t\t\t\t\t %22.14e\n",z1[j]);
      }
      return;
    }

  /*********************************************************/
  /*      Write output to file                             */
  /*********************************************************/


  void
    OFSQP::sbout2(FILE *io,int n,int i,char *s1,char *s2,double *z)
    {
      int j;

      fprintf(io,"\t\t\t %8s %5d %1s\t %22.14e\n",s1,i,s2,z[1]);
      for (j=2; j<=n; j++)
        fprintf(io,"\t\t\t\t\t\t %22.14e\n",z[j]);
      return;
    }


  /*********************************************************/
  /*      Extract ii from iact and push in front           */
  /*********************************************************/


  void OFSQP::shift(int n,int ii,int *iact)
  {
    int j,k;

    if (ii == iact[1]) return;
    for (j=1; j<=n; j++) {
      if (ii != iact[j]) continue;
      for (k=j; k>=2; k--)
        iact[k]=iact[k-1];
      break;
    }
    if (n!=0) iact[1]=ii;
    return;
  }

  /****************************************************************/
  /*      job=0 : Compute the generalized gradient of the minimax */
  /*      job=1 : Compute rhog in mode = 1                        */
  /****************************************************************/


  double
    OFSQP::slope(int nob,int nobL,int neqn,int nparam,int feasb,
        struct _objective *ob,double *grdpsf,double *x,double *y,
        double fmax,double theta,int job,double *prev,int old)
    {
      int i;
      double slope1,rhs,rhog,grdftx,grdfty,diff,grpstx,grpsty;
      double tslope;

      tslope=-bgbnd;
      if (feasb && nob==0) tslope=0.e0;
      if (neqn==0 || !feasb) {
        grpstx=0.e0;
        grpsty=0.e0;
      } else {
        grpstx=scaprd(nparam,grdpsf,x);
        grpsty=scaprd(nparam,grdpsf,y);
      }
      for (i=1; i<=nob; i++) {
        if (old) slope1=prev[i]+scaprd(nparam,ob[i].grad,x);
        else slope1=ob[i].val+scaprd(nparam,ob[i].grad,x);
        tslope=std::max(tslope,slope1);
        if (nobL != nob) tslope=std::max(tslope,-slope1);
      }
      tslope=tslope-fmax-grpstx;
      if (job == 0) return tslope;
      rhs=theta*tslope+fmax;
      rhog=1.e0;
      for (i=1; i<=nob; i++) {
        grdftx=scaprd(nparam,ob[i].grad,x)-grpstx;
        grdfty=scaprd(nparam,ob[i].grad,y)-grpsty;
        diff=grdfty-grdftx;
        if (diff <= 0.e0) continue;
        rhog=std::min(rhog,(rhs-ob[i].val-grdftx)/diff);
        if (nobL != nob) rhog=std::min(rhog,-(rhs+ob[i].val+grdftx)/diff);
      }
      tslope=rhog;
      return tslope;
    }

  /************************************************************/
  /*  Determine whether index is in set                       */
  /************************************************************/


  bool OFSQP::element(int *set,int length,int index)
  {
    int i;
    bool temp;

    temp=false;
    for (i=1; i<=length; i++) {
      if (set[i]==0) break;
      if (set[i]==index) {
        temp=true;
        return temp;
      }
    }
    return temp;
  }




  /*************************************************************/
  /*     Memory allocation utilities for CFSQP                 */
  /*                                                           */
  /*     All vectors and matrices are intended to              */
  /*     be subscripted from 1 to n, NOT 0 to n-1.             */
  /*     The addreses returned assume this convention.         */
  /*************************************************************/

  /*************************************************************/
  /*     Create double precision vector                        */
  /*************************************************************/


  double *
    OFSQP::make_dv(int len)
    {
      double *v;

      if (!len) len=1;
      v=(double *)calloc(len,sizeof(double));
      if (!v) {
        fprintf(stderr,"Run-time error in make_dv");
        exit(1);
      }
      return --v;
    }

  /*************************************************************/
  /*     Create integer vector                                 */
  /*************************************************************/


  int *
    OFSQP::make_iv(int len)
    {
      int *v;

      if (!len) len=1;
      v=(int *)calloc(len,sizeof(int));
      if (!v) {
        fprintf(stderr,"Run-time error in make_iv");
        exit(1);
      }
      return --v;
    }

  /*************************************************************/
  /*     Create a double precision matrix                      */
  /*************************************************************/


  double **
    OFSQP::make_dm(int rows, int cols)
    {
      double **temp;
      int i;

      if (rows == 0) rows=1;
      if (cols == 0) cols=1;
      temp=(double **)calloc(rows,sizeof(double *));
      if (!temp) {
        fprintf(stderr,"Run-time error in make_dm");
        exit(1);
      }
      temp--;
      for (i=1; i<=rows; i++) {
        temp[i]=(double *)calloc(cols,sizeof(double));
        if (!temp[i]) {
          fprintf(stderr,"Run-time error in make_dm");
          exit(1);
        }
        temp[i]--;
      }
      return temp;
    }

  /*************************************************************/
  /*     Free a double precision vector                        */
  /*************************************************************/


  void
    OFSQP::free_dv(double *v)
    {
      free((char *) (v+1));
    }

  /*************************************************************/
  /*     Free an integer vector                                */
  /*************************************************************/


  void
    OFSQP::free_iv(int *v)
    {
      free((char *) (v+1));
    }

  /*************************************************************/
  /*     Free a double precision matrix                        */
  /*************************************************************/


  void
    OFSQP::free_dm(double **m,int rows)
    {
      int i;

      if (!rows) rows=1;
      for (i=1; i<=rows; i++) free((char *) (m[i]+1));
      free ((char *) (m+1));
    }

  /*************************************************************/
  /*     Converts matrix a into a form that can easily be      */
  /*     passed to a FORTRAN subroutine.                       */
  /*************************************************************/


  double *
    OFSQP::convert(double **a,int m,int n)
    {
      double *temp;
      int i,j;

      temp = make_dv(m*n);

      for (i=1; i<=n; i++)     /* loop thru columns */
        for (j=1; j<=m; j++)  /* loop thru row     */
          temp[(m*(i-1)+j)] = a[j][i];

      return temp;
    }







  /**********************************************************
   *                   E X A M P L E   1                    *
   **********************************************************/

  class Ex1Pb : public OFSQPProblem
  {

    void obj(int nparam, int j, double* x, double* fj, void* cd)
    {
      *fj=pow((x[0]+3.e0*x[1]+x[2]),2.e0)+4.e0*pow((x[0]-x[1]),2.e0);
      return;
    }

    void gradob(int nparam, int j, double* x, double* gradfj, void* cd)
    {
      double fa,fb;

      fa=2.e0*(x[0]+3.e0*x[1]+x[2]);
      fb=8.e0*(x[0]-x[1]);
      gradfj[0]=fa+fb;
      gradfj[1]=fa*3.e0-fb;
      gradfj[2]=fa;
      return;
    }

    void constr(int nparam, int j, double* x, double* gj, void* cd)
    {
      switch (j) {
        case 1:
          *gj=pow(x[0],3.e0)-6.e0*x[1]-4.e0*x[2]+3.e0;
          break;
        case 2:
          *gj=1.e0-x[0]-x[1]-x[2];
          break;
      }
      return;
    }

    void gradcn(int nparam, int j, double* x, double* gradgj, void* cd)
    {
      switch (j) {
        case 1:
          gradgj[0]=3.e0*x[0]*x[0];
          gradgj[1]=-6.e0;
          gradgj[2]=-4.e0;
          break;
        case 2:
          gradgj[0]=gradgj[1]=gradgj[2]=-1.e0;
          break;
      }
      return;
    }
  };

  void testOFSQP01()
  {
    Ex1Pb pb;

    OFSQP solver;
    int nparam, nf, nineq, neq, mode, iprint, miter, neqn, nineqn, ncsrl, ncsrn, nfsr, mesh_pts[1], inform;
    double bigbnd, eps, epsneq, udelta;
    double *x, *bl, *bu, *f, *g, *lambda;

    mode = 110;
    iprint = 1;
    miter = 500;
    bigbnd = 1.e10;
    eps = 1.e-8;
    epsneq = 0.e0;
    udelta = 0.e0;
    nparam = 3;
    nf = 1;
    neqn = 0;
    nineqn = 1;
    nineq = 1;
    neq = 1;
    ncsrl = ncsrn = nfsr = mesh_pts[0] = 0;
    bl = new double[nparam];
    bu = new double[nparam];
    x = new double[nparam];
    f = new double[nf+1];
    g = new double[nineq+neq+1];
    lambda = new double[nineq+neq+nf+nparam];

    bl[0] = bl[1] = bl[2] = 0.e0;
    bu[0] = bu[1] = bu[2] = bigbnd;

    x[0] = 0.1e0;
    x[1] = 0.7e0;
    x[2] = 0.2e0;

    solver.cfsqp(nparam, nf, nfsr, nineqn, nineq, neqn, neq, ncsrl, ncsrn, mesh_pts, mode, iprint, miter, &inform, bigbnd, eps,
        epsneq, udelta, bl, bu, x, f, g, lambda, pb, 0x0);

    delete bl;
    delete bu;
    delete x;
    delete f;
    delete g;
    delete lambda;
  }


  /**********************************************************
   *                   E X A M P L E   2                    *
   **********************************************************/

  class Ex2Pb : public OFSQPProblem
  {
    void obj(int nparam, int j, double* x, double* fj, void* cd)
    {
      double pi,theta;
      int i;
      pi=3.14159265358979e0;
      theta=pi*(8.5e0+j*0.5e0)/180.e0;
      *fj=0.e0;
      for (i=0; i<=5; i++)
        *fj=*fj+cos(2.e0*pi*x[i]*sin(theta));
      *fj=2.e0*(*fj+cos(2.e0*pi*3.5e0*sin(theta)))/15.e0+1.e0/15.e0;
      return; 
    }

    void constr(int nparam, int j, double* x, double* gj, void* cd)
    {
      double ss=0.425e0;
      switch (j) 
      {
        case 1: *gj=ss-x[0];        break;
        case 2: *gj=ss+x[0]-x[1];   break;
        case 3: *gj=ss+x[1]-x[2];   break;
        case 4: *gj=ss+x[2]-x[3];   break;
        case 5: *gj=ss+x[3]-x[4];   break;
        case 6: *gj=ss+x[4]-x[5];   break;
        case 7: *gj=ss+x[5]-3.5e0;  break;
      }
      return;
    }
  };

  void testOFSQP02a()
  {
    Ex2Pb pb;

    OFSQP solver;
    int nparam, nf, nineq, neq, mode, iprint, miter, neqn, nineqn, ncsrl, ncsrn, nfsr, mesh_pts[1], inform;
    double bigbnd, eps, epsneq, udelta;
    double *x, *bl, *bu, *f, *g, *lambda;

    mode = 111;
    iprint = 1;
    miter = 500;
    bigbnd = 1.e10;
    eps = 1.e-8;
    epsneq = 0.e0;
    udelta = 0.e0;
    nparam = 6;
    nf = 163;
    neqn = 0;
    nineqn = 0;
    nineq = 7;
    neq = 0;
    ncsrl = ncsrn = nfsr = mesh_pts[0] = 0;
    bl = new double[nparam];
    bu = new double[nparam];
    x = new double[nparam];
    f = new double[nf+1];
    g = new double[nineq+neq+1];
    lambda = new double[nineq+neq+nf+nparam];

    bl[0] = bl[1] = bl[2] = bl[3] = bl[4] = bl[5] = -bigbnd;
    bu[0] = bu[1] = bu[2] = bu[3] = bu[4] = bu[5] =  bigbnd;

    x[0] = 0.5e0;
    x[1] = 1.0e0;
    x[2] = 1.5e0;
    x[3] = 2.0e0;
    x[4] = 2.5e0;
    x[5] = 3.0e0;

    solver.cfsqp(nparam, nf, nfsr, nineqn, nineq, neqn, neq, ncsrl, ncsrn, mesh_pts, mode, iprint, miter, &inform, bigbnd, eps,
        epsneq, udelta, bl, bu, x, f, g, lambda, pb, 0x0);

    delete bl;
    delete bu;
    delete x;
    delete f;
    delete g;
    delete lambda;
  }


  void testOFSQP02b()
  {
    Ex2Pb pb;

    OFSQP solver;
    int nparam, nf, nineq, neq, mode, iprint, miter, neqn, nineqn, ncsrl, ncsrn, nfsr, mesh_pts[1], inform;
    double bigbnd, eps, epsneq, udelta;
    double *x, *bl, *bu, *f, *g, *lambda;

    mode = 111;
    iprint = 1;
    miter = 500;
    bigbnd = 1.e10;
    eps = 1.e-8;
    epsneq = 0.e0;
    udelta = 0.e0;
    nparam = 6;
    nf = 1;
    nfsr = 1;
    mesh_pts[0] = 163;
    neqn = 0;
    nineqn = 0;
    nineq = 7;
    neq = 0;
    ncsrl = ncsrn = 0;
    bl = new double[nparam];
    bu = new double[nparam];
    x = new double[nparam];
    f = new double[mesh_pts[0]+1];
    g = new double[nineq+neq+1];
    lambda = new double[nineq+neq+mesh_pts[0]+nparam+1];

    bl[0] = bl[1] = bl[2] = bl[3] = bl[4] = bl[5] = -bigbnd;
    bu[0] = bu[1] = bu[2] = bu[3] = bu[4] = bu[5] =  bigbnd;

    x[0] = 0.5e0;
    x[1] = 1.0e0;
    x[2] = 1.5e0;
    x[3] = 2.0e0;
    x[4] = 2.5e0;
    x[5] = 3.0e0;

    solver.cfsqp(nparam, nf, nfsr, nineqn, nineq, neqn, neq, ncsrl, ncsrn, mesh_pts, mode, iprint, miter, &inform, bigbnd, eps,
        epsneq, udelta, bl, bu, x, f, g, lambda, pb, 0x0);

    delete bl;
    delete bu;
    delete x;
    delete f;
    delete g;
    delete lambda;
  }


  /**********************************************************
   *                   E X A M P L E   3                    *
   **********************************************************/
  class Ex3Pb : public OFSQPProblem
  {
    void obj(int nparam, int j, double* x, double* fj, void* cd)
    {
      *fj=x[0]*x[3]*(x[0]+x[1]+x[2])+x[2];
      return;
    }

    void gradob(int nparam, int j, double* x, double* gradfj, void* cd)
    {
      gradfj[0]=x[3]*(x[0]+x[1]+x[2])+x[0]*x[3];
      gradfj[1]=x[0]*x[3];
      gradfj[2]=x[0]*x[3]+1.e0;
      gradfj[3]=x[0]*(x[0]+x[1]+x[2]);
      return;
    }

    void constr(int nparam, int j, double* x, double* gj, void* cd)
    {
      switch (j) {
        case 1:
          *gj=25.e0-x[0]*x[1]*x[2]*x[3];
          break;
        case 2:
          *gj=x[0]*x[0]+x[1]*x[1]+x[2]*x[2]+x[3]*x[3]-40.e0;
          break;
      }
      return;
    }

    void gradcn(int nparam, int j, double* x, double* gradgj, void* cd)
    {
      switch (j) {
        case 1:
          gradgj[0]=-x[1]*x[2]*x[3];
          gradgj[1]=-x[0]*x[2]*x[3];
          gradgj[2]=-x[0]*x[1]*x[3];
          gradgj[3]=-x[0]*x[1]*x[2];
          break;
        case 2:
          gradgj[0]=2.e0*x[0];
          gradgj[1]=2.e0*x[1];
          gradgj[2]=2.e0*x[2];
          gradgj[3]=2.e0*x[3];
          break;
      }
      return;
    }
  };

  void testOFSQP03()
  {
    Ex3Pb pb;

    OFSQP solver;
    int nparam, nf, nineq, neq, mode, iprint, miter, neqn, nineqn, ncsrl, ncsrn, nfsr, mesh_pts[1], inform;
    double bigbnd, eps, epsneq, udelta;
    double *x, *bl, *bu, *f, *g, *lambda;

    mode = 100;
    iprint = 1;
    miter = 500;
    bigbnd = 1.e10;
    eps = 1.e-7;
    epsneq = 7.e-6;
    udelta = 0.e0;
    nparam = 4;
    nf = 1;
    neqn = 1;
    nineqn = 1;
    nineq = 1;
    neq = 1;
    ncsrl = ncsrn = nfsr = mesh_pts[0] = 0;
    bl = new double[nparam];
    bu = new double[nparam];
    x = new double[nparam];
    f = new double[nf+1];
    g = new double[nineq+neq+1];
    lambda = new double[nineq+neq+nf+nparam+1];

    bl[0] = bl[1] = bl[2] = bl[3] = 1.e0;
    bu[0] = bu[1] = bu[2] = bu[3] = 5.e0;

    x[0] = 1.e0;
    x[1] = 5.e0;
    x[2] = 5.e0;
    x[3] = 1.e0;

    solver.cfsqp(nparam, nf, nfsr, nineqn, nineq, neqn, neq, ncsrl, ncsrn, mesh_pts, mode, iprint, miter, &inform, bigbnd, eps,
        epsneq, udelta, bl, bu, x, f, g, lambda, pb, 0x0);

    delete bl;
    delete bu;
    delete x;
    delete f;
    delete g;
    delete lambda;
  }


  /**********************************************************
   *                   E X A M P L E   4                    *
   **********************************************************/
  class Ex4Pb : public OFSQPProblem
  {
    void obj(int nparam, int j, double* x, double* fj, void* cd)
    {
      *fj=x[9];
      return;
    }

    void gradob(int nparam, int j, double* x, double* gradfj, void* cd)
    {
      gradfj[0]=0.e0;
      gradfj[1]=0.e0;
      gradfj[2]=0.e0;
      gradfj[3]=0.e0;
      gradfj[4]=0.e0;
      gradfj[5]=0.e0;
      gradfj[6]=0.e0;
      gradfj[7]=0.e0;
      gradfj[8]=0.e0;
      gradfj[9]=1.e0;
      return;
    }

    void constr(int nparam, int j, double* x, double* gj, void* cd)
    {
      double t,z,s1,s2;
      int k,r;

      r=100;
      s1=s2=0.e0;
      if (j<=r) 
        t=3.14159265358979312e0*(j-1)*0.025e0;
      else 
      {
        if (j<=2*r) 
          t=3.14159265358979312e0*(j-1-r)*0.025e0;
        else 
          t=3.14159265358979312e0*(1.2e0+(j-1-2*r)*0.2e0)*0.25e0;
      }
      for (k=1; k<=9; k++) 
      {
        s1=s1+x[k-1]*cos(k*t);
        s2=s2+x[k-1]*sin(k*t);
      }
      z=s1*s1 + s2*s2;
      if (j<=r) 
        *gj=(1.e0-x[9])*(1.e0-x[9])-z;
      else 
      {
        if (j<=2*r) 
          *gj=z-(1.e0+x[9])*(1.e0+x[9]);
        else 
          *gj=z-x[9]*x[9];
      }
      return;
    }
  };

  void testOFSQP04()
  {
    Ex4Pb pb;

    OFSQP solver;
    int nparam, nf, nineq, neq, mode, iprint, miter, neqn, nineqn, ncsrl, ncsrn, nfsr, mesh_pts[3], numc, inform;
    double bigbnd, eps, epsneq, udelta;
    double *x, *bl, *bu, *f, *g, *lambda;

    int r = 100;

    mode = 100;
    iprint = 1;
    miter = 500;
    bigbnd = 1.e10;
    eps = 1.e-7;
    epsneq = 0.e0;
    udelta = 0.e0;
    nparam = 10;
    nf = 1;
    neqn = 0;
    nineqn = nineq = ncsrn = 3;
    ncsrl = 0;
    mesh_pts[0] = mesh_pts[1] = r; 
    mesh_pts[2] = 3*r/2;
    neq = nfsr = 0;
    numc = (int)3.5*r;
    bl = new double[2*nparam];
    bu = new double[2*nparam];
    x = new double[2*nparam];
    f = new double[2*(nf+1)];
    g = new double[2*(numc+1)];
    lambda = new double[2*(numc+nf+nparam+1)];

    bl[0] = bl[1] = bl[2] = bl[3] = bl[4] = bl[5] = bl[6] = bl[7] = bl[8] = bl[9] = -bigbnd;
    bu[0] = bu[1] = bu[2] = bu[3] = bu[4] = bu[5] = bu[6] = bu[7] = bu[8] = bu[9] =  bigbnd;

    x[0] = x[1] = x[2] = x[3] = x[4] = x[5] = x[6] = x[7] = x[8] = 0.1e0;
    x[9] = 1.e0;

    solver.cfsqp(nparam, nf, nfsr, nineqn, nineq, neqn, neq, ncsrl, ncsrn, mesh_pts, mode, iprint, miter, &inform, bigbnd, eps,
        epsneq, udelta, bl, bu, x, f, g, lambda, pb, 0x0);

    delete bl;
    delete bu;
    delete x;
    delete f;
    delete g;
    delete lambda;
  }

  /**********************************************************
   *                   E X A M P L E   5                    *
   **********************************************************/
  class Ex5Pb : public OFSQPProblem
  {
    void obj(int nparam, int j, double* x, double* fj, void* cd)
    {
      *fj=(1.e0/3.e0)*pow(x[0],2.e0)+pow(x[1],2.e0)+0.5e0*x[0];
      return;
    }

    void gradob(int nparam, int j, double* x, double* gradfj, void* cd)
    {
      gradfj[0]=(2.e0/3.e0)*x[0]+0.5e0;
      gradfj[1]=2.e0*x[1];
      return;
    }

    void constr(int nparam, int j, double *x, double* gj, void* cd)
    {
      double y;

      y=(j-1)/100.e0;
      *gj=pow((1.e0-pow(y*x[0],2.e0)),2.e0)-x[0]*y*y-pow(x[1],2.e0)
        +x[1];
      return;
    }

    void gradcn(int nparam, int j, double* x, double* gradgj, void* cd)
    {
      double y;

      y=(j-1)/100.e0;
      gradgj[0]=-4.e0*(1.e0-pow(y*x[0],2.e0))*y*y*x[0]-y*y;
      gradgj[1]=-2.e0*x[1]+1.e0;
      return;
    }
  };

  void testOFSQP05()
  {
    Ex5Pb pb;

    OFSQP solver;
    int nparam, nf, nineq, neq, mode, iprint, miter, neqn, nineqn, ncsrl, ncsrn, nfsr, mesh_pts[1], inform;
    double bigbnd, eps, epsneq, udelta;
    double *x, *bl, *bu, *f, *g, *lambda;

    mode = 110;
    iprint = 1;
    miter = 500;
    bigbnd = 1.e10;
    eps = 1.e-4;
    epsneq = 0.e0;
    udelta = 0.e0;
    nparam = 2;
    nf = 1;
    neqn = 0;
    nineqn = nineq = 1;
    ncsrn = 1;
    ncsrl = 0;
    mesh_pts[0] = 101;
    neq = nfsr = 0;
    bl = new double[nparam];
    bu = new double[nparam];
    x = new double[nparam];
    f = new double[nf+1];
    g = new double[nineq + (mesh_pts[0]-1)*(ncsrl+ncsrn) + neq+1];
    lambda = new double[nineq + (mesh_pts[0]-1)*(ncsrl+ncsrn) + neq + nf + nparam];

    bl[0] = bl[1] = -bigbnd;
    bu[0] = bu[1] =  bigbnd;

    x[0] = -1.e0;
    x[1] = -2.e0;

    solver.cfsqp(nparam, nf, nfsr, nineqn, nineq, neqn, neq, ncsrl, ncsrn, mesh_pts, mode, iprint, miter, &inform, bigbnd, eps,
        epsneq, udelta, bl, bu, x, f, g, lambda, pb, 0x0);

    delete bl;
    delete bu;
    delete x;
    delete f;
    delete g;
    delete lambda;
  }

}//ocra

// cmake:sourcegroup=Solvers
