#ifndef OFSQP_H
#define OFSQP_H
#pragma once


#include <stdio.h>
#include <math.h>
#include <stdlib.h>


namespace ocra
{
  namespace fsqpDetails
  {
    struct Info
    {
      int nnineq, M, ncallg, ncallf, mode, modec;
      int tot_actf_sip,tot_actg_sip,nfsip,ncsipl,ncsipn; /* SIP */
    };

    struct Prnt
    {
      int iprint,info,ipd,iter,initvl,iter_mod;
      FILE *io;
    };

    struct Grd
    {
      double epsmac,rteps,udelta,valnom;
    };

    struct Log
    {
      bool rhol_is1,get_ne_mult,first,local,update,d0_is0,dlfeas;
    };
  }

  class OFSQPProblem
  {
  public:
    OFSQPProblem();

    virtual void obj(int nparam, int j, double* x, double* fj, void* cd)= 0;
    virtual void constr(int nparam, int j, double* x, double* gj, void* cd) = 0;
		virtual void gradob(int nparam, int j, double* x, double* gradfj, void* cd);
		virtual void gradcn(int nparam, int j, double* x, double* gradgj, void* cd);

    void invalidateX();
    void validateX();
    bool isXNew();

    void setFSQPstruct(fsqpDetails::Grd& grd, fsqpDetails::Prnt& prnt);
    void resetFSQPstruct();

  private:
    bool x_is_new;
    fsqpDetails::Grd*   glob_grd;
    fsqpDetails::Prnt*  glob_prnt;
  };

/**************************************************************/
/*     Gradients - Finite Difference                          */
/**************************************************************/

class OFSQP
{	

public:
	OFSQP(void);
	~OFSQP(void);

public:
	bool isXNew(void);
	void resetNewX(void);


private:
  enum eFnType
  {
	  NONE = 0,
    OBJECT = 1,
    CONSTR = 2
  };

	struct _objective {
		double val;
		double *grad;
		double mult;
		double mult_L; /* mode A=1 */
		bool act_sip;   /* SIP      */
	};

	struct _constraint {
		double val;
		double *grad;
		double mult;
		bool act_sip;   /* SIP      */
		bool d1bind;    /* SR constraints  */
	};

	struct _parameter {
		double *x;
		double *bl;
		double *bu;
		double *mult;
		void *cd;      /* Client data pointer */
	};

	struct _violation {    /* SIP      */
		int type;
		int index;
	};

private:
  fsqpDetails::Info glob_info;
  fsqpDetails::Prnt glob_prnt;
  fsqpDetails::Grd  glob_grd;
  fsqpDetails::Log  glob_log;


public :
	/**************************************************************/
	/*     Prototype for CFSQP -   	                              */
	/**************************************************************/

	void
		cfsqp(int nparam,int nf,int nfsr,int nineqn,int nineq,int neqn,
		int neq,int ncsrl,int ncsrn,int *mesh_pts,
		int mode,int iprint,int miter,int *inform,double bigbnd,
		double eps,double epseqn,double udelta,double *bl,double *bu,
		double *x,double *f,double *g,double *lambda,
		OFSQPProblem& problem, void *cd);


private:
	/****jrl : change form char[] to char* ****/
	const char* cfsqp_version;
	double  bgbnd,tolfea;
	int  maxit;

  OFSQPProblem* pb;

	/* Declare and initialize user-accessible flag indicating    */
	/* whether x sent to user functions has been changed within  */
	/* CFSQP.				 		     */
private:
	//bool x_is_new;

private:
	/* Declare and initialize user-accessible stopping criterion */
	double objeps;
	double objrep;
	double gLgeps;
	int nstop;

	/* Workspace                                                     */
	int     *iw;
	double  *w;
	int     lenw, leniw;

	/***************************************************************/
	/*     Memory Utilities                                        */
	/***************************************************************/
private:
	static int      *make_iv(int);
	static double   *make_dv(int);
	static double   **make_dm(int, int);
	static void     free_iv(int *);
	static void     free_dv(double *);
	static void     free_dm(double **, int);
	static double   *convert(double **, int, int);

	/***************************************************************/
	/*     Utility Subroutines                                     */
	/***************************************************************/

/*	int
		ql0001_(int *,int *,int *,int *,int *,int *,double *,double *,
		double *,double *,double *,double *,double *,double *,
		int *,int *,int *,double *,int *,int *,int *,double *);*/
	static void     diagnl(int, double, double **);
	void			error(char string[],int *);
	void
		estlam(int,int,int *,double,double **,double *,double *,double *,
	struct _constraint *,double *,double *,double *,double *);
	static double   *colvec(double **,int,int);
	static double   scaprd(int,double *,double *);
	static double   small(void);
	static bool     fuscmp(double,double);
	static int      indexs(int,int);
	static void     matrcp(int,double **,int,double **);
	static void     matrvc(int,int,double **,double *,double *);
	static void     nullvc(int,double *);
	void
		resign(int,int,double *,double *,double *,struct _constraint *,
		double *,int,int);
	static void     sbout1(FILE *,int,char *,double,double *,int,int);
	static void     sbout2(FILE *,int,int,char *,char *,double *);
	static void     shift(int,int,int *);
	double
		slope(int,int,int,int,int,struct _objective *,double *,double *,
		double *,double,double,int,double *,int);
	static bool     element(int *,int,int);


	/**************************************************************/
	/*     Main routines for optimization -                       */
	/**************************************************************/

	void
		cfsqp1(int,int,int,int,int,int,int,int,int,int,int *,int,
		int,int,int,double,double,int *,int *,struct _parameter *,
	struct _constraint *,struct _objective *,double *);
	void
		check(int,int,int,bool *,int,int,int,int,int,int,int,int *,double,
		double,struct _parameter *);
	void
		initpt(int,int,int,int,int,int,int,struct _parameter *,struct _constraint *);
	void
		dir(int,int,int,int,int,int,int,int,int,int,int,int,double *,
		double,double,double *,double *,double,double *,double *,int *,
		int *,int *,int *,int *,int *,struct _parameter *,double *,
		double *,struct _constraint *,struct _objective *,double *,
		double *,double *,double *,double *,double *,double **,double *,
		double *,double *,double *,double **,double **,double *,
		double *,struct _violation *);
	void
		step1(int,int,int,int,int,int,int,int,int,int,int,int *,int *,int *,
		int *,int *,int *,int *,int *,int,double,struct _objective *,
		double *,double *,double *,double *,double *,double *,double *,
		double *,double *,double *,double *,struct _constraint *,
		double *,double *,struct _violation *viol,void *);
	void
		hessian(int,int,int,int,int,int,int,int,int,int,int,int *,int,
		double *,struct _parameter *,struct _objective *,
		double,double *,double *,double *,double *,double *,
	struct _constraint *,double *,int *,int *,double *,
		double *,double *,double **,double *,double,int *,
		double *,double *,double **,double *,double *,struct _violation *);
	void
		out(int,int,int,int,int,int,int,int,int,int,int,int *,double *,
	struct _constraint *,struct _objective *,double,
		double,double,double,double,int);
	void
		update_omega(int,int,int,int *,int,int,int,int,double,double,
	struct _constraint *,struct _objective *,double *,
	struct _violation *,void *,int);

	static void
		dealloc(int,int,double *,int *,int *,struct _constraint *cs,
	struct _parameter *);

	static void
		dealloc1(int,int,double **,double **,double **,double *,double *,
		double *,double *,double *,double *,double *,double *,
		double *,double *,double *,double *,int *,int *,int *);

	void
		dqp(int,int,int,int,int,int,int,int,int,int,int,int,int,
		int,int,int *,struct _parameter *,double *,int,
	struct _objective *,double,double *,struct _constraint *,
		double **,double *,double *,double *,double *,
		double **,double **,double *,double,int);
	void
		di1(int,int,int,int,int,int,int,int,int,int,int,int,int *,
		int,struct _parameter *,double *,struct _objective *,
		double,double *,struct _constraint *,double *,
		double *,double *,double *,double **,double *,double);


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

	static void fool(double, double, double *);

};

void testOFSQP01();
void testOFSQP02a();
void testOFSQP02b();
void testOFSQP03();
void testOFSQP04();
void testOFSQP05();

}//ocra


#endif	//OFSQP_H

// cmake:sourcegroup=Solvers
