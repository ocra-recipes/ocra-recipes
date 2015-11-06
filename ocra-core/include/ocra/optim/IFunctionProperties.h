/** @file IFunction.h
 *  @brief Declaration file of the IFunction interface.
 *
 *         Copyright (C) 2010 CEA/DRT/LIST/DIASI/LSI
 *
 *  @author Escande Adrien
 *  @date 10.05.31
 */



#ifndef _OCRABASE_IFUNCTION_PROPERTIES_H_
#define _OCRABASE_IFUNCTION_PROPERTIES_H_

#ifdef WIN32
# pragma once
#endif

#include <string>
#include <vector>

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** Enumerates the different continuity level of a function */
  enum eFunctionContinuity
  {
    CONTINUITY_C0 = 0,
    CONTINUITY_C1,
    CONTINUITY_C2,
    CONTINUITY_C3,
    CONTINUITY_C4,
    CONTINUITY_CINF = 0xfff-1,
    CONTINUITY_UNKNOWN
  };

  /** Enumerates the possible linearity properties of a function */
  enum eFunctionLinearity
  {
    LINEARITY_CONSTANT,               //< \f$ f(x)=a \f$, \f$ a \in R^m \f$
    LINEARITY_LINEAR,                 //< \f$ f(x)=Ax+b \f$, \f$ A \in R^{m \times n} \f$, \f$ b \in R^m \f$
    LINEARITY_QUADRATIC,              //< \f$ f_i(x)=x^T P_i x + q_i^T x + r_i \f$, \f$ P_i \in S^n \f$, \f$ q_i \in R^n \f$, \f$ r_i \in R \f$
    LINEARITY_LINEAR_FRACTIONAL,      //< \f$ f_i(x)=\frac{a_i^T x + b_i}{c_i^T x + d_i} \f$, \f$ a_i, c_i \in R^n \f$, \f$ b_i, d_i \in R \f$, \f$ c_i^T x + d_i>0 \f$
    LINEARITY_UNDEFINED
  };

  /** Enumerates the possible convexity properties of a function */
  enum eFunctionConvexity
  {
    CONVEXITY_CONVEX,                 //< \f$ D_f \f$ is convex and \f$ f(tx+(1-u)y \leq tf(x)+(1-u)f(y) \f$, with \f$ u \in \left[0,1\right] \f$
    CONVEXITY_STRICTLY_CONVEX,        //< \f$ D_f \f$ is convex and \f$ f(tx+(1-u)y < tf(x)+(1-u)f(y) \f$, with \f$ u \in \left[0,1\right] \f$
    CONVEXITY_CONCAVE,                //< \f$ -f \f$ is convex
    CONVEXITY_STRICTLY_CONCAVE,       //< \f$ -f \f$ is stricly convex
    CONVEXITY_CONVEX_AND_CONCAVE,     //< \f$ f \f$ is linear
    CONVEXITY_LOG_CONVEX,             //< \f$ f>0 \f$ and \f$ \log f \f$ is convex
    CONVEXITY_LOG_CONCAVE,            //< \f$ f>0 \f$ and \f$ \log f \f$ is concave
    CONVEXITY_QUASICONVEX,            //< \f$ D_f \f$ is convex and all \f$ S_{\alpha} = {x \in D_f | f(x) \leq \alpha} \f$ are convex
    CONVEXITY_QUASICONCAVE,           //< \f$ -f \f$ is quasiconvex
    CONVEXITY_QUASILINEAR,            //< \f$ f \f$ is quasiconvex and quasiconcave
    CONVEXITY_UNDEFINED
  };


  /** This class groups the properties a function can have. The usual properties used by an optimization software are
    * described by the use of the enumerations eFunctionContinuity, eFunctionLinearity and eFunctionContinuity.
    * Furthermore two flags are used to indicate wether the function has an explicit time-dependancy and if it is the
    * case if this time dependancy is separable from the variable dependancy: \f$ f(x,t) = g(x)+h(t)\f$
    * Additionnal properties can be added with the help of strings.
    */
  class IFunctionProperties
  {
  protected:
    /**
    * \brief IFunctionProperties Constructor.
    *
    * \param[in] linearity  Linearity property of the function. 
    * \param[in] convexity  Convexity property of the function. 
    * \param[in] continuity  Continuity property of the function. 
    *
    * \pre (continuity==CONTINUITY_UNKNOWN) || (continuity>0 && continuity<=CONTINUITY_CINF)
    **/
    IFunctionProperties(eFunctionLinearity linearity = LINEARITY_UNDEFINED, 
              eFunctionConvexity convexity = CONVEXITY_UNDEFINED, int continuity = CONTINUITY_UNKNOWN,
              bool explicitlyTimeDependant=false, bool separableTimeDependancy = true);

  private:
    /** \internal private constructors, never to be used*/
    //@{
    IFunctionProperties(const IFunctionProperties&);
    IFunctionProperties& operator= (const IFunctionProperties&);
    //@}

  public:

    /** Accessors */
    //@{
    eFunctionLinearity  getType(void) const;
    eFunctionConvexity  getConvexityProperty(void) const;
    int                 getContinuityProperty(void) const;
    const std::string&  getProperty(int i) const;
    int                 getNumberOfProperties(void) const;
    bool                hasProperty(const std::string& functionProperty) const;
    bool                isExplicitlyTimeDependant(void) const;
    //@}

    /** Return true if f(x,t) can be written g(x)+h(t). It includes the case when there is no explicit time dependancy
      * since in that case g(x) = f(x,t) and h(t)=0.
      */
    bool                hasSeparableTimeDependancy(void) const;



  protected:
    /** change the linearity property of the function to \a newType*/
    void  changeType(eFunctionLinearity newType);

    /** change the convexity property of the function to \a newProperty*/
    void  changeConvexityProperty(eFunctionConvexity newProperty);

    /** change the continuity property of the function  to \a newProperty
      *
      * \pre (newProperty==CONTINUITY_UNKNOWN) || (newProperty>0 && newProperty<=CONTINUITY_CINF)
      */
    void  changeContinuityProperty(int newProperty);

    /** add a property to the function
      * If the property already exist for the function, it will not be added again
      */
    void  addProperty(const std::string& functionProperty);

    /** remove a property from the function
      * If the property doesn't exist for this function, nothing happens.
      *
      * \post functionProperty doesn't appear anymore in the function properties
      */
    void  removeProperty(const std::string& functionProperty);

    /** set the explicit time dependancy property to \a b*/
    void changeExplicitTimeDependancy(bool b);

    /** set the separable time dependancy property to \a b
     * However, hasSeparableTimeDependancy will always return true, if there is no explicit time dependancy.
     */
    void changeSeparableTimeDependancy(bool b);
    


  private:
    eFunctionLinearity        _type;                    //< linearity
    eFunctionConvexity        _convexityProperty;       //< convexity
    int                       _continuityProperty;      //< continuity
    std::vector<std::string>  _properties;              //< for additional properties aside from continuity, convexity and linearity
    bool                      _timeDependant;           //< explicit time dependancy
    bool                      _separableTimeDependancy; //< true if f can be written g(x)+h(t)
  };
}

#endif //_OCRABASE_IFUNCTION_PROPERTIES_H_

// cmake:sourcegroup=Function
