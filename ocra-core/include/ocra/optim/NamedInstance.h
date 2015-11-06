#ifndef _OCRA_NAMED_INSTANCE_H_
#define _OCRA_NAMED_INSTANCE_H_

#include <string>

namespace ocra
{
  /** A trivial class with a single string member.
    * 
    * Within ocra, this class is meant to be used as a \a virtual base class to force some objects (functions, solvers)
    * to have their own name.
    */
  class NamedInstance
  {
  public:
    NamedInstance(const std::string& name);
    const std::string& getName() const;

    virtual ~NamedInstance();

  private:
    std::string name_;
  };
}

#endif //_OCRA_NAMED_INSTANCE_H_

// cmake:sourcegroup=Utils
