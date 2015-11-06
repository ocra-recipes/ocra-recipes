#ifndef _OCRABASE_BUFFER_H_
#define _OCRABASE_BUFFER_H_

#ifdef WIN32
# pragma once
#endif

namespace ocra
{
  /** This class acts as a memory pool for objects of type \a T. Upon request it returns a pointer to an array of T of
    * size specified by the caller, provided it owns enough memory for that.
    * It allocates contiguous memory blocks and ensures memory release when resizing or dying. Thus, you need not free 
    * the memory obtained via this class.
    *
    * \warning Don't trust other classes. Don't give them the buffers you obtain here: resize() invalidates them all.
    */
  template<class T>
  class Buffer
  {
  public:
    typedef T value_type;
    typedef T* ptr_type;

  private:
    /** This class is non copyable */
    //@{
    Buffer(const Buffer&);
    Buffer& operator= (const Buffer&);
    //@}

  public:
    /** Build an instance of Buffer with a memory buffer for \a size elements of type T */
    Buffer(size_t size): buf_(new value_type[size]), size_(size), index_(0) {}
    ~Buffer() { delete[] buf_; }

    /** Resize the memory buffer to a size greater or equal to the parameter \a size. Wether there is really a buffer
      * resizing or not, the previously returned buffer need to be considered as invalid, since calls to allocate
      * subsequent to a resize will return buffers starting from the beginning of the memory pool again.
      */ 
    void resize(size_t size)
    {
      index_ = 0;
      
      if(size <= size_)
        return;

      size_t nsize = size_ + 1;
      while(nsize <= size)
        nsize *= 2;

      delete[] buf_;
      buf_ = new value_type[nsize];
      size_ = nsize;
    }

    /** Tell the instance to give memory from the beginning of the pool again. All previously given buffers should be
      * considered as invalid
      */
    void reset()
    {
      index_ = 0;
    }

    /** Return the pointer on a free continuous memory block of size \a n
      *
      * \throw a runtime_error in case there is not enough memory reserved to fullfill this query.
      */
    ptr_type allocate(size_t n)
    {
      if((index_ + n) > size_)
        throw std::runtime_error("[ocra::Buffer::allocate] Insufficient memory, please resize before allocation");
      ptr_type result =  buf_ + index_;
      index_ += n;
      return result;
    }

    /** Return the size of the reserved memory */
    size_t size() const
    {
      return size_;
    }

    /** Return the size of the reserved memory still free */
    size_t freeSize() const
    {
      return size_ - index_;
    }

  private:
    ptr_type buf_;  //< the memory pool
    size_t size_;   //< its size
    size_t index_;  //< the index of the first free element in the pool
  };
}

#endif //_OCRABASE_BUFFER_H_

// cmake:sourcegroup=Utils
