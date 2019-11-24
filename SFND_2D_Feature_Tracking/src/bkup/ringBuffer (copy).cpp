#include <iostream>
#include <stddef.h>
//Foward Declaration of iterator
template <class T, int N> class RingBufferIterator;
//Ring Buffer
template<class T, int N>
class RingBuffer
{
public:
  friend class RingBufferIterator<T,N>;
  typedef RingBufferIterator<T,N> iterator;

  explicit RingBuffer()
    :ring_buffer_(nullptr)
    ,buffer_size_(0)
    ,begin_(0)
    {  ring_buffer_ = new T[N];
       capacity_ = N;
     };

  size_t size(){return buffer_size_;}
  iterator begin(){return iterator(*this, 0);}
  iterator end(){return iterator(*this, buffer_size_);}

  void push_back(const T& item)
  {
    *(end()) = item;
    if(buffer_size_ < capacity_) ++buffer_size_;
    else begin_ = (begin_ + 1) % capacity_;
  }

private:
  T* ring_buffer_;
  size_t buffer_size_;
  size_t capacity_;
  size_t begin_;
};

//Iterator declaration
template<class T, int N>
class RingBufferIterator{
  private:
    RingBuffer<T, N>& rb_;
    size_t offset_;
  public:
    RingBufferIterator(RingBuffer<T,N>& rb, size_t offset)
    :rb_(rb),
    offset_ (offset)
    {
    }

    T& operator*()
    { return rb_.ring_buffer_[(rb_.begin_ + offset_)%rb_.capacity_]; }

    T *operator->()
    {return &(operator*());}

    bool operator==(const RingBufferIterator& it)
    {
      return &(this->rb_) == &(it.rb_) && (this->offset_ == it.offset_);
    }

    bool operator!=(const RingBufferIterator& it)
    {
      return not (*this == it);
    }

    RingBufferIterator& operator+ ( ptrdiff_t delta )
    {
      offset_ += delta;
      return * this;
    }

    RingBufferIterator& operator++ ()
    {
      offset_ += 1;
      return * this;
    }

    RingBufferIterator& operator- ( ptrdiff_t delta )
    {
      offset_ -= delta;
      return * this;
    }
    RingBufferIterator& operator-- ()
    {
      offset_ -= 1;
      return * this;
    }
};
