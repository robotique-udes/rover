#ifndef ROVER_OBJECT_HPP
#define ROVER_OBJECT_HPP

template<typename Impl_T>
class RoverObject
{
  private:
    RoverObject() = default;
    friend Impl_T;

  public:
    void init(void)
    {
        static_cast<Impl_T*>(this)->_init();
    }

    void update(void)
    {
        static_cast<Impl_T*>(this)->_update();
    }
};

#endif  // ROVER_OBJECT_T_HPP
