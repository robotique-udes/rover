#ifndef COMPILE_TIME_ARRAY
#define COMPILE_TIME_ARRAY

#include <array>
#include <algorithm>
#include <utility>
#include <cstddef>
#include <type_traits>

/**
 * @brief Constexpr array which can be evaluated at compile time if declared constexpr. Good for constant list without impacting
 * runtime performance
 *
 * @tparam DataT
 * @tparam NB_ELEM
 */
template<typename DataT, std::size_t NB_ELEM>
class CompileTimeArray
{
  public:
    constexpr CompileTimeArray(const std::array<DataT, NB_ELEM>& data_):
        _data(data_)
    {
    }

    template<typename... Args,
             typename = std::enable_if_t<sizeof...(Args) == NB_ELEM && (std::is_convertible_v<Args, DataT> && ...)>>
    constexpr CompileTimeArray(Args&&... elems_):
        _data{std::forward<Args>(elems_)...}
    {
    }

    constexpr bool contains(const DataT& value_) const
    {
        for (std::size_t i = 0; i < NB_ELEM; ++i)
        {
            if (_data[i] == value_)
            {
                return true;
            }
        }
        return false;
    }

    constexpr const DataT& operator[](std::size_t index_) const
    {
        return _data[index_];
    }

    constexpr std::size_t size() const
    {
        return NB_ELEM;
    }

    constexpr const DataT* data(void) const
    {
        return _data.data();
    }

    constexpr auto begin(void) const
    {
        return _data.begin();
    }

    constexpr auto end(void) const
    {
        return _data.end();
    }

  private:
    std::array<DataT, NB_ELEM> _data;
};

#endif  // COMPILE_TIME_ARRAY
