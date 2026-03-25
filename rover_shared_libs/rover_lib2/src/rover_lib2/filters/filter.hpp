#ifndef ROVER_LIB2_FILTERS_FILTER_HPP
#define ROVER_LIB2_FILTERS_FILTER_HPP

#include "rover_lib2/rover_object.hpp"

#include <concepts>
#include <utility>

namespace Filters
{

    template<typename ImplT>
    concept Filter = requires(ImplT impl_) {
        // clang-format off
        { impl_.addValue(float{} /* newValue_ */) } -> std::same_as<float>; 
        { std::as_const(impl_).getFilteredValue() } -> std::same_as<float>;
        { impl_.reset(float{} /* fillValue_ */) } -> std::same_as<void>;
        // clang-format on
    };

}  // namespace Filters

#endif  // ROVER_LIB2_FILTERS_FILTER_HPP
