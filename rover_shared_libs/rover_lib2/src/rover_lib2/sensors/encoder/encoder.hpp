#ifndef ROVER_LIB2_SENSORS_ENCODER_ENCODER_HPP
#define ROVER_LIB2_SENSORS_ENCODER_ENCODER_HPP

#include <rover_lib2/rover_object.hpp>
#include <rover_lib2/helpers/macros.hpp>

#include <concepts>

namespace Encoders
{

    template<typename ImplT>
    concept Encoder = RoverObject<ImplT> && requires(ImplT impl_)
    {
        // clang-format off
        { impl_.dataIsValid() } -> std::same_as<bool>;

        { impl_.getPosition() } -> std::same_as<float>;

        { impl_.getSpeed() } -> std::same_as<float>;

        { impl_.calib(float{}) } -> std::same_as<void>;
        // clang-format on
    };

    class None
    {
      public:
        void init()
        {
            ASSERT_MSG("Interface");
        }

        void update()
        {
            ASSERT_MSG("Interface");
        }

        bool dataIsValid()
        {
            ASSERT_MSG("Interface");
            return false;
        }

        float getPosition()
        {
            ASSERT_MSG("Interface");
            return 0.0F;
        }

        float getSpeed()
        {
            ASSERT_MSG("Interface");
            return 0.0F;
        }

        void calib(float /*offset_*/)
        {
            ASSERT_MSG("Interface");
        }

        void setReversed(bool /*reverse_*/)
        {
            ASSERT_MSG("Interface");
        }

        VALIDATE_CONCEPT(Encoder, None);
    };

}  // namespace Encoders
#endif  // ROVER_LIB2_SENSORS_ENCODER_ENCODER_HPP
