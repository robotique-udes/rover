#ifndef __KEYBINDING_SCIENCE__HPP__
#define __KEYBINDING_SCIENCE__HPP__

#include <rover_msgs/msg/joy.hpp>
#include <rover_lib2/helpers/constants.hpp>

namespace KEYBINDINGS
{
    constexpr Constants::Keybinds::eJoyInput DEADMAN_SWITCH = Constants::Keybinds::eJoyInput::L1;

    constexpr Constants::Keybinds::eJoyInput LINEAR_ACT_UP = Constants::Keybinds::eJoyInput::A;
    constexpr Constants::Keybinds::eJoyInput LINEAR_ACT_DOWN = Constants::Keybinds::eJoyInput::B;
    constexpr Constants::Keybinds::eJoyInput EXCAVATOR = Constants::Keybinds::eJoyInput::L2;

}  // namespace KEYBINDINGS

#endif
