#ifndef __KEYBINDING_SCIENCE__HPP__
#define __KEYBINDING_SCIENCE__HPP__

#include <rover_msgs/msg/joy.hpp>
#include <rover_lib2/helpers/constants.hpp>

namespace KEYBINDINGS
{
    constexpr Constants::Keybinds::eJoyInput DEADMAN_SWITCH = Constants::Keybinds::eJoyInput::L1;

    constexpr Constants::Keybinds::eJoyInput LINEAR_ACT_UP = Constants::Keybinds::eJoyInput::CROSS_UP;
    constexpr Constants::Keybinds::eJoyInput LINEAR_ACT_DOWN = Constants::Keybinds::eJoyInput::CROSS_DOWN;
    constexpr Constants::Keybinds::eJoyInput EXCAVATOR = Constants::Keybinds::eJoyInput::A;
    constexpr Constants::Keybinds::eJoyInput BEAK = Constants::Keybinds::eJoyInput::R2;
    constexpr Constants::Keybinds::eJoyInput CARROUSEL = Constants::Keybinds::eJoyInput::X;

}  // namespace KEYBINDINGS

#endif
