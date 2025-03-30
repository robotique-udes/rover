#ifndef __KEYBINDING2__
#define __KEYBINDING2__

namespace KEYBINDINGS_EMILE
{
    constexpr uint8_t DEADMAN_SWITCH = rover_msgs::msg::Joy::L1;

    namespace JOINT
    {
        namespace JOINT_SELECTION
        {
            constexpr uint8_t JOINT_SELECT_INC = rover_msgs::msg::Joy::CROSS_UP;
            constexpr uint8_t JOINT_SELECT_DEC = rover_msgs::msg::Joy::CROSS_DOWN;
        }  // namespace JOINT_SELECTION

        namespace JL
        {
            constexpr uint8_t JL_RIGHT = rover_msgs::msg::Joy::B;
            constexpr uint8_t JL_LEFT = rover_msgs::msg::Joy::X;
            constexpr uint8_t ID = TO_UNDERLYING(eJointIndex::JL);
        }  // namespace JL

        namespace J1
        {
            constexpr uint8_t J1_FWD = rover_msgs::msg::Joy::Y;
            constexpr uint8_t J1_REV = rover_msgs::msg::Joy::A;
            constexpr uint8_t ID = TO_UNDERLYING(eJointIndex::J1);
        }  // namespace J1

        namespace J2
        {
            constexpr uint8_t J2_FWD = rover_msgs::msg::Joy::Y;
            constexpr uint8_t J2_REV = rover_msgs::msg::Joy::A;
            constexpr uint8_t ID = TO_UNDERLYING(eJointIndex::J2);
        }  // namespace J2

    }  // namespace JOINT

    namespace GRIPPER
    {

    }

}  // namespace KEYBINDINGS_EMILE

#endif
