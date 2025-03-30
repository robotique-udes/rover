#ifndef __KEYBINDING2__
#define __KEYBINDING2__

namespace KEYBINDINGS_EMILE
{
    constexpr uint8_t DEADMAN_SWITCH = rover_msgs::msg::Joy::L1;

    namespace JOINT
    {
        namespace JOINT_SELECTION
        {
            constexpr uint8_t INC = rover_msgs::msg::Joy::CROSS_UP;
            constexpr uint8_t DEC = rover_msgs::msg::Joy::CROSS_DOWN;
        }  // namespace JOINT_SELECTION

        namespace JL
        {
            constexpr uint8_t RIGHT = rover_msgs::msg::Joy::B;
            constexpr uint8_t LEFT = rover_msgs::msg::Joy::X;
            constexpr uint8_t ID = TO_UNDERLYING(eJointIndex::JL);
        }  // namespace JL

        namespace J1
        {
            constexpr uint8_t FWD = rover_msgs::msg::Joy::Y;
            constexpr uint8_t REV = rover_msgs::msg::Joy::A;
            constexpr uint8_t ID = TO_UNDERLYING(eJointIndex::J1);
        }  // namespace J1

        namespace J2
        {
            constexpr uint8_t FWD = rover_msgs::msg::Joy::Y;
            constexpr uint8_t REV = rover_msgs::msg::Joy::A;
            constexpr uint8_t ID = TO_UNDERLYING(eJointIndex::J2);
        }  // namespace J2

    }  // namespace JOINT

    namespace GRIPPER
    {
        constexpr uint8_t ROT_FWD = rover_msgs::msg::Joy::R2;
        constexpr uint8_t ROT_REV = rover_msgs::msg::Joy::L2;
        constexpr uint8_t ROT_ID = TO_UNDERLYING(eJointIndex::GRIPPER_ROT);

        constexpr uint8_t TILT_FWD = rover_msgs::msg::Joy::Y;
        constexpr uint8_t TILT_REV = rover_msgs::msg::Joy::A;
        constexpr uint8_t TILT_ID = TO_UNDERLYING(eJointIndex::GRIPPER_TILT);

        constexpr uint8_t CLOSE = rover_msgs::msg::Joy::CROSS_DOWN;
        constexpr uint8_t CLOSE_ID = TO_UNDERLYING(eJointIndex::GRIPPER_CLOSE);

    }  // namespace GRIPPER

}  // namespace KEYBINDINGS_EMILE

#endif
