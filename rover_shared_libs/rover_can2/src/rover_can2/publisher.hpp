#ifndef ROVER_CAN2_PUBLISHER_HPP
#define ROVER_CAN2_PUBLISHER_HPP

#include "rover_can2/msgs/msg.hpp"
#include "rover_lib2/helpers/circular_buffer.hpp"
#include "rover_lib2/helpers/macros.hpp"

#include <optional>

namespace RoverCan2
{
    // Forward declarations
    class ManagerT;

    /**
     * @brief Base Publisher class. Allows template abstraction for type validation
     */
    class PublisherBaseT
    {
      protected:
        PublisherBaseT() = default;
    };

    /**
     * Allows MSG_QUEUE_SIZE template abstraction for type validation while keeping MsgT
     *
     * @tparam MsgT
     */
    template<typename MsgT>
    class PublisherBase : public PublisherBaseT
    {
      protected:
        PublisherBase() = default;
    };

    /**
     * @brief Registering a publisher to a DeviceT will allow the corresponding device to send msg of that type
     *
     * @tparam MsgT Type of msg which the subscriber can send
     * @tparam MSG_QUEUE_SIZE Sent msgs are bufferized, increasing this lowers the chance of data loss if a msgs wasn't sent
     * before a 2nd call to queueMsg() but slows down code execution, increase bandwidth usage and memory usage.
     */
    template<typename MsgT, size_t MSG_QUEUE_SIZE = 1UL>
    class Publisher : public PublisherBase<MsgT>
    {
        VALIDATE_BASE_TYPE(Msgs::MsgBaseT, MsgT);

        static constexpr size_t MAX_MSG_TRANSMISSION_PER_CALL = MSG_QUEUE_SIZE;

      public:
        /**
         * @brief Queue a msg for transmission
         *
         * @attention [WARNING] Msgs won't be sent until a Manager "update/spin" is called
         * @param msg_
         */
        CircularBufferT::eErrorCode queueMsg(const MsgT& msg_)
        {
            return _msgToSendBuffer.addValue(msg_);
        }

        /**
         * @brief Called by the manager in its update loop, can be used directly by user but isn't recommended
         *
         * @tparam ManagerT
         * @param senderId_
         * @param manager_
         */
        template<typename ManagerT>
        bool sendQueuedMsgs(Constant::eDeviceId senderId_, ManagerT& manager_)
        {
            VALIDATE_BASE_TYPE(ManagerT, ManagerT);

            bool allSuccess = true;
            for (size_t i = 0; i < MAX_MSG_TRANSMISSION_PER_CALL; i++)
            {
                if (std::optional<MsgT> msgToSend = _msgToSendBuffer.getValue())
                {
                    allSuccess &= manager_.sendMsg(senderId_, msgToSend.value());
                }
            }

            return allSuccess;
        }

      private:
        CircularBuffer<MsgT, MSG_QUEUE_SIZE> _msgToSendBuffer;
    };
}  // namespace RoverCan2
#endif  // ROVER_CAN2_PUBLISHER_HPP
