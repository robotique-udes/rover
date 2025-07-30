#ifndef ROVER_CAN2_MSGS_PTZ_CONFIG_HPP
#define ROVER_CAN2_MSGS_PTZ_CONFIG_HPP

#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/helpers.hpp"

DEFINE_LOG_NODE(PtzConfig_msg, Logger::eNodeState::OFF)

namespace RoverCan2::Msgs
{
    class PtzConfig : public Msg<PtzConfig>
    {
      public:
        enum class eMsgContentID : uint8_t
        {
            PAN_MIN_POSITION,
            PAN_MAX_POSITION,
            PAN_MAX_SPEED,
            TILT_MIN_POSITION,
            TILT_MAX_POSITION,
            TILT_MAX_SPEED,
            ZOOM_MIN_POSITION,
            ZOOM_MAX_POSITION,
            ZOOM_MAX_SPEED,
            eLAST,
        };

      private:
        struct sMsgData
        {
            float panMinPosition;
            float panMaxPosition;
            float panMaxSpeed;
            float tiltMinPosition;
            float tiltMaxPosition;
            float tiltMaxSpeed;
            float zoomMinPosition;
            float zoomMaxPosition;
            float zoomMaxSpeed;
        };

        static constexpr CompileTimeArray<eMsgContentID, TO_UNDERLYING(eMsgContentID::eLAST)> VALID_MSG_IDS
            = {eMsgContentID::PAN_MIN_POSITION,
               eMsgContentID::PAN_MAX_POSITION,
               eMsgContentID::PAN_MAX_SPEED,
               eMsgContentID::TILT_MIN_POSITION,
               eMsgContentID::TILT_MAX_POSITION,
               eMsgContentID::TILT_MAX_SPEED,
               eMsgContentID::ZOOM_MIN_POSITION,
               eMsgContentID::ZOOM_MAX_POSITION,
               eMsgContentID::ZOOM_MAX_SPEED};

      public:
        PtzConfig():
            Msg(Constant::eMsgId::PTZ_CONFIG)
        {
            _data.panMinPosition = static_cast<decltype(_data.panMinPosition)>(0);
            _data.panMaxPosition = static_cast<decltype(_data.panMaxPosition)>(0);
            _data.panMaxSpeed = static_cast<decltype(_data.panMaxSpeed)>(0);
            _data.tiltMinPosition = static_cast<decltype(_data.tiltMinPosition)>(0);
            _data.tiltMaxPosition = static_cast<decltype(_data.tiltMaxPosition)>(0);
            _data.tiltMaxSpeed = static_cast<decltype(_data.tiltMaxSpeed)>(0);
            _data.zoomMinPosition = static_cast<decltype(_data.zoomMinPosition)>(0);
            _data.zoomMaxPosition = static_cast<decltype(_data.zoomMaxPosition)>(0);
            _data.zoomMaxSpeed = static_cast<decltype(_data.zoomMaxSpeed)>(0);
        }

        eLoadMsgCode _loadMsg(const CanMsg& msg_)
        {
            if (msg_.getMsgID() == Constant::eMsgId::INVALID)
            {
                return eLoadMsgCode::ERROR_INVALID_MSG;
            }

            if (msg_.getMsgID() != this->getMsgId())
            {
                return eLoadMsgCode::NOT_CONCERNED;
            }

            eMsgContentID msgContentId = static_cast<eMsgContentID>(msg_.getMsgContentID());
            if (!VALID_MSG_IDS.contains(msgContentId))
            {
                LOG_DEBUG(Logger::Nodes::PtzConfig_msg,
                          "Mismatch between received message and local message definition. Received msgContentId: (%u), "
                          "expected lower than (%u) and none zero",
                          TO_UNDERLYING(msgContentId),
                          TO_UNDERLYING(eMsgContentID::eLAST));
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            bool success = false;
            switch (msgContentId)
            {
                case eMsgContentID::PAN_MIN_POSITION:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.panMinPosition);
                    LOG_DEBUG(Logger::Nodes::PtzConfig_msg,
                              "switch (msgContentId) case eMsgContentID::PAN_MIN_POSITION: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::PAN_MAX_POSITION:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.panMaxPosition);
                    LOG_DEBUG(Logger::Nodes::PtzConfig_msg,
                              "switch (msgContentId) case eMsgContentID::PAN_MAX_POSITION: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::PAN_MAX_SPEED:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.panMaxSpeed);
                    LOG_DEBUG(Logger::Nodes::PtzConfig_msg,
                              "switch (msgContentId) case eMsgContentID::PAN_MAX_SPEED: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::TILT_MIN_POSITION:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.tiltMinPosition);
                    LOG_DEBUG(Logger::Nodes::PtzConfig_msg,
                              "switch (msgContentId) case eMsgContentID::TILT_MIN_POSITION: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::TILT_MAX_POSITION:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.tiltMaxPosition);
                    LOG_DEBUG(Logger::Nodes::PtzConfig_msg,
                              "switch (msgContentId) case eMsgContentID::TILT_MAX_POSITION: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::TILT_MAX_SPEED:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.tiltMaxSpeed);
                    LOG_DEBUG(Logger::Nodes::PtzConfig_msg,
                              "switch (msgContentId) case eMsgContentID::TILT_MAX_SPEED: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::ZOOM_MIN_POSITION:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.zoomMinPosition);
                    LOG_DEBUG(Logger::Nodes::PtzConfig_msg,
                              "switch (msgContentId) case eMsgContentID::ZOOM_MIN_POSITION: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::ZOOM_MAX_POSITION:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.zoomMaxPosition);
                    LOG_DEBUG(Logger::Nodes::PtzConfig_msg,
                              "switch (msgContentId) case eMsgContentID::ZOOM_MAX_POSITION: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::ZOOM_MAX_SPEED:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.zoomMaxSpeed);
                    LOG_DEBUG(Logger::Nodes::PtzConfig_msg,
                              "switch (msgContentId) case eMsgContentID::ZOOM_MAX_SPEED: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::eLAST:
                    [[fallthrough]];
                default:
                    return eLoadMsgCode::ERROR_IMPLEMENTATION;
            }

            if (!success)
            {
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            if (Helpers::MSG_CONTENT_IS_LAST_ELEM<eMsgContentID>(msg_))
            {
                return eLoadMsgCode::SUCCESS_COMPLETE;
            }
            else
            {
                return eLoadMsgCode::SUCCESS_INCOMPLETE;
            }
        }

        std::optional<CanMsg> _getCanMsg(const uint8_t msgContentId_) const
        {
            eMsgContentID msgContentID = static_cast<eMsgContentID>(msgContentId_);

            if (!VALID_MSG_IDS.contains(msgContentID))
            {
                return std::nullopt;
            }

            CanMsg msg_;
            switch (static_cast<eMsgContentID>(msgContentId_))
            {
                case eMsgContentID::PAN_MIN_POSITION:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.panMinPosition, msg_);
                    break;

                case eMsgContentID::PAN_MAX_POSITION:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.panMaxPosition, msg_);
                    break;

                case eMsgContentID::PAN_MAX_SPEED:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.panMaxSpeed, msg_);
                    break;

                case eMsgContentID::TILT_MIN_POSITION:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.tiltMinPosition, msg_);
                    break;

                case eMsgContentID::TILT_MAX_POSITION:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.tiltMaxPosition, msg_);
                    break;

                case eMsgContentID::TILT_MAX_SPEED:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.tiltMaxSpeed, msg_);
                    break;

                case eMsgContentID::ZOOM_MIN_POSITION:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.zoomMinPosition, msg_);
                    break;

                case eMsgContentID::ZOOM_MAX_POSITION:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.zoomMaxPosition, msg_);
                    break;

                case eMsgContentID::ZOOM_MAX_SPEED:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.zoomMaxSpeed, msg_);
                    break;

                case eMsgContentID::eLAST:
                    [[fallthrough]];

                default:
                    return std::nullopt;
                    break;
            }

            return msg_;
        }

        uint8_t _getMsgContentCount(void) const
        {
            return TO_UNDERLYING(eMsgContentID::eLAST);
        }

        sMsgData& data(void)
        {
            return _data;
        }

        const sMsgData& getData(void) const
        {
            return static_cast<const sMsgData&>(_data);
        }

      private:
        sMsgData _data;
    };

}  // namespace RoverCan2::Msgs

#endif  // ROVER_CAN2_MSGS_PTZ_CONFIG_HPP
