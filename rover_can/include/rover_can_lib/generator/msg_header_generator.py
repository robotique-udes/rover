import sys
import re
import os

def camel_to_snake(name):
    # Convert camel case to snake case
    name = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', name).upper()

def generate_class_name(input_file):
    class_name = input_file.split('.')[0]
    return class_name

def generate_header_guard(input_file):
    class_name_snake = camel_to_snake(input_file.split('.')[0])
    return f"__{class_name_snake.upper()}_HPP__"

def generate_cpp_header(input_file):
    with open(input_file, 'r') as file:
        class_name = generate_class_name(input_file)
        header_guard = generate_header_guard(input_file)
        output_file = input_file.split('.')[0] + ".hpp"
        
        if os.path.exists(output_file):
            os.remove(output_file)

        lines = file.readlines()
        members = []

        for line in lines:
            if len(line.strip()) > 0:
                members.append(line.strip())

        enum_class = f"enum class eMsgID : uint8_t\n{' ' * 8}{{\n"
        struct_members = f"struct sMsgData\n{' ' * 8}{{\n"

        for i, member in enumerate(members):
            member_name = member.split()[1].replace(";", "")  # Remove semicolon
            snake_case_name = camel_to_snake(member_name)
            enum_class += f"{' ' * 12}{snake_case_name} = 0x{i:02X},\n"
            struct_members += f"{' ' * 12}{member}\n"

        enum_class += f"{' ' * 12}eLAST\n{' ' * 8}}};\n"
        struct_members += f"{' ' * 8}}};"

        parseMsg_cases = ""  # Define parseMsg_cases here
        parseMsg_cases_linux = ""  # Define parseMsg_cases here
        getMsg_cases = ""

        for i, member in enumerate(members):
            member_name = member.split()[1].replace(";", "")  # Remove semicolon
            snake_case_name = camel_to_snake(member_name)
            parseMsg_cases += f"{' ' * 12}case eMsgID::{snake_case_name}:\n"
            parseMsg_cases += f"{' ' * 16}RoverCanLib::Helpers::canMsgToStruct<{member.split()[0]}, UnionDefinition::{member.split()[0].capitalize()}Union>(msg_, &this->data.{member_name});\n"
            parseMsg_cases += f"{' ' * 16}break;\n\n"
            parseMsg_cases_linux += f"{' ' * 12}case eMsgID::{snake_case_name}:\n"
            parseMsg_cases_linux += f"{' ' * 16}RoverCanLib::Helpers::canMsgToStruct<{member.split()[0]}, UnionDefinition::{member.split()[0].capitalize()}Union>(msg_, &this->data.{member_name}, logger_);\n"
            parseMsg_cases_linux += f"{' ' * 16}break;\n\n"
            getMsg_cases += f"{' ' * 12}case eMsgID::{snake_case_name}:\n"
            getMsg_cases += f"{' ' * 16}Helpers::structToCanMsg<{member.split()[0]}, UnionDefinition::{member.split()[0].capitalize()}Union>(&data.{member_name}, msg_);\n"
            getMsg_cases += f"{' ' * 16}break;\n\n"

        cpp_template = """#ifndef {header_guard}
#define {header_guard}

#include <cstdint>
#include "rover_can_lib/msgs/msg.hpp"

#if defined(ESP32)
#include "driver/twai.h"
#elif defined(__linux__) // defined(ESP32)
#include <linux/can.h>
#endif // defined(ESP32)

namespace RoverCanLib
{{
    namespace Helpers
    {{
#if defined(ESP32)
        twai_message_t getErrorIdMsg(void);

        template <typename COPY_TYPE, typename UNION_TYPE>
        void canMsgToStruct(IN const twai_message_t *msg_, OUT COPY_TYPE *dest_);

        template <typename COPY_TYPE, typename UNION_TYPE>
        void structToCanMsg(IN const COPY_TYPE *structMember_, OUT twai_message_t *msg_);
#endif
    }}
}}

#include "rover_can_lib/helpers.hpp"

namespace RoverCanLib::Msgs
{{
    class {class_name} : public Msg
    {{
    public:
        {enum_class}
        {struct_members}

        {class_name}() {{}}
        ~{class_name}() {{}}

#if defined(ESP32)
        Constant::eInternalErrorCode parseMsg(const twai_message_t *msg_)
        {{
            if (msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] != (uint8_t)Constant::eMsgId::ERROR_STATE)
            {{
                LOG(ERROR, "Mismatch in message types, maybe the lib version isn't the same between all nodes... Dropping msg");
                return Constant::eInternalErrorCode::WARNING;
            }}

            switch ((Msgs::{class_name}::eMsgID)(msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID]))
            {{
{parseMsg_cases}            default:
                LOG(WARN, "Unknown \\\"Message Specific Id\\\"");
                return Constant::eInternalErrorCode::ERROR;
            }}

            return Constant::eInternalErrorCode::OK;
        }}

        Constant::eInternalErrorCode getMsg(IN uint8_t msgId_, OUT twai_message_t *msg_)
        {{
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] = (uint8_t)Constant::eMsgId::ERROR_STATE;
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID] = msgId_;

            switch ((RoverCanLib::Msgs::{class_name}::eMsgID)msgId_)
            {{
{getMsg_cases}            default:
                LOG(ERROR, "Shouldn't ever fall here, implementation error");
                *msg_ = RoverCanLib::Helpers::getErrorIdMsg();
                return Constant::eInternalErrorCode::ERROR;
            }}

            return Constant::eInternalErrorCode::OK;
        }}
#elif defined(__linux__) // defined(ESP32)
        Constant::eInternalErrorCode parseMsg(const can_frame *msg_, rclcpp::Logger logger_)
        {{
            if (msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] != (uint8_t)Constant::eMsgId::ERROR_STATE)
            {{
                RCLCPP_ERROR(logger_, "Mismatch in message types, maybe the lib version isn't the same between all nodes... Dropping msg");
                return Constant::eInternalErrorCode::WARNING;
            }}

            switch ((Msgs::{class_name}::eMsgID)(msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID]))
            {{
{parseMsg_cases_linux}            default:
                RCLCPP_WARN(logger_, "Unknown \\\"Message Specific Id\\\"");
                return Constant::eInternalErrorCode::ERROR;
            }}

            return Constant::eInternalErrorCode::OK;
        }}

        Constant::eInternalErrorCode getMsg(IN uint8_t msgId_, OUT can_frame *msg_, rclcpp::Logger logger_)
        {{
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] = (uint8_t)Constant::eMsgId::ERROR_STATE;
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID] = msgId_;

            switch ((RoverCanLib::Msgs::{class_name}::eMsgID)msgId_)
            {{
{getMsg_cases}            default:
                RCLCPP_ERROR(logger_, "Shouldn't ever fall here, implementation error");
                *msg_ = RoverCanLib::Helpers::getErrorIdMsg();
                return Constant::eInternalErrorCode::ERROR;
            }}

            return Constant::eInternalErrorCode::OK;
        }}
#endif // defined(ESP32)

        uint8_t getMsgIDNb(void)
        {{
            return (uint8_t)eMsgID::eLAST;
        }}

        sMsgData data;
    }};
}}

#endif // {header_guard}
""".format(class_name=class_name, enum_class=enum_class, struct_members=struct_members, parseMsg_cases=parseMsg_cases, parseMsg_cases_linux=parseMsg_cases_linux, getMsg_cases=getMsg_cases, header_guard=header_guard)

        with open(output_file, 'w') as output:
            output.write(cpp_template)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py input_file.txt")
        sys.exit(1)

    input_file = sys.argv[1]
    generate_cpp_header(input_file)
