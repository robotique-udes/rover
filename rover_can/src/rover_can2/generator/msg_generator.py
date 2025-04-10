#!/usr/bin/env python3

import sys
import re
import os


def snake_to_camel(snake_str):
    components = snake_str.split('_')
    return components[0] + ''.join(x.title() for x in components[1:])


def snake_to_capitalized_camel(snake_str):
    components = snake_str.split('_')
    return ''.join(x.title() for x in components)


def camel_to_upper_snake(name):
    # Convert camel case to snake case
    name = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    name = re.sub('([a-z0-9])([A-Z])', r'\1_\2', name)
    # Separate numbers by underscore
    name = re.sub(r'([0-9]+)([a-zA-Z])', r'\1_\2', name)
    # Separate letters following numbers by underscore
    name = re.sub(r'([a-zA-Z])([0-9]+)', r'\1_\2', name)
    # Remove underscore at the front or the end if any
    name = name.lstrip('_')
    name = name.rstrip('_')
    return name.upper()


def camel_to_snake(value):
    return camel_to_upper_snake(value).lower()


def generate_class_name(input_file_name):
    class_name_snake = input_file_name.split('.')[0]
    class_name = snake_to_capitalized_camel(class_name_snake)
    return class_name


def generate_header_guard(class_name_camel):
    class_name_snake = camel_to_upper_snake(class_name_camel.split('.')[0])
    return f"{class_name_snake.upper()}_HPP"


def generate_cpp_header(input_file_path):
    with open(input_file_path, 'r') as file:
        input_file_name = os.path.basename(input_file_path)
        output_file = input_file_path.split('.')[0] + ".hpp"

        if os.path.exists(output_file):
            os.remove(output_file)

        lines = file.readlines()
        members = []

        for line in lines:
            if len(line.strip()) > 0:
                members.append(line.strip())

        class_name = generate_class_name(input_file_name)
        class_name_upper_snake = camel_to_upper_snake(class_name)
        header_guard = generate_header_guard(class_name)
        logNodeName = class_name + "_msg"
        enum_class_eMsgContentID_members = ""
        struct_sMsgData_members = ""
        valid_msg_ids_member = ""
        constructor_init_to_zero = ""
        loadMsg_switch_case_gen = ""
        getCanMsg_switch_case_gen = ""

        for i, member in enumerate(members):
            member_name = member.split()[1].replace(";",
                                                    "")  # Remove semicolon
            member_name_capital_snake_case = camel_to_upper_snake(member_name)
            enum_class_eMsgContentID_members += f"{' ' * 12}{member_name_capital_snake_case},\n"
            struct_sMsgData_members += f"{' ' * 12}{member}\n"
            valid_msg_ids_member += f"eMsgContentID::{member_name_capital_snake_case}, "
            constructor_init_to_zero += f"{' ' * 8}_data.{member_name} = static_cast<decltype(_data.{member_name})>(0);\n"

            loadMsg_switch_case_gen += \
f"""\
{12*' '}case eMsgContentID::{member_name_capital_snake_case}:
{16*' '}success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.{member_name});
{16*' '}LOG_DEBUG(Logger::Nodes::{logNodeName},
{26*' '}"switch (msgContentId) case eMsgContentID::{member_name_capital_snake_case}: %s",
{26*' '}success ? "success" : "failed");
{16*' '}break;\n
"""
            getCanMsg_switch_case_gen += \
f"""\
{12*' '}case eMsgContentID::{member_name_capital_snake_case}:
{16*' '}Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.{member_name}, msg_);
{16*' '}break;\n
"""
        #end_for

        # Removes unwanted trailing elements
        enum_class_eMsgContentID_members = enum_class_eMsgContentID_members[:-1]
        struct_sMsgData_members = struct_sMsgData_members[:-1]
        constructor_init_to_zero = constructor_init_to_zero[:-1] 
        valid_msg_ids_member = valid_msg_ids_member[:-2]
        loadMsg_switch_case_gen = loadMsg_switch_case_gen[:-1]
        getCanMsg_switch_case_gen = getCanMsg_switch_case_gen[:-1]

        cpp_template = \
f"""\
#ifndef {header_guard}
#define {header_guard}

#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/helpers.hpp"

DEFINE_LOG_NODE({logNodeName}, Logger::eNodeState::OFF)

namespace RoverCan2::Msgs
{{
    class {class_name} : public Msg<{class_name}>
    {{
      public:
        enum class eMsgContentID : uint8_t
        {{
{enum_class_eMsgContentID_members}
            eLAST,
        }};

      private:
        struct sMsgData
        {{
{struct_sMsgData_members}
        }};

        static constexpr CompileTimeArray<eMsgContentID, TO_UNDERLYING(eMsgContentID::eLAST)> VALID_MSG_IDS
            = {{{valid_msg_ids_member}}};

      public:
        {class_name}();

        eLoadMsgCode _loadMsg(const CanMsg& msg_);
        std::optional<CanMsg> _getCanMsg(const uint8_t msgContentId_) const;
        uint8_t _getMsgContentCount(void) const;
        sMsgData& data(void);
        const sMsgData& getData(void) const;

      private:
        sMsgData _data;
    }};

    {class_name}::{class_name}():
        Msg(Constant::eMsgId::{class_name_upper_snake})
    {{
{constructor_init_to_zero}
    }}

    eLoadMsgCode {class_name}::_loadMsg(const CanMsg& msg_)
    {{
        if (msg_.getMsgID() == Constant::eMsgId::INVALID)
        {{
            return eLoadMsgCode::ERROR_INVALID_MSG;
        }}

        if (msg_.getMsgID() != this->getMsgId())
        {{
            return eLoadMsgCode::NOT_CONCERNED;
        }}

        eMsgContentID msgContentId = static_cast<eMsgContentID>(msg_.getMsgContentID());
        if (!VALID_MSG_IDS.contains(msgContentId))
        {{
            LOG_DEBUG(Logger::Nodes::{logNodeName},
                      "Missmatch between received message and local message definition. Received msgContentId: (%u), "
                      "expected lower than (%u) and none zero",
                      TO_UNDERLYING(msgContentId),
                      TO_UNDERLYING(eMsgContentID::eLAST));
            return eLoadMsgCode::ERROR_MISSMATCH;
        }}

        bool success = false;
        switch (msgContentId)
        {{
{loadMsg_switch_case_gen}
            default:
                return eLoadMsgCode::ERROR_IMPLEMENTATION;
        }}

        if (!success)
        {{
            return eLoadMsgCode::ERROR_MISSMATCH;
        }}

        if (Helpers::MSG_CONTENT_IS_LAST_ELEM<eMsgContentID>(msg_))
        {{
            return eLoadMsgCode::SUCCESS_COMPLETE;
        }}
        else
        {{
            return eLoadMsgCode::SUCCESS_INCOMPLETE;
        }}
    }}

    std::optional<CanMsg> {class_name}::_getCanMsg(const uint8_t msgContentId_) const
    {{
        eMsgContentID msgContentID = static_cast<eMsgContentID>(msgContentId_);

        if (!VALID_MSG_IDS.contains(msgContentID))
        {{
            return std::nullopt;
        }}

        CanMsg msg_;
        switch (static_cast<eMsgContentID>(msgContentId_))
        {{
{getCanMsg_switch_case_gen}
            case eMsgContentID::eLAST:
                return std::nullopt;
        }}

        return msg_;
    }}

    uint8_t {class_name}::_getMsgContentCount(void) const
    {{
        return TO_UNDERLYING(eMsgContentID::eLAST);
    }}

    {class_name}::sMsgData& {class_name}::data(void)
    {{
        return _data;
    }}
    
    const {class_name}::sMsgData& {class_name}::getData(void) const
    {{
        return static_cast<const {class_name}::sMsgData&>(_data);
    }}
}}  // namespace RoverCan2::Msgs

#endif  // {header_guard}
\
"""
        with open(output_file, 'w') as output:
            output.write(cpp_template)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: ./msg_generator.py pathToCustomMsgFile.txt")
        sys.exit(1)

    input_file = sys.argv[1]
    generate_cpp_header(input_file)
