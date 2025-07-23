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
        # Ensure output_file is placed correctly, possibly in the same dir as input or a specified output dir
        # For simplicity, this example places it in the same directory as the input file.
        output_file_dir = os.path.dirname(input_file_path)
        if not output_file_dir: # Handle case where input_file_path is just a filename
            output_file_dir = "."
        output_file_basename = input_file_name.split('.')[0] + ".hpp"
        output_file = os.path.join(output_file_dir, output_file_basename)


        if os.path.exists(output_file):
            print(f"Output file {output_file} exists. Removing it.")
            os.remove(output_file)

        lines = file.readlines()
        members = []

        for line in lines:
            if len(line.strip()) > 0:
                members.append(line.strip())

        class_name = generate_class_name(input_file_name)
        class_name_upper_snake = camel_to_upper_snake(class_name)
        header_guard = generate_header_guard(class_name) # Note: generate_header_guard expects class_name, not input_file_name
        logNodeName = class_name + "_msg"
        enum_class_eMsgContentID_members = ""
        struct_sMsgData_members = ""
        valid_msg_ids_member = ""
        constructor_init_to_zero = ""
        loadMsg_switch_case_gen = ""
        getCanMsg_switch_case_gen = ""

        for i, member in enumerate(members):
            member_parts = member.split()
            if len(member_parts) < 2:
                print(f"Warning: Skipping malformed member line in {input_file_name}: '{member}'")
                continue
            member_name = member_parts[1].replace(";", "")  # Remove semicolon
            member_name_capital_snake_case = camel_to_upper_snake(member_name)
            enum_class_eMsgContentID_members += f"{' ' * 12}{member_name_capital_snake_case},\n"
            struct_sMsgData_members += f"{' ' * 12}{member}\n" # original member line with type and name
            valid_msg_ids_member += f"eMsgContentID::{member_name_capital_snake_case}, "
            constructor_init_to_zero += f"{' ' * 12}_data.{member_name} = static_cast<decltype(_data.{member_name})>(0);\n"

            loadMsg_switch_case_gen += \
f"""\
{16*' '}case eMsgContentID::{member_name_capital_snake_case}:
{20*' '}success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.{member_name});
{20*' '}LOG_DEBUG(Logger::Nodes::{logNodeName},
{30*' '}"switch (msgContentId) case eMsgContentID::{member_name_capital_snake_case}: %s",
{30*' '}success ? "success" : "failed");
{20*' '}break;\n
"""
            getCanMsg_switch_case_gen += \
f"""\
{16*' '}case eMsgContentID::{member_name_capital_snake_case}:
{20*' '}Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.{member_name}, msg_);
{20*' '}break;\n
"""
        #end_for

        # Removes unwanted trailing elements
        if enum_class_eMsgContentID_members.endswith(",\n"):
             enum_class_eMsgContentID_members = enum_class_eMsgContentID_members[:-1]
        if struct_sMsgData_members.endswith("\n"):
            struct_sMsgData_members = struct_sMsgData_members[:-1]
        if constructor_init_to_zero.endswith("\n"):
            constructor_init_to_zero = constructor_init_to_zero[:-1]
        if valid_msg_ids_member.endswith(", "):
            valid_msg_ids_member = valid_msg_ids_member[:-2]
        if loadMsg_switch_case_gen.endswith("\n\n"): # It adds an extra \n
             loadMsg_switch_case_gen = loadMsg_switch_case_gen[:-1]
        if getCanMsg_switch_case_gen.endswith("\n\n"): # It adds an extra \n
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
        {class_name}():
            Msg(Constant::eMsgId::{class_name_upper_snake})
        {{
{constructor_init_to_zero}
        }}

        eLoadMsgCode _loadMsg(const CanMsg& msg_)
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
                          "Mismatch between received message and local message definition. Received msgContentId: (%u), "
                          "expected lower than (%u) and none zero",
                          TO_UNDERLYING(msgContentId),
                          TO_UNDERLYING(eMsgContentID::eLAST));
                return eLoadMsgCode::ERROR_MISMATCH;
            }}

            bool success = false;
            switch (msgContentId)
            {{
{loadMsg_switch_case_gen}
                case eMsgContentID::eLAST:
                    [[fallthrough]];
                default:
                    return eLoadMsgCode::ERROR_IMPLEMENTATION;
            }}

            if (!success)
            {{
                return eLoadMsgCode::ERROR_MISMATCH;
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

        std::optional<CanMsg> _getCanMsg(const uint8_t msgContentId_) const
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
                    [[fallthrough]];
                    
                default:
                    return std::nullopt;
                    break;
            }}

            return msg_;
        }}

        uint8_t _getMsgContentCount(void) const
        {{
            return TO_UNDERLYING(eMsgContentID::eLAST);
        }}

        sMsgData& data(void)
        {{
            return _data;
        }}

        const sMsgData& getData(void) const
        {{
            return static_cast<const sMsgData&>(_data);
        }}

      private:
        sMsgData _data;
    }};

}}  // namespace RoverCan2::Msgs

#endif  // {header_guard}
\
"""
        with open(output_file, 'w') as output_f:
            output_f.write(cpp_template)
        print(f"Generated C++ header: {output_file}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: ./msg_generator.py <file1.txt> [file2.txt ...]")
        print("       Wildcards like * can be used (e.g., ./msg_generator.py msg_files/*.txt)")
        sys.exit(1)

    for input_file_arg in sys.argv[1:]:
        # This script assumes shell expansion for wildcards like msg_files/*
        # If a path doesn't resolve to a file, it will be caught here.
        if os.path.isfile(input_file_arg):
            try:
                print(f"Processing file: {input_file_arg}")
                generate_cpp_header(input_file_arg)
            except Exception as e:
                print(f"Error processing file {input_file_arg}: {e}")
        else:
            print(f"Skipping '{input_file_arg}': not a valid file or does not exist.")
