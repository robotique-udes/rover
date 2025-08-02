#!/usr/bin/env python3

import sys
import re
import os
import glob

def snake_to_camel(snake_str):
    components = snake_str.split('_')
    return components[0] + ''.join(x.title() for x in components[1:])

def snake_to_capitalized_camel(snake_str):
    components = snake_str.split('_')
    return ''.join(x.title() for x in components)

def camel_to_upper_snake(name):
    name = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    name = re.sub('([a-z0-9])([A-Z])', r'\1_\2', name)
    name = re.sub(r'([0-9]+)([a-zA-Z])', r'\1_\2', name)
    name = re.sub(r'([a-zA-Z])([0-9]+)', r'\1_\2', name)
    return name.strip('_').upper()

def generate_class_name(input_file_name):
    class_name_snake = input_file_name.split('.')[0]
    return snake_to_capitalized_camel(class_name_snake)

def generate_header_guard(class_name_camel):
    class_name_snake = camel_to_upper_snake(class_name_camel.split('.')[0])
    return f"{class_name_snake}_HPP"

def generate_cpp_header(input_file_path):
    with open(input_file_path, 'r') as file:
        input_file_name = os.path.basename(input_file_path)
        output_file_dir = os.path.dirname(input_file_path) or "."
        output_file_basename = input_file_name.split('.')[0] + ".hpp"
        output_file = os.path.join(output_file_dir, output_file_basename)

        if os.path.exists(output_file):
            print(f"Output file {output_file} exists. Removing it.")
            os.remove(output_file)

        lines = file.readlines()
        members = []
        custom_includes = []

        for line in lines:
            line_strip = line.strip()
            if line_strip.startswith("#include"):
                custom_includes.append(line_strip)
            elif len(line_strip) > 0:
                members.append(line_strip)

        class_name = generate_class_name(input_file_name)
        class_name_upper_snake = camel_to_upper_snake(class_name)
        header_guard = generate_header_guard(class_name)
        logNodeName = class_name + "_msg"

        enum_class_eMsgContentID_members = ""
        struct_sMsgData_members = ""
        static_asserts = ""
        valid_msg_ids_member = ""
        constructor_init_to_zero = ""
        loadMsg_switch_case_gen = ""
        getCanMsg_switch_case_gen = ""

        for member in members:
            member_parts = member.split()
            if len(member_parts) < 2:
                print(f"Warning: Skipping malformed member line in {input_file_name}: '{member}'")
                continue
            member_name = member_parts[1].replace(";", "")
            member_name_capital_snake_case = camel_to_upper_snake(member_name)
            enum_class_eMsgContentID_members += f"{' ' * 12}{member_name_capital_snake_case},\n"
            struct_sMsgData_members += f"{' ' * 12}{member}\n"
            static_asserts += f"{' ' * 12}static_assert(sizeof({member_name}) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA), \"Can messages cannot include field longer than 6 bytes\");\n"
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

        # Clean up trailing commas and newlines
        enum_class_eMsgContentID_members = enum_class_eMsgContentID_members.rstrip("\n")
        struct_sMsgData_members = struct_sMsgData_members.rstrip("\n")
        static_asserts = static_asserts.rstrip("\n")
        constructor_init_to_zero = constructor_init_to_zero.rstrip("\n")
        valid_msg_ids_member = valid_msg_ids_member.rstrip(", ")
        loadMsg_switch_case_gen = loadMsg_switch_case_gen.rstrip("\n\n")
        getCanMsg_switch_case_gen = getCanMsg_switch_case_gen.rstrip("\n\n")

        includes_block = "\n".join(custom_includes)

        cpp_template = \
f"""\
#ifndef {header_guard}
#define {header_guard}

#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/helpers.hpp"
{includes_block}

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

{static_asserts}
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
"""

        with open(output_file, 'w') as output_f:
            output_f.write(cpp_template)
        print(f"Generated C++ header: {output_file}")


if __name__ == "__main__":
    files_to_process = []

    if len(sys.argv) > 1:
        for arg in sys.argv[1:]:
            expanded_files = glob.glob(arg)
            if not expanded_files:
                print(f"No file matches '{arg}'")
            files_to_process.extend(expanded_files)
    else:
        print("No .msg file provided as argument.")
        while True:
            user_input = input("Enter path to .msg file (or press ENTER to finish): ").strip()
            if not user_input:
                break
            if os.path.isfile(user_input):
                files_to_process.append(user_input)
            else:
                print(f"File '{user_input}' does not exist.")

    if not files_to_process:
        print("No valid input files provided. Exiting.")
        sys.exit(1)

    for input_file in files_to_process:
        try:
            print(f"Processing file: {input_file}")
            generate_cpp_header(input_file)
        except Exception as e:
            print(f"Error processing file {input_file}: {e}")