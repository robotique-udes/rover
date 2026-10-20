import json
import os
import subprocess

from launch import LaunchDescription
from launch.logging import get_logger
from launch_ros.actions import Node


LOGGER = get_logger("joy_launch")
REGISTRY_PATH = os.environ.get(
    "ROVER_CONTROLLER_REGISTRY",
    os.path.expanduser("~/.config/rover/controllers.json"),
)
SUPPORTED_TYPES = {"PS4", "DS4", "Xbox", "XBox", "Logitech", "Logitech Generic"}


def read_controller_profiles():
    if not os.path.isfile(REGISTRY_PATH):
        return []

    try:
        with open(REGISTRY_PATH, encoding="utf-8") as registry_file:
            registry = json.load(registry_file)
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(f"Could not read controller registry {REGISTRY_PATH}: {error}") from error

    profiles = registry.get("controllers")
    if not isinstance(profiles, list):
        raise RuntimeError(f"Controller registry {REGISTRY_PATH} must contain a controllers list.")
    return [
        profile
        for profile in profiles
        if isinstance(profile, dict)
        and profile.get("type") in SUPPORTED_TYPES
        and profile.get("sdl_guid")
        and profile.get("sdl_name")
    ]


def read_sdl_joysticks():
    try:
        result = subprocess.run(
            ["ros2", "run", "joy", "joy_enumerate_devices"],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as error:
        raise RuntimeError(f"Could not enumerate connected SDL controllers: {error}") from error

    devices = []
    for line in result.stdout.splitlines():
        if not line.strip() or line.startswith("ID") or line.startswith("---"):
            continue
        separator = "|" if "|" in line else ":"
        fields = [field.strip() for field in line.split(separator)]
        if len(fields) < 5:
            continue
        try:
            device_id = int(fields[0])
        except ValueError:
            continue
        devices.append(
            {
                "id": device_id,
                "guid": fields[1],
                "name": separator.join(fields[4:]).strip(),
            }
        )
    return devices


def infer_controller_type(device):
    name = device["name"].lower()
    guid = device["guid"].lower()

    if any(token in name for token in ("xbox", "x-input", "xinput")) or guid[8:12] == "5e04":
        return "Xbox"
    if any(token in name for token in ("logitech", "rumblepad", "f310", "f510", "f710")) or guid[8:12] == "6d04":
        return "Logitech"
    if any(token in name for token in ("dualshock", "sony", "wireless controller")) or guid[8:12] == "4c05":
        return "PS4"
    return None


def match_connected_controllers(profiles, devices):
    connected = []

    for device in devices:
        controller_type = next(
            (
                profile["type"]
                for profile in profiles
                if profile["sdl_guid"].lower() == device["guid"].lower()
                and profile["sdl_name"] == device["name"]
            ),
            None,
        )
        if controller_type is None:
            controller_type = infer_controller_type(device)
        if controller_type is not None:
            connected.append((device, controller_type))

    if len(connected) > 2:
        LOGGER.warning("More than two supported controllers are connected; using the first two.")
    return connected[:2]


def create_controller_nodes(slot, controller_type, device=None):
    slot_name = "main" if slot == 0 else "secondary"
    raw_topic = f"{slot_name}_raw"
    formatted_topic = f"{slot_name}_formatted"
    type_topic = f"{slot_name}_controller_type"

    joy_parameters = {
        "autorepeat_rate": 20.0,
        "deadzone": 0.0,
    }
    if device is None:
        joy_parameters["device_name"] = (
            "Wireless Controller" if slot == 0 else "Logitech RumblePad 2 USB"
        )
    else:
        joy_parameters["device_id"] = device["id"]

    joy_node = Node(
        package="joy",
        namespace="/base/joy",
        executable="joy_node",
        name=f"{slot_name}_joy_node",
        parameters=[joy_parameters],
        remappings=[("joy", raw_topic)],
    )
    formator_node = Node(
        package="rover_joy",
        namespace="/base/joy",
        executable="joy_formator",
        name=f"{slot_name}_formator",
        parameters=[{"controller_type": controller_type}],
        remappings=[
            ("raw/joy", raw_topic),
            ("formated/joy", formatted_topic),
            ("controller_type", type_topic),
        ],
    )
    return [joy_node, formator_node]


def generate_launch_description():
    devices = read_sdl_joysticks()
    profiles = read_controller_profiles()
    controllers = match_connected_controllers(profiles, devices)

    actions = []
    if controllers:
        for slot, (device, controller_type) in enumerate(controllers):
            LOGGER.info(
                f"Recognized {controller_type} controller {device['name']!r} "
                f"(SDL GUID {device['guid']})."
            )
            actions.extend(create_controller_nodes(slot, controller_type, device))
    else:
        LOGGER.warning(
            "No supported SDL controller was recognized; using the configured PS4 and Logitech defaults."
        )
        actions.extend(create_controller_nodes(0, "DS4"))
        actions.extend(create_controller_nodes(1, "Logitech"))

    actions.append(
        Node(
            package="rover_joy",
            namespace="/base/joy",
            executable="joy_demux",
            name="demux",
            parameters=[{"controller_type": "Logitech"}],
            remappings=[
                ("main_joy", "main_formatted"),
                ("secondary_joy", "secondary_formatted"),
            ],
        )
    )
    actions.append(
        Node(
            package="rover_joy",
            namespace="/base/joy",
            executable="joy_demux_controller",
            name="demux_controller",
        )
    )
    return LaunchDescription(actions)
