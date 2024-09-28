# Table of content

- [Table of content](#table-of-content)
- [Getting started](#getting-started)
    - [Useful links](#useful-links)
    - [Task tracking and team organisation](#task-tracking-and-team-organisation)
    - [Architecture](#architecture)
    - [Software development](#software-development)
    - [Electrical development](#electrical-development)
- [Dependencies](#dependencies)
    - [Upgrade already installed package:](#upgrade-already-installed-package)
    - [From apt](#from-apt)
    - [From pip](#from-pip)
  - [ESP32 and micro controller ROS development](#esp32-and-micro-controller-ros-development)

# Getting started

To setup your computer for development, [read this page](doc/prog/how_to/intial_setup.md). The rest of the documentation takes for granted that you are all set up.

### Useful links

- [Rovus Github Project](https://github.com/orgs/robotique-udes/projects/1/views/1): Task management, backlog and test planning
- [Coding Guidelines](doc/prog/how_to/coding%20guidelines.md): Good practices and styling guide
- [Electrical Standard](doc/electrical/electric_standard.md): Like coding guidelines but for electrical elements
- [Testing for the first time guide TODO](TODO): Guide to start, test and stop the rover safely
- [Documentation](doc/): Folder with all the documentation
- [*rover* repo](https://github.com/robotique-udes/rover): Rover main repo, ROS code/packages and all documentation
- [*rover_micro* repo](https://github.com/robotique-udes/rover_micro): Rover's Micro controller project (platformio) repo
- [*PCB* repo](https://github.com/robotique-udes/PCB): All PCBs projects
- [MS Teams](https://teams.microsoft.com/l/channel/19%3Ae38473dc00d9495599b19b8342af0a4c%40thread.skype/Rovus%20-%20G%C3%A9n%C3%A9ral?groupId=91136b22-e319-4e01-a080-e57a35690eee): Larger files storage and robotiqueUdeS team management
- [How to request new feature on the GUI](doc/prog/gui/feature_request.md): Procedure to make request for GUI element necessary for your tasks
- [ADD MORE!]()

### Task tracking and team organisation

We now use _github project_ to manage tasks, all members of the github rovus team can now access the project on the [RobotiqueUdeS project tab](https://github.com/orgs/robotique-udes/projects/1/views/1)

### Architecture

<center>
    <img src="doc/diagrams/VeryHighLevelSoftwareStructure.drawio.svg" alt="Very high level structure diagram" class="center" style="width:700px;"/>
</center>

- **TODO: Bring the ROS architecture from Teams to github**

The current ROS architecture is located on the _RobotiqueUdeS MSTeams_ -> _Rovus-General_ channel -> _architecture_ tab. Refer to it when naming topics/services/nodes/namespaces/etc. and update the diagram if you see mismatch.

### Software development

All the codebase is regrouped into two distinct repositories:
- [rover](https://github.com/robotique-udes/rover): Contains all the code running on the base station computer or the rover main computer
  - The *rover* repo is a ROS metapackage and should only be cloned into your ROS workspace (*ros2_ws/src*).
- [rover_micro](https://github.com/robotique-udes/rover_micro): Contains all micro controller projects which runs on our multiple PCBs 
  - The rover_micro repo shouldn't be cloned inside your ROS workspace as a requirement for [micro ros](https://micro.ros.org/) because the repo might include micro ros projects in the future.

A folder structure example is in [the setup documentation](doc/prog/master%20computing%20unit/setup.md).

The use of [gitsource](https://sourcegit-scm.github.io/) offers a GUI for git (for windows and linux) which is very convenient for new users. It's used by most of the team members, ask any software team member for a quick tutorial if needed.

For more info about development see the in [detail readme](TODO)

### Electrical development

All the electrical projects (mostly PCBs) are located into the shared [PCB](https://github.com/robotique-udes/PCB) repo. Its a shared repo with the other RobotiqueUdeS group and doesn't have any branch protection. As of now, it's only used as a storage and sharing space. 

# Dependencies

Right now, it's still your responsibility to update your dependencies often (needs to be automated). If you get compilation or execution errors while running the most recent release branch it's probably because your dependencies are not installed. We'll try to always use the latest version of dependencies when possible, otherwise specify the version and add an explanation as to why the codebase wasn't updated to support the dependencies latest version.

### Upgrade already installed package:
After upgrading all packages, you'll want to reinstall all depedencies with a specified version (just run all commands below and it'll be done automatically)

```bash
sudo apt update
sudo apt upgrade
pip3 list --outdated --format=freeze | grep -v '^\-e' | cut -d = -f 1 | xargs -n1 pip3 install -U 
```

### From apt

```bash
sudo apt -y update
sudo apt -y install pip
sudo apt -y install python3-venv
sudo apt -y install python-is-python3
sudo apt -y install ros-humble-desktop
sudo apt -y install ros-humble-ros-base
sudo apt -y install ros-dev-tools
sudo apt -y install ros-humble-joy
sudo apt -y install can-utils
sudo apt -y install qt6-base-dev
sudo apt -y install gstreamer1.0-rtsp
rm -r ~/.cache/gstreamer-1.0/
sudo apt purge gstreamer1.0-vaapi 
sudo apt -y install gir1.2-gst-rtsp-server-1.0
sudo apt -y install python3-gi
sudo apt -y install ros-humble-rviz2
sudo apt -y install ros-humble-joint-state-publisher
sudo apt -y install ros-humble-robot-state-publisher
sudo apt -y install libssh-dev
sudo apt -y install sl
```

### From pip

```bash
pip install setuptools==58.2.0 #*
pip install empy==3.3.4 #**
pip install opencv-python
pip install opencv-contrib-python
```

- *Install setuptools version 58.2.0 for compatibility reasons ([further details](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/))
- **Install empy version 3.3.4 because version 4.0 cause problems ([further details](https://robotics.stackexchange.com/questions/105842/what-causes-build-error-cannot-import-name-override-opt-from-em))

## ESP32 and micro controller ROS development

See corresponding [repository](https://github.com/robotique-udes/rover_micro)
