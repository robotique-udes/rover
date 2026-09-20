#!/bin/bash
#
# Fresh build of the workspace followed by clang-tidy on every package
# (packages under deps/ are built but not analyzed).
#
# Usage:
#   scripts/static_analysis.sh                # fresh build + clang-tidy on all packages
#   scripts/static_analysis.sh --skip-build   # reuse existing build/, only run clang-tidy
#   scripts/static_analysis.sh pkg_a pkg_b    # only analyze the given packages
#
# Requirements: ROS 2 Jazzy, clang-tidy, ros-jazzy-ament-clang-tidy

set -uo pipefail

ROS_DISTRO_NAME="jazzy"

BLUE="\e[0;34m"
GREEN="\e[0;32m"
RED="\e[0;31m"
YELLOW="\e[0;33m"
NC="\e[0m"

# Repo root = parent of scripts/. The colcon workspace is two levels above it
# (ws/src/rover -> ws), so build/ install/ log/ live in the workspace root.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_ROOT="$(cd "$REPO_ROOT/../.." && pwd)"
LOG_FILE="$REPO_ROOT/tidy.log"

if [ ! -d "$WS_ROOT/src" ]; then
    echo -e "${RED}[ERROR] Expected the repo at <workspace>/src/<repo>, but $WS_ROOT/src does not exist${NC}"
    exit 1
fi
cd "$WS_ROOT" || exit 1

# --- Arguments ---------------------------------------------------------------
SKIP_BUILD=false
REQUESTED_PKGS=()
for arg in "$@"; do
    case "$arg" in
        --skip-build) SKIP_BUILD=true ;;
        -h|--help)
            sed -n '2,11p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        -*)
            echo -e "${RED}[ERROR] Unknown option: $arg${NC}"
            exit 2
            ;;
        *) REQUESTED_PKGS+=("$arg") ;;
    esac
done

# --- Checks ------------------------------------------------------------------
if [ ! -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]; then
    echo -e "${RED}[ERROR] ROS 2 ${ROS_DISTRO_NAME} not found in /opt/ros/${ROS_DISTRO_NAME}${NC}"
    exit 1
fi

if [ ! -f "$REPO_ROOT/.clang-tidy" ]; then
    echo -e "${RED}[ERROR] .clang-tidy not found at $REPO_ROOT${NC}"
    exit 1
fi

# ROS setup scripts reference unset variables, so source them with nounset off
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u

for tool in colcon clang-tidy ament_clang_tidy; do
    if ! command -v "$tool" > /dev/null 2>&1; then
        echo -e "${RED}[ERROR] '$tool' not found.${NC}"
        echo "Install with: sudo apt-get install -y clang-tidy ros-${ROS_DISTRO_NAME}-ament-clang-tidy python3-colcon-common-extensions"
        exit 1
    fi
done

# --- Fresh build -------------------------------------------------------------
if [ "$SKIP_BUILD" = false ]; then
    echo -e "${BLUE}=== Cleaning previous build (build/ install/ log/) ... ===${NC}"
    rm -rf build install log

    echo -e "${BLUE}=== Building ROS2 packages ... ===${NC}"
    export CPR_USE_SYSTEM_CURL=ON
    export CXXFLAGS="-Werror"
    if colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON; then
        echo -e "${GREEN}[OK] Build succeeded${NC}"
    else
        echo -e "${RED}[FAILED] Build step triggered warnings or errors${NC}"
        exit 1
    fi
    # -Werror is only for the build; don't leak it into anything run afterwards
    unset CXXFLAGS
else
    echo -e "${YELLOW}=== Skipping build, reusing existing build/ ===${NC}"
    if [ ! -d build ]; then
        echo -e "${RED}[ERROR] build/ does not exist, run without --skip-build first${NC}"
        exit 1
    fi
fi

# --- Select packages ---------------------------------------------------------
if [ ${#REQUESTED_PKGS[@]} -gt 0 ]; then
    PKGS="${REQUESTED_PKGS[*]}"
else
    # name<TAB>path<TAB>(type), skip anything with a deps/ folder in its path
    # (paths look like src/rover/deps/cpr when listed from the workspace root)
    PKGS=$(colcon list | awk -F'\t' '$2 !~ /(^|\/)deps\// {print $1}')
fi

echo -e "${BLUE}=== Running clang-tidy on: ===${NC}"
echo "$PKGS" | tr ' ' '\n' | sed 's/^/  - /'

# --- clang-tidy --------------------------------------------------------------
: > "$LOG_FILE"
FAILED_PKGS=()
SKIPPED_PKGS=()

for pkg in $PKGS; do
    if [ ! -f "build/$pkg/compile_commands.json" ]; then
        # msgs-only or pure Python packages have no compile_commands.json
        SKIPPED_PKGS+=("$pkg")
        continue
    fi

    echo -e "${BLUE}--- $pkg ---${NC}"
    ament_clang_tidy --config "$REPO_ROOT/.clang-tidy" "build/$pkg" 2>&1 | tee -a "$LOG_FILE"
    # PIPESTATUS[0] is ament_clang_tidy's own exit code (must be read right after the pipe)
    tidy_status=${PIPESTATUS[0]}
    if [ "$tidy_status" -ne 0 ]; then
        FAILED_PKGS+=("$pkg")
        echo -e "${RED}[FAIL] $pkg${NC}"
    else
        echo -e "${GREEN}[OK] $pkg${NC}"
    fi
done

# --- Summary -----------------------------------------------------------------
echo
echo -e "${BLUE}=== Summary ===${NC}"

if [ ${#SKIPPED_PKGS[@]} -gt 0 ]; then
    echo -e "${YELLOW}Skipped (no compile_commands.json): ${SKIPPED_PKGS[*]}${NC}"
fi

if [ ${#FAILED_PKGS[@]} -gt 0 ]; then
    echo -e "${RED}Packages with findings: ${FAILED_PKGS[*]}${NC}"
    echo
    echo "Findings per check (full output in $LOG_FILE):"
    grep "error:" "$LOG_FILE" \
        | sed -n 's/.*\[\([a-zA-Z0-9.,-]*\),-warnings-as-errors\].*/\1/p' \
        | sort | uniq -c | sort -rn
    exit 1
fi

echo -e "${GREEN}[SUCCESS] clang-tidy found no issues${NC}"
exit 0