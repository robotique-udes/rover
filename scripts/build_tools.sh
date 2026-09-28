#!/bin/bash

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
readonly SRC_DIR="$WORKSPACE_DIR/src"

readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly NC='\033[0m'

print_status() {
    local color=$1
    local message=$2

    printf '%b%s%b\n' "$color" "$message" "$NC"
}

get_build_jobs() {
    if [[ "${WORKER_QUANTITY:-}" =~ ^[1-9][0-9]*$ ]]; then
        printf '%d' "$WORKER_QUANTITY"
    else
        local jobs=$(( $(nproc) / 2 ))
        (( jobs < 1 )) && jobs=1
        printf '%d' "$jobs"
    fi
}

_build_all_packages() {
    local jobs
    jobs=$(get_build_jobs)

    print_status "$BLUE" "Building all packages with $jobs parallel workers..."

    colcon build \
        --parallel-workers "$jobs" \
        --symlink-install \
        --cmake-args \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
}

_build_selected_packages() {
    local packages=("$@")

    if [[ ${#packages[@]} -eq 0 ]]; then
        return 0
    fi

    local jobs
    jobs=$(get_build_jobs)

    print_status "$BLUE" "Building selected packages with $jobs parallel workers..."

    colcon build \
        --packages-above-and-dependencies "${packages[@]}" \
        --allow-overriding rover_msgs \
        --parallel-workers "$jobs" \
        --symlink-install \
        --cmake-args \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
}

b() {
    pushd "$WORKSPACE_DIR" > /dev/null || {
        print_status "$RED" "Failed to access workspace directory: $WORKSPACE_DIR"
        return 1
    }

    _build_all_packages
    local build_result=$?

    popd > /dev/null
    return "$build_result"
}

bs() {
    if [[ $# -eq 0 ]]; then
        print_status "$YELLOW" "Usage: bs package1 [package2 ...]"
        return 1
    fi

    pushd "$WORKSPACE_DIR" > /dev/null || {
        print_status "$RED" "Failed to access workspace directory: $WORKSPACE_DIR"
        return 1
    }

    print_status "$GREEN" "Building selected packages: $*"

    _build_selected_packages "$@"
    local build_result=$?

    popd > /dev/null
    return "$build_result"
}

clean() {
    pushd "$WORKSPACE_DIR" > /dev/null || {
        print_status "$RED" "Failed to access workspace directory: $WORKSPACE_DIR"
        return 1
    }

    print_status "$YELLOW" "Cleaning workspace..."

    rm -rf "${WORKSPACE_DIR}"/{build,install,log}

    _build_all_packages
    local build_result=$?

    popd > /dev/null
    source ~/.bashrc
    return "$build_result"
}

list_packages() {
    print_status "$BLUE" "Available packages in workspace:"

    find "$SRC_DIR" -name "package.xml" -exec dirname {} \; 2>/dev/null |
        while read -r package_dir; do
            local package_name
            package_name=$(grep -oP '<name>\K[^<]+' "$package_dir/package.xml" 2>/dev/null)

            if [[ -n "$package_name" ]]; then
                local relative_dir
                relative_dir=$(realpath --relative-to="$SRC_DIR" "$package_dir")
                printf '   %s (%s)\n' "$package_name" "$relative_dir"
            fi
        done
}

_build_select_completion() {
    pushd "$WORKSPACE_DIR" > /dev/null || {
        print_status "$RED" "Failed to access workspace directory: $WORKSPACE_DIR"
        return 1
    }

    local cur="${COMP_WORDS[COMP_CWORD]}"
    local packages

    packages=$(colcon list --names-only 2>/dev/null)

    COMPREPLY=(
        $(compgen -W "$packages" -- "$cur")
    )
    popd > /dev/null
}

complete -F _build_select_completion bs

b_help() {
    cat << 'EOF'
ROS 2 Build Tools

Available commands:
  b                  Build all packages
  bs package1 ...    Build selected packages and their dependencies/dependents
  clean              Clean workspace, rebuild everything and source bashrc
  list_packages      List all available packages
  b_help             Show this help message

Build workers:

  By default, builds use half of the available CPU threads.

  Set WORKER_QUANTITY to override the default:

    export WORKER_QUANTITY=4

  The value must be a positive integer. Invalid values are ignored
  and the default (half of the available CPU threads) is used.

Examples:
  b
  bs rover_can
  bs rover_gui rover_joy
  clean
EOF
}