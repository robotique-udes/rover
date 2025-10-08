#!/bin/bash

readonly WORKSPACE_DIR="/home/Rover/ros2_ws"
readonly SRC_DIR="${WORKSPACE_DIR}/src"
readonly LOG_BASE="/home/Rover/.colcon/log"

readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly NC='\033[0m'

print_status() {
    local color=$1
    local message=$2
    echo -e "${color}${message}${NC}"
}

smart_build() {
    local changed_packages=()
    local all_packages=()
    
    pushd "$WORKSPACE_DIR" > /dev/null || {
        print_status "$RED" "Failed to access workspace directory: $WORKSPACE_DIR"
        return 1
    }
    
    print_status "$BLUE" "Detecting changes in workspace..."

    while IFS= read -r -d '' package_xml; do
        if [[ -f "$package_xml" ]]; then
            local package_dir
            package_dir=$(dirname "$package_xml")
            local package_name
            package_name=$(grep -oP '<name>\K[^<]+' "$package_xml" 2>/dev/null)
            if [[ -n "$package_name" && "$package_name" != "unknown" ]]; then
                all_packages+=("$package_name:$package_dir")
            fi
        fi
    done < <(find "$SRC_DIR" -name "package.xml" -print0 2>/dev/null)

    print_status "$BLUE" "Found ${#all_packages[@]} packages in workspace"

    if ! git -C "$SRC_DIR" rev-parse --git-dir > /dev/null 2>&1; then
        print_status "$YELLOW" "No git repository found under $SRC_DIR. Building all packages..."
        _build_all_packages 3
        popd > /dev/null
        return $?
    fi

    local changed_files=""
    if git -C "$SRC_DIR" rev-parse --verify HEAD >/dev/null 2>&1; then
        changed_files+=$(git -C "$SRC_DIR" diff --name-only --relative HEAD -- . 2>/dev/null)
    else
        changed_files+=$(git -C "$SRC_DIR" ls-files -- .)
    fi

    changed_files+=$'\n'$(git -C "$SRC_DIR" diff --name-only --cached -- . 2>/dev/null)
    changed_files+=$'\n'$(git -C "$SRC_DIR" diff --name-only -- . 2>/dev/null)
    changed_files+=$'\n'$(git -C "$SRC_DIR" ls-files --others --exclude-standard -- . 2>/dev/null)

    changed_files=$(printf '%s\n' "$changed_files" | sed '/^$/d' | sort -u)

    if [[ -z "$changed_files" ]]; then
        print_status "$GREEN" "No changes detected under $SRC_DIR. Skipping build."
        popd > /dev/null
        return 0
    fi

    print_status "$BLUE" "Changed files (relative to src/):"
    echo "$changed_files" | sed 's/^/   /'

    for package_info in "${all_packages[@]}"; do
        IFS=':' read -r package_name package_dir <<< "$package_info"
        local rel_pkg_dir
        rel_pkg_dir=$(realpath --relative-to="$SRC_DIR" "$package_dir" 2>/dev/null || echo "$package_dir")

        if echo "$changed_files" | grep -E -q "^${rel_pkg_dir}(/|$)"; then
            changed_packages+=("$package_name")
        fi
    done

    if [[ ${#changed_packages[@]} -gt 0 ]]; then
        IFS=" " read -r -a changed_packages <<< "$(printf '%s\n' "${changed_packages[@]}" | sort -u | tr '\n' ' ')"
    fi

    if [[ ${#changed_packages[@]} -eq 0 ]]; then
        print_status "$YELLOW" "No ROS 2 packages affected by changes under $SRC_DIR. Building all packages..."
        _build_all_packages 3
    else
        print_status "$GREEN" "Changed packages: ${changed_packages[*]}"
        _build_selected_packages "${changed_packages[@]}"
    fi

    local build_result=$?
    popd > /dev/null

    if [[ $build_result -eq 0 ]]; then
        print_status "$GREEN" "Build completed successfully."
    else
        print_status "$RED" "Build failed."
    fi

    return $build_result
}

_build_all_packages() {
    local workers=${1:-3}
    print_status "$BLUE" "Building all packages with $workers parallel workers..."
    colcon --log-base "$LOG_BASE" build --parallel-workers "$workers" --symlink-install
}

_build_selected_packages() {
    local packages=("$@")
    local num_packages=${#packages[@]}
    local workers
    if [[ $num_packages -eq 1 ]]; then
        workers=4
        print_status "$BLUE" "Building single package '${packages[0]}' with $workers parallel workers..."
    elif [[ $num_packages -le 3 ]]; then
        workers=4
        print_status "$BLUE" "Building $num_packages packages with $workers parallel workers..."
    else
        workers=3
        print_status "$BLUE" "Building $num_packages packages with $workers parallel workers..."
    fi
    colcon --log-base "$LOG_BASE" build --packages-select "${packages[@]}" --parallel-workers "$workers" --symlink-install
}

build_all() {
    pushd "$WORKSPACE_DIR" > /dev/null || {
        print_status "$RED" "Failed to access workspace directory: $WORKSPACE_DIR"
        return 1
    }
    _build_all_packages 3
    local build_result=$?
    popd > /dev/null
    return $build_result
}

build_select() {
    if [[ $# -eq 0 ]]; then
        print_status "$YELLOW" "Usage: build_select package1 [package2 ...]"
        return 1
    fi
    pushd "$WORKSPACE_DIR" > /dev/null || {
        print_status "$RED" "Failed to access workspace directory: $WORKSPACE_DIR"
        return 1
    }
    print_status "$GREEN" "Building selected packages: $*"
    print_status "$BLUE" "Building with 4 parallel workers..."
    colcon --log-base "$LOG_BASE" build --packages-select "$@" --parallel-workers 4 --symlink-install
    local build_result=$?
    popd > /dev/null
    return $build_result
}

clean_workspace() {
    pushd "$WORKSPACE_DIR" > /dev/null || {
        print_status "$RED" "Failed to access workspace directory: $WORKSPACE_DIR"
        return 1
    }
    print_status "$YELLOW" "Cleaning workspace..."
    rm -rf "${WORKSPACE_DIR}"/{build,install,log}
    mkdir -p "$LOG_BASE"
    print_status "$BLUE" "Rebuilding all packages..."
    _build_all_packages 3
    local build_result=$?
    popd > /dev/null
    return $build_result
}

list_packages() {
    print_status "$BLUE" "Available packages in workspace:"
    find "$SRC_DIR" -name "package.xml" -exec dirname {} \; 2>/dev/null | while read -r package_dir; do
        local package_name
        package_name=$(grep -oP '<name>\K[^<]+' "$package_dir/package.xml" 2>/dev/null)
        if [[ -n "$package_name" ]]; then
            local relative_dir
            relative_dir=$(realpath --relative-to="$SRC_DIR" "$package_dir")
            echo "   $package_name ($relative_dir)"
        fi
    done
}

show_help() {
    cat << 'EOF'
ROS2 Smart Build Tools

Available commands:
  b, smart_build     - Smart build (detects changed packages via git under src/)
  ba, build_all      - Build all packages (force full build)
  bs, build_select   - Build specific packages: bs package1 package2
  clean              - Clean workspace and rebuild everything
  list_packages      - List all available packages
  build_help         - Show this help message

Examples:
  b                  # Build only changed packages
  ba                 # Build everything
  bs my_robot nav    # Build only my_robot and nav packages
  bu navigation      # Build navigation package and all its dependencies
  clean              # Clean and rebuild everything

The smart build (b) checks for changes relative to the Git repository at src/.
EOF
}

alias b='smart_build'
alias ba='build_all'
alias bs='build_select'
alias clean='clean_workspace'
alias build_help='show_help'
