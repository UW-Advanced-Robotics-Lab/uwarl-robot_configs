#!/usr/bin/env zsh
source "$HOME/uwarl-robot_configs/scripts/common.sh"

function find_file_or_prompt_for_selections() {
    # find bag file:
    local placeholder_name=$1 # returns
    local search_directory=$2 #~/.ros/bag_replay_recorder_files
    local bag_file_pattern=$3 #"$file_prefix*.bag"
    
    ic_title "Finding $bag_file_pattern:"
    local bag_file_names=($(find $search_directory -type f -name $bag_file_pattern -exec basename {} \;))

    # filter selections:
    local N=${#bag_file_names[@]}
    if [ $N = 1 ]; then
        ic " > Found: ${bag_file_names[@]}"
        selected_file=$bag_file_names
    elif [ $N -gt 1 ]; then
        ic_wrn " > Multiple files found for pattern: $bag_file_pattern @ $search_directory"
        ic_wrn " > Please select the listed index to play:"
        select selected_file in "${bag_file_names[@]}"; do
            if [ $REPLY -gt $N ]; then
                ic_err "Invalid option: ($REPLY)" && exit 1
            else
                ic " > Selected: ($REPLY): $selected_file"
                break
            fi
        done
    else
        ic_err " x- No file found for pattern: $bag_file_pattern @ $search_directory"
        exit 1
    fi
    # indirect:
    set -A "$placeholder_name" "${selected_file[@]}"

}

function find_all_files() {
    # find bag file:
    local placeholder_name=$1
    local search_directory=$2 #~/.ros/bag_replay_recorder_files
    local bag_file_pattern=$3 #"$file_prefix*.bag"
    ic " > Finding $bag_file_pattern:"
    reply=($(find $search_directory -type f -name $bag_file_pattern -exec basename {} \;))
    # indirect:
    set -A "$placeholder_name" "${reply[@]}"
}

function find_all_directories() {
    # find bag file:
    local placeholder_name=$1 # returns
    local search_directory=$2 #~/.ros/bag_replay_recorder_files
    local bag_file_pattern=$3 #"session_*"
    ic " > Finding $bag_file_pattern:"
    reply=($(find $search_directory -type d -name $bag_file_pattern -exec basename {} \;))
    # indirect:
    set -A "$placeholder_name" "${reply[@]}"
}