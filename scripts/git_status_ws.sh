#!/usr/bin/env zsh
OUTPUT_STATUS_LOG_DIR="$ROS_CATKIN_WS/src/git_status_ws.log"

YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

ic () {
    echo -e "${CYAN}[UWARL_catkin_ws] ${NC} ${YELLOW} $1 ${NC}"
    echo "$1" >> $OUTPUT_STATUS_LOG_DIR
}


check_submodule_status(){
    echo "[Git Status Lastly Updated on $(date)]" > $OUTPUT_STATUS_LOG_DIR
    # update submodules:
    submodule_status_log=""
    ic "Indexing Submodules Recursively uwarl-robot_configs @ $ROS_CATKIN_WS/src >>--log-->> $OUTPUT_STATUS_LOG_DIR"
    echo "------------------------------------------------------------------------------------------------" >> $OUTPUT_STATUS_LOG_DIR
    echo "------------------------------------------------------------------------------------------------"
    
    i=0
    for dir in */ ; do
        i=$(( i + 1 ))
        if [ "$(ls -A $dir)" ]; then
            ic "[$i] $dir is loaded: "
            cd "$ROS_CATKIN_WS/src/$dir"
            # now check changes
            git_stats=$(git status --porcelain)
            git_remote=$(git remote -v)

            ic "   > $dir remote version: \n$git_remote"
            if [[ $(git status --porcelain | wc -l) -gt 0 ]]; then 
                ic "   > [!] $dir has changes: \n $git_stats"
            else   
                ic "   > [OK] $dir is up-to-date"
            fi 

            # git status
            #
            cd "$ROS_CATKIN_WS/src"
        else
            ic "[$i] $dir submodule is not loaded! "
        fi
        echo "------------------------------------------------------------------------------------------------" >> $OUTPUT_STATUS_LOG_DIR
        echo "------------------------------------------------------------------------------------------------"
    done
    
    ic "x- Done indexing submodules."

}

check_submodule_status



# alias git-status-all="find . -maxdepth 1 -type d -execdir sh -c 'cd {}; echo \"[Git {}]\"; git status; git remote -v; echo \"----------\n\"' \;"
# git-status-all > git-status-all.log