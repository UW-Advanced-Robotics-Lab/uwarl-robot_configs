## kill processor:
alias kill-ros="ps aux  | grep -e ros | awk '{print $2}' | xargs -i -exec kill -9 {}"
alias catkin_build_noetic="catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3" # for noetic on u20

## Git Batch Shortcuts:
alias git-status-all="find . -maxdepth 1 -type d -execdir sh -c 'cd {}; echo \"[Git {}]\"; git status; git remote -v; echo \"----------\n\"' \;"
alias git-push-all="find . -maxdepth 1 -type d -execdir sh -c 'cd {}; echo \"[Pushing {}]\"; git push; echo \"----------\n\"' \;"
alias git-fetch-all="find . -maxdepth 1 -type d -execdir sh -c 'cd {}; echo \"[Fetching {}]\"; git fetch; echo \"----------\n\"' \;"
alias git-pull-all="find . -maxdepth 1 -type d -execdir sh -c 'cd {}; echo \"[Fetching {}]\"; git pull; echo \"----------\n\"' \;"
