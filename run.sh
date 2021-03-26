#!/bin/bash

if ! [ -x "$(command -v tmux)" ]; then
  echo 'Error: tmux is not installed.' >&2
  exit 1
fi

[[ -z "$1" ]] && grep="" || grep="| grep $1"

tmux kill-session -t rosc &> /dev/null

tmux new -s rosc -d
tmux split-window -d -t rosc
tmux split-window -d -t rosc
#tmux select-layout even-vertical

tmux resize-pane -t rosc.0 -U 15 
tmux resize-pane -t rosc.2 -D 20 

tmux send-keys -t rosc.0 "morse run auto_warehouse auto_factory_paper.py" enter

tmux send-keys -t rosc.1 "sleep 2 && roslaunch --screen auto_smart_factory full_system_paper.launch $grep" enter

tmux send-keys -t rosc.2 "sleep 4 && roslaunch auto_smart_factory visualization.launch" enter

tmux a

# Exit with CTRL+B followed by & and cofirm with y
