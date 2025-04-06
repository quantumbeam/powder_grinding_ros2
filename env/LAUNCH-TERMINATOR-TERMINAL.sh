#!/bin/bash

################################################################################

# Make a backup of the user's original Terminator configuration.
mkdir -p ~/.config/terminator/

# Update the user's current Terminator configuration with the project's one.
cp ./terminator/config ~/.config/terminator/config

if [ ! -f ~/.config/terminator/config.backup ]; then
  cp ~/.config/terminator/config ~/.config/terminator/config.backup
fi

################################################################################

# Run Terminator with the project's custom layout.
terminator -m -l onolab-split &
sleep 1
