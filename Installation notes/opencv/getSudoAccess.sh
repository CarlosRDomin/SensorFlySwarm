#!/bin/bash
# This script provides sudo access: if you already have sudo access, it does nothing; otherwise, it prompts the user to enter the sudo password.

test="MyTest"
if [ "$(echo '' | sudo -S echo $test 2>/dev/null)" != $test ]; then # Redirect stderr to /dev/null so potential password error is not displayed
    echo "Sudo is not granted! Please input sudo password..."
    pwd=$(osascript -e 'Tell application "System Events" to display dialog "Enter sudo password:" default answer "" with hidden answer' -e 'text returned of result' 2>/dev/null)
    if [ "$(echo $pwd | sudo -S echo $test 2>/dev/null)" != $test ]; then
        echo "error: Can't get sudo access, incorrect password."
    else
        echo "Password is correct, sudo access gained!"
    fi
else
    echo "Sudo access was already granted! :)"
fi
