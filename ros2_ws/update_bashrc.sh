#!/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Define the path to the custom .bashrc additions file
CUSTOM_FILE="$SCRIPT_DIR/.bashrc"
BASHRC="$HOME/.bashrc"

# Check if the file exists
if [ ! -f "$CUSTOM_FILE" ]; then
    echo "Error: $CUSTOM_FILE not found in $SCRIPT_DIR!"
    exit 1
fi

# Create a clean grep pattern that escapes special characters in the file path
ESCAPED_CUSTOM_FILE=$(printf "%q" "$CUSTOM_FILE")

# Check if .bashrc already sources the file
if ! grep -Fxq "if [ -f \"$CUSTOM_FILE\" ]; then" "$BASHRC"; then
    echo -e "\n# Source custom .bashrc additions\nif [ -f \"$CUSTOM_FILE\" ]; then\n    source \"$CUSTOM_FILE\"\nfi" >> "$BASHRC"
    echo "Added source line to $BASHRC."
else
    echo "$BASHRC already sources $CUSTOM_FILE."
fi

# Optionally, validate if sourcing was successful
if source "$BASHRC"; then
    echo ".bashrc has been reloaded."
else
    echo "Error reloading .bashrc."
fi
