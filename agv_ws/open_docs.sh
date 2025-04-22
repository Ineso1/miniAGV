#!/bin/bash

# Define the path to the previously generated documentation directory
DOC_DIR="$PWD/html"

# Check if the documentation directory exists, and remove it if it does
if [[ -d "$DOC_DIR" ]]; then
    echo "Removing previous documentation..."
    rm -rf "$DOC_DIR"
fi

# Run Doxygen to generate new documentation
doxygen Doxyfile

# Define the path to your newly generated Doxygen documentation
DOC_PATH="$PWD/html/index.html"

# Check if the file exists
if [[ -f "$DOC_PATH" ]]; then
    # Open the documentation in the default web browser
    xdg-open "$DOC_PATH" || open "$DOC_PATH" || start "$DOC_PATH"
else
    echo "Documentation not found at $DOC_PATH"
fi
