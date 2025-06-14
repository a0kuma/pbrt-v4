#!/bin/sh

# Check if filename argument is provided
if [ -z "$1" ]; then
  echo "Usage: $0 filename.txt"
  exit 1
fi

# Extract base filename without extension
basename=$(basename "$1" .txt)

# Copy head.pbrt to the new filename with .pbrt extension
cp head.pbrt "$basename.pbrt"

# Append the contents of the .txt file to the .pbrt file
cat "$1" >> "$basename.pbrt"
