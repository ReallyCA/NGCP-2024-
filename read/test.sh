#!/bin/bash
# TEST

output=$(./read)

IFS=',' read -r lat long <<< "$output"

echo "Latitude: $lat"
echo "Longitude: $long"
