#!/bin/bash

# Mapviz doesn't work on aarch64
if [ ! $(uname -m) == "aarch64" ]; then
	# Check if the mapproxy container is already running
	if [ $(docker ps | grep danielsnider/mapproxy | wc -l) -eq 0 ]; then
		# https://github.com/danielsnider/docker-mapproxy-googlemaps/tree/master
		echo "Starting the mapproxy container..."
		docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
	else
		echo "Docker container is already running"
	fi
else
	echo "Mapproxy is not supported on aarch64 architecture."
fi
