#!/bin/bash
# Get the port number from ROS parameter server or default to 8000
PORT=$(rosparam get /http_server/port 2>/dev/null)
if [ -z "$PORT" ]; then
    PORT=8000
fi
# Change directory to where the HTML file is located
HTML_DIR="/home/stu/web_project_2/src/my_rosbridge_pkg/web"
cd "$HTML_DIR" || exit
# Start the HTTP server
python3 -m http.server "$PORT"
