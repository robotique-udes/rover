#!/usr/bin/env python3

import subprocess
from http.server import HTTPServer, SimpleHTTPRequestHandler
import json

PASSWORD = "roverrover"

def kill_ros2():
    subprocess.run(
        ['sudo', '-S', 'pkill', '-f', 'ros2'],
        input=f'{PASSWORD}\n',
        capture_output=True,
        text=True
    )
    return True, "Killed all ros2 processes"

def relaunch_rover():
    try:
        subprocess.Popen(['ros2', 'launch', 'rover_msgs', 'rover.launch.py'])
        return True, "Rover relaunched"
    except Exception as e:
        return False, str(e)

class Handler(SimpleHTTPRequestHandler):
    def do_POST(self):
        if self.path == '/kill_teleop':
            success, message = kill_ros2()
            self._respond(success, message)
        elif self.path == '/relaunch_teleop':
            success, message = relaunch_rover()
            self._respond(success, message)
        else:
            self.send_response(404)
            self.end_headers()

    def do_GET(self):
        # Serve the HTML file
        if self.path == '/' or self.path == '/index.html':
            self.path = '/index.html'
        return SimpleHTTPRequestHandler.do_GET(self)

    def _respond(self, success, message):
        body = json.dumps({"success": success, "message": message}).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', len(body))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):
        print(f"[server] {format % args}")

if __name__ == '__main__':
    port = 8080
    print(f"Serving on http://0.0.0.0:{port}")
    HTTPServer(('0.0.0.0', port), Handler).serve_forever()