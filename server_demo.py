from http.server import HTTPServer, BaseHTTPRequestHandler
from os import curdir
from os.path import join as pjoin
import json
import datetime

class S(BaseHTTPRequestHandler):
    """
    Server with get, post, and head. Post writes to a JSON file storing the 
    sensor id, type, time, and data. 

    GET returns the entire JSON data file; client should filter by 
    sensor id or by time etc. 

    POST can be sent with or without a date-time. Either way, the server
    time is recorded. 

    Basically copied from: https://gist.github.com/bradmontgomery/2219997
    """
    #set directory for all sensor readings
    sensor_path = pjoin(curdir, 'sensor.json')

    def _set_headers(self):
        """
        Set headers
        """
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()

    def _html(self, message):
        """
        This just generates an HTML document that includes `message`
        in the body. Override, or re-write this do do more interesting stuff.
        """
        content = f"<html><body><h1>{message}</h1></body></html>"
        return content.encode("utf8")  # NOTE: must return a bytes object!

    def do_HEAD(self):
        self._set_headers()

    def do_GET(self):
        """
        Return the entire json file containing the json. It will fail if 
        no sensor data has been posted / there is no json. 
        """
        if self.path == '/sensor.json':
            with open(self.sensor_path) as fh:
                self.send_response(200)
                self.send_header('Content-type', 'text/json')
                self.end_headers()
                self.wfile.write(fh.read().encode())
        else:
            self._set_headers()
            self.wfile.write(self._html("hola!"))

    def do_POST(self):
        """
        Read the input data and save to a json file. The path should be /sensor.json
        for sensor data. 
        
        Tries to write to an existing json file if one exists (reads from its 'data'
        key and appends to the list contained within). If anything fails it makes a 
        new json file instead. 
        """
        if self.path == '/sensor.json':
            length = self.headers['content-length']
            data = self.rfile.read(int(length))
            params = data.decode().split("&")
            new_data = {}
            for p in params:
                kv = p.split("=")
                new_data[kv[0]] = kv[1]
            new_data['server_time'] = str(datetime.datetime.now())

            try:
                with open(self.sensor_path) as fh:
                    json_data = json.load(fh)
                    json_data['data'].append(new_data)
                    with open(self.sensor_path, 'w') as fh:
                        json.dump(json_data, fh)
            except:
                json_data = {'data': [new_data]}
                with open(self.sensor_path, 'w') as fh:
                    json.dump(json_data, fh)

        self._set_headers()
        self.wfile.write(bytes("<html><body><h1>POST!</h1></body></html>", "utf-8"))
        self.send_response(200)


def run(server_class=HTTPServer, handler_class=S, addr="localhost", port=8000):
    server_address = (addr, port)
    httpd = server_class(server_address, handler_class)

    print(f"Starting httpd server on {addr}:{port}")
    httpd.serve_forever()


if __name__ == "__main__":
    run()
