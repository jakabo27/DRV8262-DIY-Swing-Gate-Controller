"""Static server for the diagram page, plus POST /save?name=<file>.svg -> out/<file>.svg"""
import http.server
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(ROOT, "out")
os.makedirs(OUT, exist_ok=True)


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **k):
        super().__init__(*a, directory=ROOT, **k)

    def do_POST(self):
        if not self.path.startswith("/save?name="):
            self.send_error(404)
            return
        name = os.path.basename(self.path.split("=", 1)[1])
        if not name.endswith(".svg"):
            self.send_error(400)
            return
        data = self.rfile.read(int(self.headers["Content-Length"]))
        with open(os.path.join(OUT, name), "wb") as f:
            f.write(data)
        self.send_response(200)
        self.end_headers()
        self.wfile.write(b"saved %d bytes" % len(data))


http.server.ThreadingHTTPServer(("127.0.0.1", int(sys.argv[1])), Handler).serve_forever()
