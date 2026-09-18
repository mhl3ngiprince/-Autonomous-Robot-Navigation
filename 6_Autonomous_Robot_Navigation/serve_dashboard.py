"""Serve the navigation dashboard as a real web page.

A ROS2 robot's nodes speak DDS, not HTTP, so this small server hosts the
operator dashboard as a web page. It reads ``robot_status.json`` (if present)
and the local ``maps/`` directory.

    python serve_dashboard.py            # http://127.0.0.1:8006
    python serve_dashboard.py --port 8080
"""
from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import dashboard


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path not in ("/", "/dashboard", "/index.html"):
            self.send_error(404, "Not found")
            return
        body = dashboard.render().encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass  # keep the console clean; the server prints its own line


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8006)
    args = p.parse_args(argv)
    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"Navigation dashboard on http://{args.host}:{args.port}  (Ctrl+C to stop)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        srv.server_close()


if __name__ == "__main__":
    main()
