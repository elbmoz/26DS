"""Temporarily expose /root/ball_tests over read-only HTTP."""

import http.server
import os
import socketserver
import threading

from maix import app, time


HOST = "0.0.0.0"
PORT = 8080
RESULT_ROOT = "/root/ball_tests"


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, format_text, *args):
        print("HTTP:", format_text % args)


def main():
    os.chdir(RESULT_ROOT)
    server = socketserver.TCPServer((HOST, PORT), QuietHandler)
    worker = threading.Thread(target=server.serve_forever)
    worker.daemon = True
    worker.start()
    print("results server: http://10.16.6.1:{}/".format(PORT))
    try:
        while not app.need_exit():
            time.sleep_ms(100)
    finally:
        server.shutdown()
        server.server_close()
        print("results server stopped")


if __name__ == "__main__":
    main()
