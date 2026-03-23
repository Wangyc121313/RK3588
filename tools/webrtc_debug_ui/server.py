#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import mimetypes
import os
import pathlib
import urllib.error
import urllib.parse
import urllib.request

from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer


ROOT = pathlib.Path(__file__).resolve().parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RK3588 WebRTC debug UI server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--zlm-host", default="127.0.0.1")
    parser.add_argument("--zlm-http-port", type=int, default=8080)
    parser.add_argument("--telemetry-path", default="")
    parser.add_argument("--default-app", default="live")
    parser.add_argument("--default-stream", default="camera")
    return parser.parse_args()


def read_last_telemetry_line(path: str) -> dict | None:
    if not path or not os.path.exists(path):
        return None

    with open(path, "rb") as handle:
        handle.seek(0, os.SEEK_END)
        position = handle.tell()
        if position == 0:
            return None

        buffer = bytearray()
        while position > 0:
            position -= 1
            handle.seek(position)
            byte = handle.read(1)
            if byte == b"\n" and buffer:
                break
            if byte != b"\n":
                buffer.extend(byte)

    if not buffer:
        return None

    try:
        line = buffer[::-1].decode("utf-8")
        return json.loads(line)
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None


class DebugUiHandler(SimpleHTTPRequestHandler):
    server_version = "RK3588DebugUI/1.0"

    def __init__(self, *args, directory: str | None = None, **kwargs):
        super().__init__(*args, directory=directory or str(ROOT), **kwargs)

    def log_message(self, fmt: str, *args) -> None:
        print("[debug-ui] " + fmt % args)

    def do_OPTIONS(self) -> None:
        self.send_response(HTTPStatus.NO_CONTENT)
        self._send_cors_headers()
        self.end_headers()

    def do_GET(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/config":
            self._handle_config()
            return
        if parsed.path == "/api/telemetry/latest":
            self._handle_latest_telemetry()
            return
        if parsed.path in {"/", "/index.html"}:
            self.path = "/index.html"
        super().do_GET()

    def do_POST(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/webrtc":
            self._proxy_webrtc(parsed.query)
            return
        self.send_error(HTTPStatus.NOT_FOUND, "unknown API")

    def end_headers(self) -> None:
        self._send_cors_headers()
        super().end_headers()

    def guess_type(self, path: str) -> str:
        if path.endswith(".js"):
            return "application/javascript; charset=utf-8"
        if path.endswith(".json"):
            return "application/json; charset=utf-8"
        return mimetypes.guess_type(path)[0] or "application/octet-stream"

    def _send_cors_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Cache-Control", "no-store")

    def _write_json(self, code: int, payload: dict) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _handle_config(self) -> None:
        app = self.server.debug_default_app
        stream = self.server.debug_default_stream
        payload = {
            "zlm_host": self.server.debug_zlm_host,
            "zlm_http_port": self.server.debug_zlm_http_port,
            "default_app": app,
            "default_stream": stream,
            "telemetry_enabled": bool(self.server.debug_telemetry_path),
            "default_play_url": f"rtc://{self.server.debug_zlm_host}:{os.environ.get('RK3588_WEBRTC_RTC_PORT', '8000')}/{app}/{stream}",
        }
        self._write_json(HTTPStatus.OK, payload)

    def _handle_latest_telemetry(self) -> None:
        snapshot = read_last_telemetry_line(self.server.debug_telemetry_path)
        if snapshot is None:
            self._write_json(HTTPStatus.OK, {"ok": False, "snapshot": None})
            return
        self._write_json(HTTPStatus.OK, {"ok": True, "snapshot": snapshot})

    def _proxy_webrtc(self, query: str) -> None:
        length = int(self.headers.get("Content-Length", "0") or "0")
        body = self.rfile.read(length)
        upstream = (
            f"http://{self.server.debug_zlm_host}:{self.server.debug_zlm_http_port}"
            f"/index/api/webrtc?{query}"
        )
        forwarded_host = self.headers.get("Host", "")
        upstream_headers = {"Content-Type": "text/plain;charset=utf-8"}
        if forwarded_host:
            upstream_headers["Host"] = forwarded_host
        request = urllib.request.Request(
            upstream,
            data=body,
            method="POST",
            headers=upstream_headers,
        )

        try:
            with urllib.request.urlopen(request, timeout=8.0) as response:
                payload = response.read()
                self.send_response(response.status)
                self.send_header("Content-Type", response.headers.get_content_type() + "; charset=utf-8")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
        except urllib.error.HTTPError as exc:
            payload = exc.read() or json.dumps({"code": -1, "msg": str(exc)}).encode("utf-8")
            self.send_response(exc.code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
        except Exception as exc:  # pylint: disable=broad-except
            self._write_json(HTTPStatus.BAD_GATEWAY, {"code": -1, "msg": f"proxy failed: {exc}"})


def main() -> int:
    args = parse_args()
    server = ThreadingHTTPServer((args.host, args.port), DebugUiHandler)
    server.debug_zlm_host = args.zlm_host
    server.debug_zlm_http_port = args.zlm_http_port
    server.debug_telemetry_path = args.telemetry_path
    server.debug_default_app = args.default_app
    server.debug_default_stream = args.default_stream

    print(
        f"RK3588 WebRTC debug UI listening on http://{args.host}:{args.port} "
        f"(ZLM http {args.zlm_host}:{args.zlm_http_port}, telemetry={args.telemetry_path or 'disabled'})"
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())