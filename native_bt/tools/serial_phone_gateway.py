"""Read-only phone access to the existing viewer, behind private Tailscale Serve.

Never opens serial ports. Only fixed viewer GET routes are proxied. A private
URL token is required, and the local viewer's control token is never forwarded.
"""
import argparse
import hmac
import http.server
import json
import os
import re
import secrets
import urllib.error
import urllib.request
from pathlib import Path
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[1]
ROUTES = {'/', '/dashboard.js', '/dashboard.css', '/state', '/events', '/captures', '/export'}
HIDDEN = ('release', 'resume', 'follow-capture', 'clear', 'archive-toggle', 'archive-panel')


def remote_asset(path, body, prefix):
    if path == '/':
        body = re.sub(rb'((?:href|src)=")/', lambda m: m[1] + prefix.encode() + b'/', body)
        style = '<style>' + ','.join('#' + x for x in HIDDEN) + '{display:none!important}</style>'
        body = body.replace(b'</head>', style.encode() + b'</head>')
        body = body.replace(b'ESP32 / BLUETOOTH AUDIO', b'PRIVATE TAILSCALE / PHONE VIEW')
    elif path == '/dashboard.js':
        body, count = re.subn(rb"const TOKEN='[^']*';", b"const TOKEN='';", body)
        if count != 1:
            raise ValueError('Viewer control token declaration changed; refusing remote script')
        body = body.replace(b"fetch('/", b"fetch('" + prefix.encode() + b'/')
        body = body.replace(b'async function command(action,extra={}){',
                            b"async function command(action,extra={}){toast('Phone view is read-only');return;")
    return body


def make_handler(token, hostname):
    prefix = '/' + token
    allowed_hosts = {hostname, hostname + ':80', '127.0.0.1:8766'}

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, status, body, kind='text/plain; charset=utf-8'):
            self.send_response(status)
            for key, value in [('Content-Type', kind), ('Content-Length', str(len(body))),
                               ('Cache-Control', 'no-store'), ('Referrer-Policy', 'no-referrer'),
                               ('X-Content-Type-Options', 'nosniff'), ('X-Frame-Options', 'DENY')]:
                self.send_header(key, value)
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            if self.headers.get('Host') not in allowed_hosts:
                self.reply(403, b'Rejected host')
                return
            parts = urlsplit(self.path)
            segment, separator, suffix = parts.path.lstrip('/').partition('/')
            if not separator or not hmac.compare_digest(segment, token):
                self.reply(404, b'Private log link required')
                return
            path = '/' + suffix
            if path not in ROUTES:
                self.reply(404, b'Unknown log route')
                return
            try:
                url = 'http://127.0.0.1:8765' + path + ('?' + parts.query if parts.query else '')
                with urllib.request.urlopen(url, timeout=4) as response:
                    body = response.read(64 * 1024 * 1024 + 1)
                    if len(body) > 64 * 1024 * 1024:
                        raise ValueError('Viewer response too large')
                    kind = response.headers.get('Content-Type', 'application/octet-stream')
                self.reply(200, remote_asset(path, body, prefix), kind)
            except (OSError, ValueError, urllib.error.URLError):
                self.reply(502, b'Logging service unavailable; retry shortly')

        def do_POST(self):
            self.reply(405, b'Phone view is read-only')

        do_PUT = do_POST
        do_DELETE = do_POST
        do_PATCH = do_POST

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hostname', required=True)
    args = parser.parse_args()
    if not re.fullmatch(r'[a-z0-9-]+\.[a-z0-9-]+\.ts\.net', args.hostname):
        parser.error('Exact private Tailscale hostname required')
    state = ROOT / '_build/serial/phone-access.json'
    state.parent.mkdir(parents=True, exist_ok=True)
    previous = json.loads(state.read_text()) if state.exists() else {}
    token = previous.get('token') or secrets.token_urlsafe(32)
    if not re.fullmatch(r'[A-Za-z0-9_-]{43}', token):
        raise ValueError('Invalid private URL token')
    server = http.server.ThreadingHTTPServer(('127.0.0.1', 8766), make_handler(token, args.hostname))
    server.daemon_threads = True
    info = {'pid': os.getpid(), 'token': token, 'hostname': args.hostname,
            'url': 'http://' + args.hostname + '/' + token + '/',
            'listen': '127.0.0.1:8766', 'upstream': 'http://127.0.0.1:8765', 'read_only': True}
    state.write_text(json.dumps(info, indent=2) + '\n', encoding='utf-8', newline='\n')
    server.serve_forever()


if __name__ == '__main__':
    main()
