"""Phone gateway isolation and asset tests; no serial or Tailscale access."""
import http.client
import http.server
import argparse
import json
import threading
import time
import unittest
from unittest.mock import patch

from serial_phone_gateway import ROOT, make_handler, remote_asset
from serial_monitor_server import Monitor


class PhoneGatewayTests(unittest.TestCase):
    def test_existing_viewer_private_route_and_remote_command_rejection(self):
        args=argparse.Namespace(port='COM6',usb_serial='CA0ACAD67933EF1180C36117A04C7D1C',baud=115200,released=True,
                                state=str(ROOT/'_build/serial-tests/phone-state.json'),http_port=0,
                                phone_hostname='speaker.tailtest.ts.net')
        monitor=Monitor(args)
        thread=threading.Thread(target=monitor.run,daemon=True)
        with patch('serial_monitor_server.list_ports.comports') as ports:
            thread.start()
            try:
                end=time.monotonic()+3
                while 'url' not in monitor.data and time.monotonic()<end:time.sleep(.01)
                port=int(monitor.data['url'].split(':')[-1].strip('/'))
                for method,path,expected in [('GET','/',404),('GET','/'+monitor.phone_token+'/',200),
                                             ('GET','/'+monitor.phone_token+'/dashboard.js',200),
                                             ('GET','/'+monitor.phone_token+'/events?after=0',200),
                                             ('POST','/command',403)]:
                    connection=http.client.HTTPConnection('127.0.0.1',port)
                    connection.request(method,path,body='{"action":"resume"}' if method=='POST' else None,
                                       headers={'Host':args.phone_hostname,'Content-Type':'application/json',
                                                'X-Monitor-Token':monitor.token})
                    response=connection.getresponse();body=response.read()
                    self.assertEqual(response.status,expected)
                    if path.endswith('dashboard.js'):self.assertNotIn(monitor.token.encode(),body)
                    connection.close()
                self.assertEqual(monitor.control[1],'release')
                ports.assert_not_called()
            finally:
                monitor.stop.set();monitor.wake.set();thread.join(3)

    def test_real_assets_rebased_and_control_token_removed(self):
        assets = ROOT / 'tools/serial_dashboard'
        html = remote_asset('/', (assets / 'index.html').read_bytes(), '/private')
        script = remote_asset('/dashboard.js', (assets / 'dashboard.js').read_bytes().replace(b'__CONTROL_TOKEN__', b'sensitive-local-control'), '/private')
        self.assertIn(b'src="/private/dashboard.js"', html)
        self.assertIn(b'#release', html)
        self.assertNotIn(b'sensitive-local-control', script)
        self.assertNotIn(b"fetch('/events", script)
        self.assertIn(b"fetch('/private/events", script)
        self.assertIn(b'Phone view is read-only', script)

    def test_remote_mutations_and_unadmitted_routes_never_proxy(self):
        server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), make_handler('private', 'speaker.tailtest.ts.net'))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with patch('serial_phone_gateway.urllib.request.urlopen') as upstream:
                for method, path, host, expected in [
                    ('POST', '/private/command', 'speaker.tailtest.ts.net', 405),
                    ('GET', '/private/command', 'speaker.tailtest.ts.net', 404),
                    ('GET', '/state', 'speaker.tailtest.ts.net', 404),
                    ('GET', '/wrong/state', 'speaker.tailtest.ts.net', 404),
                    ('GET', '/private/state', 'attacker.example', 403)]:
                    connection = http.client.HTTPConnection('127.0.0.1', server.server_port)
                    connection.request(method, path, headers={'Host': host})
                    response = connection.getresponse()
                    self.assertEqual(response.status, expected)
                    response.read()
                    connection.close()
                upstream.assert_not_called()
        finally:
            server.shutdown()
            server.server_close()
            thread.join(2)


if __name__ == '__main__':
    unittest.main()
