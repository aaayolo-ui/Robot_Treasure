"""Real HTTP handlers, fake clock, no serial connection."""
import json
import sys
import threading
import unittest
from pathlib import Path
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import Request, build_opener, ProxyHandler
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).with_name('assets')))
from console_server import SharedFeed, handler_for
from round_timer import RoundTimer
from test_round_timer import Clock
from test_auto_trials import frame


class ManualRoundApiTests(unittest.TestCase):
    def test_private_end_and_map_reset_hold_time_without_touching_serial(self):
        clock = Clock(); feed = SharedFeed(); feed.round_timer = RoundTimer(clock)
        server = ThreadingHTTPServer(('127.0.0.1', 0), handler_for(feed, 'offline-key'))
        thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
        opener = build_opener(ProxyHandler({}))
        def post(path, auth=False):
            headers = {'Content-Type': 'application/json'}
            if auth: headers['X-Desktop-Key'] = 'offline-key'
            req = Request('http://127.0.0.1:%d%s' % (server.server_port, path), data=b'{}', headers=headers)
            with opener.open(req, timeout=2) as response: return json.load(response)
        try:
            with patch('console_server.serial.Serial') as serial_factory:
                feed.parse_line(frame(feed, 1, 1, 10000, 2)); clock.advance(45)
                with self.assertRaises(HTTPError) as error: post('/api/console/end-round')
                self.assertEqual(error.exception.code, 403)
                self.assertTrue(feed.round_timer.snapshot()['running'])
                result = post('/api/console/end-round', True)
                self.assertEqual(result['round_timer_version'], 3)
                self.assertEqual(result['round_timer']['elapsed_ms'], 45000)
                self.assertFalse(result['round_timer']['running'])
                clock.advance(80); result = post('/api/reset-position')
                self.assertEqual(result['round_timer']['elapsed_ms'], 45000)
                self.assertEqual(result['position']['source'], 'VIEW_RESET')
                serial_factory.assert_not_called()
                self.assertEqual(feed.tx, 0)
        finally:
            server.shutdown(); server.server_close(); thread.join(2)

if __name__ == '__main__': unittest.main()
