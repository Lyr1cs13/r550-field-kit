"""New local theme routes must not widen file access or bypass command sessions."""
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
import urllib.request
import urllib.error
from http.server import ThreadingHTTPServer
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from server import handler


class WebAssetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.web=ThreadingHTTPServer(('127.0.0.1',0),handler(SimpleNamespace(token='test-only')))
        cls.worker=threading.Thread(target=cls.web.serve_forever,daemon=True);cls.worker.start()
        cls.url='http://127.0.0.1:'+str(cls.web.server_port)

    @classmethod
    def tearDownClass(cls):
        cls.web.shutdown();cls.web.server_close();cls.worker.join()

    def test_theme_assets_are_served_with_correct_types(self):
        for path,kind in (('/ui.js','text/javascript'),('/vendor/tabler.min.css','text/css'),('/vendor/icons.svg','image/svg+xml')):
            with urllib.request.urlopen(self.url+path) as response:
                self.assertIn(kind,response.headers['Content-Type'])
                self.assertGreater(len(response.read()),100)

    def test_static_route_cannot_read_arbitrary_workspace_files(self):
        for path in ('/server.py','/vendor/../server.py','/vendor/SOURCE.json'):
            with self.assertRaises(urllib.error.HTTPError) as error:urllib.request.urlopen(self.url+path)
            self.assertEqual(error.exception.code,404)

    def test_new_theme_does_not_bypass_robot_command_token(self):
        request=urllib.request.Request(self.url+'/api/command',data=b'{"action":"start"}',
                                       headers={'Content-Type':'application/json'},method='POST')
        with self.assertRaises(urllib.error.HTTPError) as error:urllib.request.urlopen(request)
        self.assertEqual(error.exception.code,403)
