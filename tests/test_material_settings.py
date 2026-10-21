"""Exercise distance persistence and real HTTP ownership without physical hardware."""
import json
import urllib.error
import urllib.request
from types import SimpleNamespace
from unittest.mock import patch

from host.webapp import Console
from tests import test_vision_tuning_http as fixture
import unittest


class MaterialSettingsTests(unittest.TestCase):
    setUp = fixture.TuningHttpTests.setUp
    tearDown = fixture.TuningHttpTests.tearDown

    def call(self, config=None, token='writer'):
        """Use production GET/POST routes with the fixture's explicit writer lease."""
        request = urllib.request.Request(
            'http://127.0.0.1:{}/api/material-flow/config'.format(self.server.server_port),
            data=None if config is None else json.dumps({'config':config}).encode())
        request.add_header('X-Session', token)
        try:
            with urllib.request.urlopen(request, timeout=3) as response:
                return response.status, json.load(response)
        except urllib.error.HTTPError as error:
            return error.code, json.load(error)

    def test_persistence_validation_and_no_movement(self):
        self.assertEqual(self.call()[1]['config'], dict(point2_steps=2150, point3_steps=4230))
        config = dict(point2_steps=1920, point3_steps=4070)
        self.assertEqual(self.call(config)[0], 200)
        path = self.console.material_settings.path
        before = path.read_bytes()
        restarted = Console(navigation_path=self.console.mission.path)
        self.assertEqual(restarted.material_settings.snapshot(), config)
        self.assertEqual(restarted.material_settings.offsets(), {1:0, 2:1920, 3:4070})
        for invalid in ({}, dict(config, point2_steps=True), dict(config, point2_steps=0),
                        dict(config, point3_steps=100001), dict(config, point2_steps=1.5),
                        dict(config, point3_steps=1920), dict(config, point3_steps=1000),
                        dict(config, extra=1)):
            with self.subTest(invalid=invalid):
                self.assertEqual(self.call(invalid)[0], 400)
                self.assertEqual(path.read_bytes(), before)
                self.assertEqual(self.call()[1]['config'], config)
        self.assertIsNone(self.console.connection)

    def test_readonly_and_busy_cannot_change_distances(self):
        original = self.call()[1]['config']
        config = dict(point2_steps=2200, point3_steps=4400)
        self.assertEqual(self.call(config, token='observer')[0], 409)
        for name in ('task', 'competition', 'pick_place', 'navigator', 'camera_alignment'):
            with self.subTest(owner=name), patch.object(self.console, name, SimpleNamespace(active=True)):
                self.assertEqual(self.call(config)[0], 400)
        self.assertEqual(self.call()[1]['config'], original)
        self.assertFalse(self.console.material_settings.path.exists())
