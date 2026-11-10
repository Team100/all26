import unittest
from app.analysis.fake_analysis import FakeColorAnalysis, FakeMonoAnalysis
from app.camera.fake_camera import FakeCamera
from app.dashboard.fake_display import FakeDisplay
from app.interpreter.dual_interpreter import DualInterpreter
from app.network.fake_network import FakeNetwork
from app.util.timestamps import Timestamps


class DualInterpreterTest(unittest.TestCase):

    def test_basic(self) -> None:
        camera = FakeCamera("images/green_blob.jpg")
        display1 = FakeDisplay()
        display2 = FakeDisplay()
        network = FakeNetwork()
        timestamps = Timestamps(network)
        analyzer_mono = FakeMonoAnalysis()
        analyzer_color = FakeColorAnalysis()
        interp = DualInterpreter(
            camera,
            display1,
            display2,
            network,
            timestamps,
            analyzer_mono,
            analyzer_color,
        )
        request = camera.capture_request()
        interp.interpret(request)
        self.assertEqual(1, display1.frame_count)
        self.assertEqual(1, analyzer_mono.count)
        self.assertEqual((600, 800), analyzer_mono.size)
        self.assertEqual(1, analyzer_color.count)
        self.assertEqual((600, 800, 3), analyzer_color.size)
