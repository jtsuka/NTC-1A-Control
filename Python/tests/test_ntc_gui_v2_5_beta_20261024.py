"""Offline regression for v2.5 BETA GUI, without pygame or serial hardware.

Run:
  python3 -m unittest discover -s Python/tests -p 'test_ntc_gui_v2_5_beta_20261024.py' -v
"""
import ast
import pathlib
import types
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]
GUI_BETA = ROOT / "Python/ntc_pygame_ui_v2_5_beta.py"
GUI_ORIGINAL = ROOT / "Python/ntc_pygame_ui_v2_4.py"


class FakeSurface:
    def __init__(self, width, height):
        self.width, self.height = width, height

    def get_width(self):
        return self.width

    def get_height(self):
        return self.height


class FakeFont:
    def __init__(self, size):
        self.size = size

    def render(self, text, antialias, color):
        return FakeSurface(max(1, len(text) * self.size // 2), self.size)


def fitted_renderer():
    """Compile only the new helper AST: never execute GUI module top-level."""
    source = GUI_BETA.read_text(encoding="utf-8")
    tree = ast.parse(source)
    funcs = [node for node in tree.body
             if isinstance(node, ast.FunctionDef)
             and node.name == "render_fitted_actual_tension"]
    if len(funcs) != 1:
        raise AssertionError("Expected one fitted-render helper")
    code = compile(ast.Module(body=funcs, type_ignores=[]), str(GUI_BETA), "exec")
    pygame = types.SimpleNamespace(
        font=types.SimpleNamespace(Font=lambda name, size: FakeFont(size)),
        transform=types.SimpleNamespace(
            smoothscale=lambda surface, size: FakeSurface(*size)
        ),
    )
    ns = {"font": FakeFont(50), "FONT_SIZE": 50, "pygame": pygame}
    exec(code, ns)
    return ns["render_fitted_actual_tension"]


class BetaGuiTests(unittest.TestCase):
    def test_beta_import_only(self):
        beta = GUI_BETA.read_text(encoding="utf-8")
        old = GUI_ORIGINAL.read_text(encoding="utf-8")
        self.assertIn("from NTC_1A_serial_comm_v1_3_5_beta import (", beta)
        self.assertNotIn("from NTC_1A_serial_comm_v1_3_4 import (", beta)
        self.assertIn("from NTC_1A_serial_comm_v1_3_4 import (", old)

    def test_four_digit_tension_fits_box(self):
        render = fitted_renderer()
        for value in (0, 9, 99, 100, 999, 1000, 1023):
            with self.subTest(value=value):
                surface = render(str(value), types.SimpleNamespace(width=80), (255, 255, 255))
                self.assertLessEqual(surface.get_width(), 70)
                self.assertGreater(surface.get_width(), 0)

    def test_extremely_narrow_box_uses_scaling(self):
        render = fitted_renderer()
        surface = render("1023", types.SimpleNamespace(width=7), (255, 255, 255))
        self.assertLessEqual(surface.get_width(), 1)

    def test_monitor_only_changes(self):
        beta = GUI_BETA.read_text(encoding="utf-8")
        self.assertIn("if key == 'ACTUAL_TENSION':", beta)
        self.assertIn("render_fitted_actual_tension(fields[key], rect, value_color)", beta)
        self.assertIn("else:\n                val_txt = font.render(fields[key], True, value_color)", beta)
        self.assertIn("RIGHT_MONITOR_VALUE_X_SHIFT = 0.04", beta)
        self.assertIn("fields['ACTUAL_TENSION'] = '---' if telemetry['actual_tension'] is None else str(telemetry['actual_tension'])", beta)


if __name__ == "__main__":
    unittest.main()
