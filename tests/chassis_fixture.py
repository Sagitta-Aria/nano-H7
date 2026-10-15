"""Expected wheel directions shared by firmware and browser contract tests.

This fixture contains no emulator imports so Nano host checks need no ARM tools.
"""
MATRIX = {"FWD": (1, 1, 1, 1), "BACK": (-1, -1, -1, -1),
          "LEFT": (1, -1, -1, 1), "RIGHT": (-1, 1, 1, -1),
          "CCW": (-1, 1, -1, 1), "CW": (1, -1, 1, -1)}
