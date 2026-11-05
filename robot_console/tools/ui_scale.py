#!/usr/bin/env python3
"""Choose a readable presentation size before starting Qt on dense displays."""
import math


def scale_for(width, height):
    available = min(width / 1600, height / 950)
    return max(1., min(2.5, math.floor(available * 4) / 4))


if __name__ == '__main__':
    from PyQt5.QtWidgets import QApplication
    app = QApplication([])
    screen = app.primaryScreen()
    area = screen.availableGeometry()
    print('%g' % scale_for(area.width(), area.height()))
