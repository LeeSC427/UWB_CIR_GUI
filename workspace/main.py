#!/usr/bin/env_python

from run import RUN_TAG
from PyQt5 import QtWidgets, QtCore
import sys

if __name__ == "__main__":

    app = QtWidgets.QApplication(sys.argv)
    tags = RUN_TAG()

    tags.tag_1.r_0.setup_pozyx()

    tags.tag_1.r_0.pozyx_loop()

    tags.gui.show()
    sys.exit(app.exec_())
