#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
ZetCode PyQt4 tutorial 

In this example, we create a simple
window in PyQt4.
"""

import sys
from PyQt4 import QtGui


def main():
    
    app = QtGui.QApplication(sys.argv)

    w = QtGui.QWidget()
    w.resize(450, 350)
    w.move(400, 300)
    w.setWindowTitle('Crear ventana PyQt')
    w.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
