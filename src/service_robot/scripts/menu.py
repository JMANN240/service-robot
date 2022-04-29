#/usr/bin/env python3

import sys

from service_robot.menu import MenuPlugin
from rqt_gui.main import Main

plugin = "menu"
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
