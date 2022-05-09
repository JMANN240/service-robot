#/usr/bin/env python3

import sys

from service_robot.order import OrderPlugin
from rqt_gui.main import Main

plugin = "order"
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
