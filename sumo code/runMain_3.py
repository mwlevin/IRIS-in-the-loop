# Initialization Stuff
from __future__ import absolute_import
from __future__ import print_function
import os
import sys
import optparse
import traci
import xml.etree.ElementTree as ET
import numpy as np
import pandas as pd
from sim import *


# ################################################## give me an alarm when the code finish running
# from IPython.display import Audio, display
# #################################################

test = Simulation("test_network/site_trial.net.xml", "test_network/HD_fullHV.rou.xml", "test_network/detectors.txt", "output_test.txt")
test.run()
