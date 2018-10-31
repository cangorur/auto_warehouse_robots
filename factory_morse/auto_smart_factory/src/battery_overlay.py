from morse.core.overlay import MorseOverlay
from morse.core import status
import random

class RandomInitBatteryOverlay(MorseOverlay):

    def __init__(self, overlaid_object):
        # Call the constructor of the parent class
        MorseOverlay.__init__(self, overlaid_object)
        
        # set random initial charge
        # minimum 20% to avoid initially broken down robots
        self.overlaid_object.local_data['charge'] = random.uniform(20., 100.)
