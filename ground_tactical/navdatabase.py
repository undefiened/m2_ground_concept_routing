from bluesky import core, stack
from bluesky.navdatabase import Navdatabase
import bluesky as bs
import numpy as np

def init_plugin():
    # Addtional initilisation code
    # Configuration parameters
    e = GroundNavdatabase()
    config = {
        # The name of your plugin
        'plugin_name': 'GROUNDNAVDATABASE',
        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type': 'sim',
    }

    return config


class GroundNavdatabase(core.Entity):
    def __init__(self):
        super(GroundNavdatabase, self).__init__()

    @stack.command
    def defwpt2(self, name: 'txt', lat: float, lon: float, wptype: 'txt'):
        if name.upper() in bs.navdb.wpid:
            i = bs.navdb.wpid.index(name.upper())

            bs.navdb.wplat[i] = lat
            bs.navdb.wplon[i] = lon
            bs.scr.addnavwpt(name.upper(), lat, lon)
        else:
            bs.navdb.wpid.append(name.upper())
            bs.navdb.wplat = np.append(bs.navdb.wplat, lat)
            bs.navdb.wplon = np.append(bs.navdb.wplon, lon)

            if wptype == None:
                bs.navdb.wptype.append("")
            else:
                bs.navdb.wptype.append(wptype)

            bs.navdb.wpelev.append(0.0)  # elevation [m]
            bs.navdb.wpvar.append(0.0)  # magn variation [deg]
            bs.navdb.wpfreq.append(0.0)  # frequency [kHz/MHz]
            bs.navdb.wpdesc.append("Custom waypoint")  # description

            # Update screen info
            bs.scr.addnavwpt(name.upper(), lat, lon)

        return True, name.upper() + " added to navdb."