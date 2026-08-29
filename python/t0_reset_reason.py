import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

from config import URI, connect_with_timeout

cflib.crtp.init_drivers()
cf = Crazyflie()
if connect_with_timeout(cf, URI):
    try:
        lg = LogConfig(name="ResetReason", period_in_ms=100)
        lg.add_variable("sys.resetReason", "uint8_t")
        with SyncLogger(cf, lg) as logger:
            for entry in logger:
                print("resetReason =", entry[1]["sys.resetReason"])
                break
    finally:
        cf.close_link()
