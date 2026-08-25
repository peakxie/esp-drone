import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

from config import URI

cflib.crtp.init_drivers()
with SyncCrazyflie(URI) as scf:
    lg = LogConfig(name="ResetReason", period_in_ms=100)
    lg.add_variable("sys.resetReason", "uint8_t")
    with SyncLogger(scf, lg) as logger:
        for entry in logger:
            print("resetReason =", entry[1]["sys.resetReason"])
            break
