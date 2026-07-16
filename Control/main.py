from mavlink_io import ManualControl
from function import initialize
from controlfunction import run_control_loop


if __name__ == "__main__":
    initialize()
    run_control_loop()
    ManualControl(0, 0, 300, 0)
