from lib.State import State
from lib.Communication import Communication
from lib.MotorControl import MotorControl
from _thread import start_new_thread
from time import sleep

sleep(3)

state = State()
mt = MotorControl(state)
com = Communication(state)

def Core0():
    mt.Run()

def Core1():
    com.Run()

try:
    start_new_thread(Core1, () )
    Core0()
    
except KeyboardInterrupt as e:
    state.End()
    print("Ended by keyboard")
except Exception as e:
    state.End()
    sleep(1)
    print(f"Error: {e}")
finally:
    sleep(1)
    print("End of program")
