from lib.State import State
from machine import Pin, ADC
from time import sleep_ms, ticks_us, ticks_diff

class ADCwithPullUp(ADC):
    def __init__(self, gpio, adc_vref=3.3):
        self.gpio = gpio
        self.adc_vref=adc_vref
        adc_pin = Pin(gpio, mode=Pin.IN, pull=Pin.PULL_UP)
        super().__init__(adc_pin)
        adc_pin = Pin(gpio, mode=Pin.IN, pull=Pin.PULL_UP)
        
    def sample(self):
        self.adc_value = self.read_u16()
        # Convert the ADC value to voltage
        self.voltage = (self.adc_value / 65535) * self.adc_vref
        #print("ADC Value:", self.adc_value, "Voltage:", self.voltage)
        return self.voltage


class MotorControl:
    # Circumferinta = 18.85 cm
    # Numar pasi intr-o rotatie = 4096
    # 1 pas = 0.0046 cm
    wave = [
        [1, 0, 0, 1],
        [1, 0, 0, 0],
        [1, 1, 0, 0],
        [0, 1, 0, 0],
        [0, 1, 1, 0],
        [0, 0, 1, 0],
        [0, 0, 1, 1],
        [0, 0, 0, 1]
    ]
    wave_len = len(wave)

    leftWheelPins = [9, 8, 7, 6]
    rightWheelPins = [18, 19, 20, 21]
    adcPins = [27, 26]
    irEmitorPin = 2

    threshold = 2

    def __init__(self, machine_state : State):
        self.machine_state = machine_state
        self.map = []

        self.adcs = list(map(ADCwithPullUp, self.adcPins))
        self.LED_ir = Pin(self.irEmitorPin, mode=Pin.OUT)

        self.pinsDr = [Pin(pin, Pin.OUT) for pin in self.rightWheelPins]
        self.pinsSt = [Pin(pin, Pin.OUT) for pin in self.leftWheelPins]

        self.speed = 0 #  cm / ss
    
    def SetMotorStep(self, pins, step):
        for p, s in zip(pins, step):
            p.value(s)

    def SetThreshold(self):
        pass

    def SetMap(self, map):
        self.map = map
    
    def calculateSpeed(self, new_speed):
        self.speed = self.speed + (new_speed - self.speed) / 100
    
    def calculateDelay(self, volts, maxim):
        s = self.speed * 10 * volts / maxim
        return 0 if volts < 0.2 else s # volts < 0.20

    def Run(self):
        try:
            i1 = 0
            i2 = 0
            last_time_m1 = ticks_us()
            last_time_m2 = ticks_us()
            
            while not self.machine_state.end:
                isOn = self.machine_state.IsMoving()
                isManual = self.machine_state.IsManual()
                if not isOn:
                    sleep_ms(200)
                    self.speed = 0
                    self.machine_state.SetCurrentSpeed(0)                    
                else:
                    self.calculateSpeed(self.machine_state.GetSpeed())
                    #print(self.speed)
                    
                    current_time = ticks_us()

                    self.LED_ir.on()
                    sleep_ms(1)
                    A = [ x.sample() for x in self.adcs ] 

                    self.LED_ir.off()
                    sleep_ms(1)
                    B = [ x.sample() for x in self.adcs ]

                    C = [ 1 if (b - a) > self.threshold else 0 for a, b in zip(A, B) ]
                    D = [ int( (b - a) * 1000 ) for a, b in zip(A, B) ]
                    #print(D)
                    #print(C)
                    
                    if not isManual:
                        left = self.calculateDelay(A[0], B[0])
                        right = self.calculateDelay(A[1], B[1])
                        speed = (left + right) / 20
                        #print(speed)
                        self.machine_state.SetCurrentSpeed(speed)
                    else:
                        # left, right must be between 0 - 1000
                        # 0 - stop, 1000 - max speed
                        left = self.machine_state.GetLeft() * 10
                        right = self.machine_state.GetRight() * 10
                        #print(left)
                        #print(right)

                    # Left motor stepping
                    if left != 0:
                        step_time_motor1 = 1000000 / left  # us / step 
                        if ticks_diff(current_time, last_time_m1) >= step_time_motor1 + 1000: # 2 ms from adcs
                            self.SetMotorStep(self.pinsDr, self.wave[i1])
                            i1 = (i1 + 1) % self.wave_len
                            last_time_m1 = current_time
                    
                    # Right motor stepping
                    if right != 0:
                        step_time_motor2 = 1000000 / right
                        if ticks_diff(current_time, last_time_m2) >= step_time_motor2 + 1000:  # 2 ms from adcs
                            self.SetMotorStep(self.pinsSt, self.wave[i2])
                            i2 = (i2 + 1) % self.wave_len
                            last_time_m2 = current_time

        except Exception as e: 
            print("Exceptie:", e)
            
        finally:
            for pin in self.pinsSt:
                pin.value(0)
            for pin in self.pinsDr:
                pin.value(0)

