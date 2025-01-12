from machine import UART, Pin
from time import sleep_ms
from lib.State import State
from re import search
from rp2 import PIO, StateMachine, asm_pio
from lib.Message import Message

class Communication:
    manager_id = 0
    def __init__(self, machine_state : State) -> None:
        self.id=1
        self.messageId=0
        self.machine_state = machine_state
        self.bluetooth = BluetoothCom(0, 9600, timeout=100)
        self.infrared = IrCom()

    def processIR(self, msg):
        if msg.srcType == 'S':
            print("Mesaj primit de la Semafor")
            
        elif msg.srcType == 'R':
            print("Mesaj primit de la o Masina")

    def processSVR(self, msg):
        # print("procesSVR")
        # print(msg.bytes())
        # print(msg.data)
        # print(f"{msg.data[0]}{msg.data[2:]}")
        # print(f"{msg.srcType}")
        if msg.srcType == ord('M'):
            # print("Mesaj primit de la Server")
            if msg.data == b'1':
                # print("Start")
                self.machine_state.Start()
            elif msg.data[0] == ord('2'):
                viteza = int(msg.data[2:].decode('utf-8'))
                self.machine_state.SetSpeed(viteza)
            elif msg.data == b'0':
                # print("Stop")
                self.bluetooth.start = False
                self.machine_state.Stop()
            elif msg.data == b'-1':
                # print("End")
                self.machine_state.End()
            elif msg.data[0] == ord('3'):
                self.machine_state.SetManual(True)
                print(self.machine_state.IsManual())
                end_position = msg.data.decode('utf-8').find('.')
                left_value = int(msg.data[2:end_position].decode('utf-8'))
                self.machine_state.SetLeft(left_value)
                right_value = int(msg.data[end_position + 1:].decode('utf-8'))
                self.machine_state.SetRight(right_value)
                # print(left_value)
                # print(right_value)
                # print("aici")
                # print(msg.data)
                

    def ProcessMessages(self):
        message = self.bluetooth.receive()
        if message is not None:
                # print(f"Mesaj bluetooth: {message.chars()}")
                self.processSVR(message)    
        
        message = self.infrared.receive()
        if message is not None:
                # print(f"Mesaj infrarosu: {message.chars()}")
                self.processIR(message)    

    def SendInfoToManager(self):
    #   (sourceType, sourceId, destination, messageId, data, crc8):
        msg = Message(ord('R'), self.id, self.manager_id, self.messageId, f"Speed: {self.machine_state.GetCurrentSpeed()//1}")
        print(msg.chars())
        self.bluetooth.send(msg)

    def Run(self):
        while not self.machine_state.end:
            self.ProcessMessages()
            if self.bluetooth.start == True:
                self.SendInfoToManager()
            sleep_ms(1000)


class BluetoothCom(UART):
    start = False
    '''
    Citirea se face pana la caracterul '\r\n'
    Se calculeaza crc-ul mesajului primit
    Daca e corect se returneaza mesajul sau -1 daca este incorect
    '''
    def receive(self):
        msg = None
        if self.any():
            data = self.readline()
            # print(data)
            try:
                msg = Message.create_message(data)
                self.start = True
            except ValueError as e:
                print(f"Eroare bluetooth: {e}")
        return msg
    
    def send(self, msg: Message):
        data_to_send = msg.bytes() + bytearray(b'\r\n')
        #print(data_to_send)
        self.write(data_to_send)


class IrCom(UART):
    def __init__(self, uart_id=1, baud=1200*3, sm_id=0, pout=3, freq_Hz=38_000, extra_off_duty=15, extra_on_duty=0): # up to 3*1200 baud seems to work (because of the Ir receiver constraints)
        #define inline the 
        @asm_pio(sideset_init=(PIO.OUT_LOW)) #, set_init=(PIO.OUT_LOW), out_init=(PIO.OUT_LOW), in_shiftdir=PIO.SHIFT_LEFT, out_shiftdir=PIO.SHIFT_RIGHT)
        def pio_prog():
            label("start") # labels don't consume SM instruction space
            jmp(pin, "start").side(0) [extra_off_duty] # when no uart tx line is in idle state, do not modulate Ir pulses
            jmp("start").side(1) [extra_on_duty] # when no uart tx line is not idle, this line produces an Ir pulse
            
        super().__init__(uart_id)
        uart_config_str = str(self) # get the uart config to extract GPIO numbers
        self._pin = Pin(int(search(' tx=(\\d+), ', uart_config_str).group(1)), Pin.IN) # use regex to auto obtain the assigned tx pin based on selected uart instance
        self._prx = Pin(int(search(' rx=(\\d+), ', uart_config_str).group(1)), Pin.IN) # could be used to check for darkness...
        self._pout = Pin(pout)
        # extra_off_duty is used to consume less power by keeping the Ir LED off for a longer fraction of the modulation period
        self._sm = StateMachine(sm_id, pio_prog, freq=(extra_off_duty + extra_on_duty + 2) * freq_Hz, sideset_base=self._pout, jmp_pin=self._pin) # , set_base=self._pout, out_base=self._pout, in_base=self._pin)
        self._sm.active(1) # use 1 to activate, 0 to stop
        self.init(baud) # reinit uart since it rx pin was overiden by PIO...

    def receive(self):
        msg = None
        if self.any():
            data = self.readline()
            # print(data)
            try:
                msg = Message.create_message(data)
                self.start = True
            except ValueError as e:
                print(f"Eroare infrarosu: {e}")
        return msg
    
    def send(self, msg: Message):
        data_to_send = msg.bytes() + bytearray(b'\r\n')
        self.write(data_to_send)

