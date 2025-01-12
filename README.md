# Mașinuță șmecheră

## Universitatea Tehnică „Gheorghe Asachi” Iași
## Facultatea de Automatică și Calculatoare Iași
## Specializarea: Calculatoare și Tehnologia Informației
## Disciplina: Sisteme cu microprocesoare

**Autori:** 
- **Gălățanu Marco Ionuț 1306B**: asamblarea circuitului, desen tehnic, detecția liniei, comunicare bluetooth
- **Minea Eduard Constantin 1306B**: markdown, comunicare infraroșu
- **Petrișor Rareș Gabriel 1306B**: arhitectură, integrare module comunicare, realizare PID
- **Zaharia Teodor Ștefan 1306B**: CRC8, markdown, control steppere

**Data:** 19.06.2024

## Cuprins
1. [Descrierea proiectului](#descrierea-proiectului)
    1. [Componente Hardware](#componente-hardware)
    2. [Funcționalități Software](#funcționalități-software)
    3. [Configurare pini GPIO](#configurare-pini-gpio)
2. [Arhitectura sistemului](#arhitectura-sistemului)
3. [Configurare proiect](#configurare-proiect)
4. [Anexe](#anexe)
    1. [Părți semnificative ale codului sursă](#părți-semnificative-ale-codului-sursă)
    2. [Ilustrarea sistemului implementat](#ilustrarea-sistemului-implementat)
5. [Concluzie](#concluzie)
6. [Referințe](#referințe)

## Descrierea proiectului:
Acest proiect prezintă o mașină inteligentă, având ca și componentă centrală un microcontroller Raspberry Pi Pico. Sistemul utilizează mai multe componente hardware, printre care: 2 motoare stepper unipolare, fiecare cu câte un driver (tranzistoare ULN 2003A) specific pentru controlul mișcărilor, un modul Bluetooth pentru a comunica cu serverul, un ansamblu format din 2 receptoare infraroșu si un emițător infraroșu cu rolul de a menține traseul, un alt ansamblu infraroșu format din emițător și receptor TSOP45, folosit pentru comunicarea cu semaforul. Prin intermediul unui cod microPython, rulat din utilitarul Thonny/VSCode (extensia micropico), mașina este pusă în mișcare de către comenzile server-ului. 

### Componente Hardware:
- **Raspberry Pi Pico cu microprocesor RP 2040:** Microcontrolerul principal care rulează codul și gestionează funcționarea mașinii
- **PCB 50x70, 10x60:** Suport pentru conectarea tuturor componentelor
- **2xMotor stepper unipolar:** Asigură mobilitatea mașinii
- **2xDriver motor stepper ULN2003A:** Circuit integrat pentru controlul precis al mișcărilor
- **2xReceptor infraroșu + LED infraroșu:** Detecție linie
- **Modulul Bluetooth HC-05:** Permite comunicarea wireless cu serverul pentru a primi comenzi de start, stop și end
- **Receptor infraroșu TSOP45 + LED infraroșu:** Pentru comunicarea cu semaforul
- **Cablu de alimentare și fire de conexiune:** Necesare pentru a conecta toate componentele și a asigura alimentarea necesară
- **2xRezistență de 220 Ω:** Asigură limitarea curentului care trece prin LED-uri și le protejează împotriva supraîncălzirii și a deteriorării.
- **Baterie externă:** Alimentează întreg sistemul

### Funcționalități Software:
- **Configurare pini GPIO:** Detaliat mai jos
- **Control motoare:** Metode pentru setarea stării motoarelor
- **Detecție linie:** Receptorii captează semnalul trimis de LED-ul dintre ei și activează stepper-ele
- **Comunicare Bluetooth cu server-ul:** Configurat pentru a trimite mesaje(distanța parcursă) către server și a primi comenzi (start, stop, end)
- **Comunicare infraroșu cu semaforul:** Citim starea semaforului
- **microPython:** Limbajul de programare folosit
- **Thonny/VSCode:** Aplicația folosită pentru încărcarea codului pe Raspberry Pi Pico

### Configurare pini GPIO:
![GPIO](<poze/configurare-pini-GPIO.jpg>)
- **Control motoare:**
  - VCC: PIN40
  - **Motor stânga:**
    - PIN4_STEPPER_STÂNGA: GPIO6
    - PIN3_STEPPER_STÂNGA: GPIO7
    - PIN2_STEPPER_STÂNGA: GPIO8
    - PIN1_STEPPER_STÂNGA: GPIO9
    - GND_ULN_STÂNGA: PIN13(GND)
  - **Motor dreapta:**
    - PIN4_STEPPER_DREAPTA: GPIO21
    - PIN3_STEPPER_DREAPTA: GPIO20
    - PIN2_STEPPER_DREAPTA: GPIO19
    - PIN1_STEPPER_DREAPTA: GPIO18
    - GND_ULN_STÂNGA: PIN23(GND)

- **Detecție linie:**
  - **Receptor stânga:**
    - ANOD: GPIO27 (ADC1)
    - CATOD: PIN33(AGND)
  - **Receptor dreapta:**
    - ANOD: GPIO26 (ADC0)
    - CATOD: PIN33(AGND)
  - **LED:**
    - ANOD: rezistență(220 Ω), după care GPIO2
    - CATOD: PIN3(GND)

- **Comunicare Bluetooth HC-05:**
  - GND_BLUETOOTH: PIN38(GND)
  - VCC: PIN40
  - RX_BLUETOOTH: GPIO1
  - TX_BLUETOOTH: GPIO0

- **Comunicare infraroșu:**
  - **Receptor:**
    - ANOD: GPIO5
    - CATOD: PIN3(GND) 
  - **LED:**
    - ANOD: rezistență(220 Ω), după care GPIO3
    - CATOD: PIN3(GND)

## Arhitectura sistemului
- **main.py**
Punctul de pornire al programului.
- **manager.py**
    - **Clasa Manager**: Centralizează controlul mașinii, incluzând comunicarea și controlul motoarelor.
- **lib/**
    - **Communication.py** 
        - **Clasa Communication**: Gestionează comunicațiile prin Bluetooth și infraroșu.
        - **Clasa BluetoothCom**: Extinde UART pentru a gestiona comunicația prin Bluetooth.
        - **Clasa IrCom**: Extinde UART pentru a gestiona comunicația prin infraroșu.
    - **Message.py**
        - **Clasa Message**: Gestionează crearea și validarea mesajelor.
    - **MotorControl.py**
        - **Clasa ADCwithPullUp**: Extinde clasa ADC pentru a adăuga funcționalitate de pull-up.
        - **Clasa MotorControl**: Controlează mișcarea roților și monitorizează senzori.
    - **State.py**
        - **Clasa State**: Gestionează starea mașinii și viteza curentă.


## Configurare proiect
În funcție de utilitarul dorit urmărim unul din cele 2 tutoriale:
- VSCode: https://randomnerdtutorials.com/raspberry-pi-pico-vs-code-micropython/#flash-micropython
- Thonny: https://projects.raspberrypi.org/en/projects/getting-started-with-the-pico/

## Anexe
### Părți semnificative ale codului sursă:
1. **Clasa pentru comunicarea prin bluetooth cu server-ul:**
    ```python
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
    ```

2. **Clasa pentru comunicarea prin infraroșu cu semaforul:**
    ```python
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
    ```

3. **Clasa ce se ocupă de crearea și validarea mesajelor, folosind CRC8 Dallas/Maxim:**
    ```python
    class Message:
    crc_table = [
                0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 
                0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41, 
                0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 
                0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 
                0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 
                0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 
                0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 
                0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 
                0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 
                0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 
                0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 
                0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 
                0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 
                0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 
                0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 
                0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 
                0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 
                0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 
                0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 
                0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 
                0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 
                0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 
                0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 
                0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 
                0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 
                0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 
                0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 
                0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 
                0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 
                0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 
                0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 
                0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
                ]
    
    @staticmethod
    def CRC8(msg: bytearray):
        crc = 0x00
        for byte in msg:
            crc = Message.crc_table[crc ^ byte]
        return crc
    
    def __init__(self, sourceType, sourceId=0, destination=0, messageId=0, data=""):
        if not (0 <= sourceId <= 255):
            raise ValueError("sourceId must be between 0 and 255")
        if not (0 <= destination <= 255):
            raise ValueError("destination must be between 0 and 255")
        if not (0 <= messageId <= 255):
            raise ValueError("messageId must be between 0 and 255")
        #print(f"MSG type: {sourceType}: {[ord('M'), ord('R'), ord('S')]}")
        if isinstance(sourceType, str):
            sourceType = ord(sourceType)
        if sourceType not in [ord('M'), ord('R'), ord('S')]:
            #print(ord(sourceType))
            raise ValueError("sourceType must be 'M', 'S', or 'R'")
        
        self.sync = "UUU3"
        self.srcId = sourceId
        self.srcType = sourceType
        self.dest = destination
        self.msgId = messageId
        self.data = data
        self.crc8 = 0
        self.len = len(self.data)

    @staticmethod
    def create_message(data):
        # print(data[-1])
        # print(Message.CRC8(data[4:-1]))
        if data[0:4] != b'UUU3':
            raise ValueError("Syncronization not made")
        
        if Message.CRC8(data[4:]) != 0x00:
            raise ValueError("Message trasmited incorrectly")
        
        length = int(data[4])
        srcType = data[6]
        # print(srcType)
        srcId = int(data[7])
        dest = int(data[8])
        msgId = int(data[9])
        msg_data = data[10:10 + length] if length > 0 else ""
        
        return Message(sourceType=srcType, sourceId=srcId, destination=dest, messageId=msgId, data=msg_data)       
        
    def bytes(self):
        neg_len = 255 - self.len
        data_str = ''
        hex_sync = ''.join(f"{ord(c):02X}" for c in self.sync)
        if self.data is not None:
            for item in self.data:
                if isinstance(item, int):
                    data_str += f"{item:02X}"  
                elif isinstance(item, str):
                    data_str += ''.join(f"{ord(char):02X}" for char in item)

        msg = f"{self.len:02X}{neg_len:02X}{self.srcType:02X}{self.srcId:02X}{self.dest:02X}{self.msgId:02X}{data_str}"
        self.crc8 = Message.CRC8(bytearray.fromhex(msg))
        msg = f"{hex_sync}{msg}{self.crc8:02X}"
        return bytearray.fromhex(msg)
    
    def chars(self):
        neg_len = 255 - self.len
        self.bytes()
        return f"{self.sync} {self.len} {neg_len} {self.srcType} {self.srcId} {self.dest} {self.msgId} {self.data} {self.crc8}"
    ```

4. **Extinde ADC pentru a adăuga funcționalitate de pull-up:**
    ```python
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
    ```

5. **Controlează mișcarea roților și monitorizează senzori:**
    ```python
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
    
    def calculateDelay(self, volts, max):
        s = self.speed * 10 * volts / max
        return 0 if volts < 0.2 else s # volts < 0.20

    def Run(self):
        try:
            i1 = 0
            i2 = 0
            last_time_m1 = ticks_us()
            last_time_m2 = ticks_us()
            
            while not self.machine_state.end:
                isOn = self.machine_state.IsMoving()
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

                    # C = [ 1 if (b - a) > self.threshold else 0 for a, b in zip(A, B) ]
                    # D = [ int( (b - a) * 1000 ) for a, b in zip(A, B) ]
                    # print(D)
                    # print(C)

                    left = self.calculateDelay(A[0], B[0])
                    right = self.calculateDelay(A[1], B[1])
                    speed = (left + right) / 20
                    #print(speed)
                    self.machine_state.SetCurrentSpeed(speed)                
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
    ```

6. **Gestionează starea mașinii și viteza curentă:**
```python
class State:
    def __init__(self) -> None:
        self.lock = allocate_lock()

        self.end = False
        self.start = False
        self.speed = 0
        self.current_speed = 0
        self.current_segment = 0
        self.distance = 0.0

    def Start(self):
        with self.lock:
            self.start = True
            self.speed = 100

    def Stop(self):
        with self.lock:
            self.start = False
            self.speed = 0

    def End(self):
        self.end = True


    def IsMoving(self):
        with self.lock:
            return self.start
        
    def SetSpeed(self, speed):
        with self.lock:
            if speed <= 0:
                self.speed = 0
                self.start = False
            elif 0 < speed <= 100:
                self.speed = speed

    def SetCurrentSpeed(self, speed):
        with self.lock:
            self.current_speed = speed

    def GetCurrentSpeed(self):
        with self.lock:
            return self.current_speed

    def GetSpeed(self):
        with self.lock:
            return self.speed
        
    def SetCurrentSegment(self, segment):
        with self.lock:
            if segment >= 0:
                self.current_segment = segment

    def GetCurrentSegment(self):
        with self.lock:
            return self.current_segment

    def SetDistance(self, distance):
        with self.lock:
            if distance >= 0:
                self.distance = distance

    def GetDistance(self):
        with self.lock:
            return self.distance
```

7. **Simulare a server-ului pentru testare:**
```python
# Open the serial port
ser = serial.Serial('COM8', 9600, timeout=10000)

def send_message_via_bluetooth(message_bytes):
    try:
        # Send the message over the serial port
        ser.write(message_bytes)
        print(f"Sent: {message_bytes}")
        sleep(0.2)
    except Exception as e:
        print(f"Failed to send message: {e}")

def receive_message_via_bluetooth():
    while True:
        try:
            data = ser.readline()
            if data:
                print(f"Data: {data}")
                msg = Message.create_message(data[0:-2])
                if msg:
                    print(f"Received: {msg.bytes()}")
        except Exception as e:
            print(f"Failed to receive message: {e}")

if __name__ == "__main__":
    # Example message text\
    # message_text = 'UUU312243M12553HelloBoysYes18'

    start_txt = Message('M', 1, 0xFF, 3, "1").bytes()
    stop_text = Message('M', 1, 0xFF, 3, "0").bytes()
    speed_text = Message('M', 1, 0xFF, 3, "2:30").bytes()
    speed_text2 = Message('M', 1, 0xFF, 3, "2:80").bytes()
    end_text = Message('M', 1, 0xFF, 3, "-1").bytes()

    # Start the reception thread
    reception_thread = threading.Thread(target=receive_message_via_bluetooth)
    reception_thread.daemon = True  # This makes the thread exit when the main program exits

    send_message_via_bluetooth(start_txt)

    sleep(1)
    reception_thread.start()

    sleep(4)
    send_message_via_bluetooth(speed_text)

    sleep(10)
    send_message_via_bluetooth(stop_text)

    sleep(5)
    send_message_via_bluetooth(start_txt)
    sleep(1)
    send_message_via_bluetooth(speed_text2)

    sleep(10)
    send_message_via_bluetooth(stop_text)

    sleep(3)
    send_message_via_bluetooth(end_text)
```

### Ilustrarea sistemului implementat:
Imaginile prezintă configurația hardware utilizată în proiectul nostru de mașinuță șmecheră.

![difi2](<poze/DIFI2.jpg>)
![pcbs](<poze/PCBs.png>)
![IR_line_follower](<poze/IR_line_detector.png>)
![wiring](<poze/wiring.png>)
![Initial_version](<poze/Initial_version.png>)
![THE_CAR](<poze/THE_CAR.png>)

## Concluzie
Proiectul „Mașinuță șmecheră” reprezintă o implementare practică a unui sistem de control bazat pe Raspberry Pi Pico, utilizând diverse componente hardware . Acest proiect este ideal pentru înțelegerea principiilor de bază ale programării și electronicii, precum și pentru explorarea comunicării prin Bluetooth și prin infraroșu.

## Referințe
- [hardware-design-with-rp2040.pdf](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [HC-05 Datasheet.pdf](https://components101.com/sites/default/files/component_datasheet/HC-05%20Datasheet.pdf)
- [TSOP45.pdf](https://www.vishay.com/docs/82460/tsop45.pdf)
- [ULN2003A](https://pdf1.alldatasheet.com/datasheet-pdf/view/25566/STMICROELECTRONICS/ULN2003A.html)
- [VSCode tutorial](https://randomnerdtutorials.com/raspberry-pi-pico-vs-code-micropython/#flash-micropython)
- [Thonny tutorial](https://projects.raspberrypi.org/en/projects/getting-started-with-the-pico/)
