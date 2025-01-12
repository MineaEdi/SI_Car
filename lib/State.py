from _thread import allocate_lock

class State:
    def __init__(self) -> None:
        self.lock = allocate_lock()

        self.end = False
        self.start = False
        self.speed = 0
        self.current_speed = 0
        self.current_segment = 0
        self.distance = 0.0
        self.manual = False
        self.left = 0
        self.right = 0

    def Start(self):
        with self.lock:
            self.start = True
            self.manual = False

    def Stop(self):
        with self.lock:
            self.start = False
            self.speed = 0
            self.manual = True

    def End(self):
        self.end = True

    def IsMoving(self):
        with self.lock:
            return self.start
    
    def IsManual(self):
        with self.lock:
            return self.manual
        
    def SetManual(self, value: bool):
        with self.lock:
            self.manual = value
        
    def SetSpeed(self, speed):
        self.manual = False
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
    
    def SetLeft(self, left_value):
        with self.lock:
            if left_value >= 0:
                self.left = left_value

    def GetLeft(self):
        with self.lock:
            return self.left
    
    def SetRight(self, right_value):
        with self.lock:
            if right_value >= 0:
                self.right = right_value

    def GetRight(self):
        with self.lock:
            return self.right

