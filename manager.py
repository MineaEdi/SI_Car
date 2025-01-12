from lib.Message import Message
import serial
from time import sleep
import threading

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
    left_right_text1 = Message('M', 1, 0xFF, 3, "3:1.50").bytes()
    left_right_text2 = Message('M', 1, 0xFF, 3, "3:1.100").bytes()
    end_text = Message('M', 1, 0xFF, 3, "-1").bytes()

    # Start the reception thread
    reception_thread = threading.Thread(target=receive_message_via_bluetooth)
    reception_thread.daemon = True  # This makes the thread exit when the main program exits

    send_message_via_bluetooth(start_txt)

    sleep(1)
    reception_thread.start()

    # sleep(4)
    # send_message_via_bluetooth(speed_text)

    # sleep(10)
    # send_message_via_bluetooth(stop_text)

    # sleep(5)
    # send_message_via_bluetooth(start_txt)
    # sleep(1)
    # send_message_via_bluetooth(speed_text2)

    # sleep(10)
    # send_message_via_bluetooth(stop_text)
    
    sleep(10)
    send_message_via_bluetooth(left_right_text1)
    
    sleep(10)
    send_message_via_bluetooth(left_right_text2)
    
    sleep(10)
    send_message_via_bluetooth(stop_text)

    sleep(3)
    send_message_via_bluetooth(end_text)
