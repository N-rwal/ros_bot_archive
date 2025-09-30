import struct
import paho.mqtt.client as mqtt
import time
import sys
import tty
import termios
import serial
import json

def on_connect(client, userdata, flags, rc):
    print("Connected to broker with result code", rc)
    client1.subscribe("frigate/events")

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload)
        print("\nEvent Received:")
        print(json.dumps(payload, indent=2))

        cmd|=0x09
        ser.write(bytes([cmd]))     #send cmd
        cmd = 0x00                  #clear cmd

    except Exception as e:
        print(f"Error decoding message: {e}")

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  #USB connection

client1 = mqtt.Client("client1")    #Frigate service
client1.connect("192.168.100.5", 1883, 60)
client1.on_connect = on_connect
client1.on_message = on_message

client2 = mqtt.Client("client2")    #Robot commands
client2.connect("192.168.123.161", 1883, 60)

client2.publish("controller/action", "walk")

move_x = 0.0
move_y = 0.0
rotation = 0.0
cmd = 0b00000000

client1.loop_start()
client2.loop_start()

try:
    while True:
        move_x = 0.0
        move_y = 0.0
        rotation = 0.0
        key = get_key()

        if key == 'q':
            move_x = -0.5
        elif key == 'd':
            move_x= 0.5
        elif key == 'z':
            move_y = 0.3
        elif key == 's':
            move_y = -0.3
        elif key == 'a':
            rotation = -0.25
        elif key == 'e':
            rotation = 0.25
        elif key == 't':
            print("\nEnter numeric command:")
            cmd_str = input()
            try:
                cmd = int(cmd_str)          #user input
                ser.write(bytes([cmd]))     #send cmd
                cmd = 0x00                  #clear cmd
            except ValueError:
                print("Invalid number.")
        elif key == '\x03':  #Ctrl+C
            break
        
        payload = struct.pack('<ffff', move_x, rotation, 0.0, move_y) #0.0 not used, it's the pitch
        client2.publish("controller/stick", payload)
        time.sleep(0.05)
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)
        ser.close()
        client1.loop_stop()
        client2.loop_stop()
        client1.disconnect()
        client2.disconnect()
"""
cmd = 0b00000011
ser.write(bytes([cmd]))
"""