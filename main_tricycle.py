import machine
import socket
import math
import utime
import network
import time
import sys
from machine import Pin, PWM, UART
from time import sleep
from network import WLAN

x = 0
y = 0
theta = 0
alpha = 0
end_angle = 0
angle_wheel = 0
DIST_PROX_ERROR = 0.01
DIST_FINAL_ERROR = 0.001
ANGLE_PROX_ERROR = 10
ANGLE_FINAL_ERROR = 0.5
WHEEL_SPEED_NORMAL = 25000 # PWM frequency
WHEEL_SPEED_PROX = 10000
ToMetres = 0.219/960
radius = 86.5/1000
SERVO_SPEED = 1500

# SERVO CODE SETUP - CHECK CODE test_servo_lewan for help
SERVO_MOVE_TIME_WRITE = 1

def lower_byte(value):
    return int(value) % 256

def higher_byte(value):
    return int(value / 256) % 256

def word(low, high):
    return int(low) + int(high)*256

def clamp(range_min, range_max, value):
    return min(range_max, max(range_min, value))

class ServoController(object):
    def __init__(self, uart, timeout=1):
        self.uart = uart

    def _command(self, servo_id, command, *params):
        length = 3 + len(params)
        checksum = 255-((servo_id + length + command + sum(params)) % 256)
        #print(f"Sending servo control packet: [0x55, 0x55, {servo_id}, {length}, {command}, {list(params)}, {checksum}]") 
        self.uart.write(bytearray([0x55, 0x55, servo_id, length, command] + list(params) + [checksum]))

    def move(self, servo_id, position, time=0):
        position = clamp(0, 1000, position)
        time = clamp(0, 30000, time)

        self._command(servo_id, SERVO_MOVE_TIME_WRITE, lower_byte(position), higher_byte(position), lower_byte(time), higher_byte(time),)

# END SERVO SETUP

#LED to make sure the connection is complete
led = machine.Pin("LED",machine.Pin.OUT)
 
# encoder pins
encA=machine.Pin(26,machine.Pin.IN, Pin.PULL_DOWN)
encB=machine.Pin(27,machine.Pin.IN, Pin.PULL_DOWN)

#motor pins
# motor 2 PICO4DRIVE
# Red to B
# Black to A
PWMA = PWM(Pin(13)) #Forwards
PWMB = PWM(Pin(12)) #Backwards
 
PWMA.freq(1000) 
PWMB.freq(1000) 

#Create servo
ctrl = ServoController(uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1), txbuf=512))
 
#DIFFERENT WIFIs
ssid = 'Haste'
password = 'Wifipassword'

# Set the static IP address, subnet mask, gateway, and DNS server
ip_address = '192.168.8.101'
subnet_mask = '255.255.255.0'
gateway = '192.168.1.1'
dns_server = '8.8.8.8'

def connect():
    #Connect to WLAN
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    while wlan.isconnected() == False:
        print('Waiting for connection...')
        sleep(1)
    # Set the static IP address
    wlan.ifconfig((ip_address, subnet_mask, gateway, dns_server))

    ip = wlan.ifconfig()[0]
    print(f'Connected on {ip}')
    led.on()
    return ip

def open_socket(ip):
    # Open a socket
    address = (ip, 80)
    connection = socket.socket()
    connection.bind(address)
    connection.listen(1)
    return connection 
 
def webpage():
    html = """
        <!DOCTYPE html>
        <html>
        <head>
            <style>
                body {
                    text-align: center;
                }
                input[type="submit"] {
                    font-size: 24px;
                    padding: 10px 20px;
                }
                input[type="text"] {
                    font-size: 24px;
                    padding: 5px;
                    width: 100px;
                }
                label {
                    font-size: 28px;
                }
            </style>
        </head>
        <body>
            <form action="/path">
                <input type="submit" value="Path">
            </form>
            
            <form action="/xytheta">
                <label for="x">x goal:</label><br>
                <input type="text" id="x" name="x"><br>
                <label for="y">y goal:</label><br>
                <input type="text" id="y" name="y"><br>
                <label for="theta">theta goal:</label><br>
                <input type="text" id="theta" name="theta"><br>
                <input type="submit" value="GoToXYtheta">
            </form>
            
            <form action="/curve">
                <input type="submit" value="Curve">
            </form>
            
            <form action="/stop">
                <input type="submit" value="Stop">
            </form>
        </body>
        </html>
        """
    return str(html)

def rot_wheels(end_angle):
    global angle_wheel
    if angle_wheel == end_angle:
        return
    #print(f"angle : {end_angle}")
    end_angle = -end_angle
    pos = (end_angle + 120 + 24) * 100/24
    ctrl.move(1, pos, SERVO_SPEED)
    sleep(SERVO_SPEED/1000)
    angle_wheel = -end_angle
    return

def rotate(end_angle):

    global theta
    
    count = 0 #Number of RE pulses of the A channel of encoder of Motor 

    dist = 0

    rot_direction = 0 #Direction of motor LEFT = -1 | RIGHT = 1

    encA_old = encA.value() #RE for pulse Motor

     # Calculate the absolute amount of degrees to rotate from current orientation to desired orientation
    rot = end_angle - theta

    # Normalize the rotation amount to the range of -180 to 180 degrees
    rot = (rot + 180) % 360 - 180

    # Choose the smallest rotation direction
    if rot > 180:
        rot -= 360
    elif rot < -180:
        rot += 360

    if (end_angle - ANGLE_PROX_ERROR) <= theta <= (end_angle + ANGLE_PROX_ERROR):
        return
    
    elif rot > 0:
        rot_direction = 1
        rot_wheels(90)
        
        PWMA.duty_u16(WHEEL_SPEED_NORMAL)

    elif rot < 0:
        rot_direction = -1
        rot_wheels(90)
        
        PWMB.duty_u16(WHEEL_SPEED_NORMAL)

    while True:
        
        # Update the encoder, count the RE and calculate the current angle
        if (encA_old == 0 and encA.value() == 1):
            count += rot_direction
            dist = count * ToMetres
            encA_old = encA.value()
        if (encA_old == 1 and encA.value() == 0):
            encA_old = encA.value()
        
        delta_theta = 360 * dist/(radius*2*math.pi)
                     
        # If the angle approaches the end_angle reduce speed, and then stop
        if (abs(delta_theta) > (abs(rot) - ANGLE_PROX_ERROR)):
            if rot_direction == 1:
                PWMA.duty_u16(WHEEL_SPEED_PROX)
            elif rot_direction == -1:
                PWMB.duty_u16(WHEEL_SPEED_PROX)
        
        if (abs(delta_theta) > (abs(rot) - ANGLE_FINAL_ERROR)):
            
            PWMA.duty_u16(0)
            PWMB.duty_u16(0)
            break
    
    theta = (theta + delta_theta)
    rot_wheels(0)
    return

def move(xf, yf, direction):
    global x
    global y
    global theta 
    global alpha
    global end_angle

    error_dist = math.sqrt(((xf - x) ** 2) + ((yf - y) ** 2))
    
    count = 0 #Number of RE pulses of the A channel of encoder of Motor 

    dist = 0
    
    encA_old = encA.value()
    
    if direction == 1:
        PWMA.duty_u16(WHEEL_SPEED_NORMAL)
    
    elif direction == -1:
        PWMB.duty_u16(WHEEL_SPEED_NORMAL)

    while dist < error_dist:
                    
        # Update the encoder, count the RE and calculate the current angle
        if (encA_old == 0 and encA.value() == 1):
            count += 1
            dist = count * ToMetres
            encA_old = encA.value()
        if (encA_old == 1 and encA.value() == 0):
            encA_old = encA.value()
                        
        # If the distance approaches the error distance reduce speed, and then stop
        if (dist >= (error_dist - DIST_PROX_ERROR)):
            if direction ==1:
                PWMA.duty_u16(WHEEL_SPEED_PROX)
            elif direction == -1:
                PWMB.duty_u16(WHEEL_SPEED_PROX)
            
        if (dist >= (error_dist - DIST_FINAL_ERROR)):
            
            PWMA.duty_u16(0)
            PWMB.duty_u16(0)
            break

    x = x + dist*math.cos(math.radians(alpha))
    y = y + dist*math.sin(math.radians(alpha))
    return

def find_direction(xi, yi, xn, yn):
    
    global alpha
    global end_angle

    
    alpha = math.degrees(math.atan2(yn-yi, xn-xi))
    end_angle = alpha - 90

    rotate(end_angle)
    direction = 1

    return direction
 
def serve(connection):
    
    global x
    global y
    global theta
    
    
    f= open("GoToXYTheta.txt","w+")
    f.write("Go To XYTheta: \n")
    
    pos = (0 + 120 + 24) * 100/24
    ctrl.move(1, pos, SERVO_SPEED)
    sleep(SERVO_SPEED/1000)
    
    PWMA.duty_u16(0)
    PWMB.duty_u16(0)
    
    
    while True:
        client = connection.accept()[0]
        request = client.recv(1024)
        request = str(request)
        try:
            request = request.split()[1]
        except IndexError:
            pass
        
        if request == '/stop?':
            f.close()
        
        elif request == '/curve?':

            angle_rot = 25
            
            gamma = 86.5/1000
            
            rc = gamma / math.tan(math.radians(angle_rot))
            
            rot_wheels(angle_rot)
            
            direction = 1
            
            count = 0 #Number of RE pulses of the A channel of encoder of Motor 

            dist = 0
            
            encA_old = encA.value()
            
            error_dist = 2*rc*math.pi/2
            
            if direction == 1:
                PWMA.duty_u16(WHEEL_SPEED_NORMAL)
            
            elif direction == -1:
                PWMB.duty_u16(WHEEL_SPEED_NORMAL)

            while dist < error_dist:
                            
                # Update the encoder, count the RE and calculate the current angle
                if (encA_old == 0 and encA.value() == 1):
                    count += 1
                    dist = count * ToMetres
                    encA_old = encA.value()
                if (encA_old == 1 and encA.value() == 0):
                    encA_old = encA.value()
                                
                # If the angle approaches the end_angle reduce speed, and then stop
                if (dist >= (error_dist - DIST_PROX_ERROR)):
                    if direction ==1:
                        PWMA.duty_u16(WHEEL_SPEED_PROX)
                    elif direction == -1:
                        PWMB.duty_u16(WHEEL_SPEED_PROX)
                    
                if (dist >= (error_dist - DIST_FINAL_ERROR)):
                    
                    PWMA.duty_u16(0)
                    PWMB.duty_u16(0)
                    rot_wheels(0)
                    break
            
            
            
        if request == '/path?':
        
            import a_star
            i = 0

            while i < len(a_star.path) - 1:
                xi, yi = a_star.path[i]
                xn, yn = a_star.path[i+1]
                
                direction = find_direction(xi/10, yi/10, xn/10, yn/10)
            
                move(xn/10, yn/10, direction)
                
                i += 1
        elif request[0:9] == '/xytheta?':
            
            #  /xy?x=1&y=1&theta=45
            
            # Find the indices of "=" and "&" for x
            x_equal_index = request.index("x=")
            x_ampersand_index = request.index("&", x_equal_index)

            # Find the indices of "=" and the end of string for y
            y_equal_index = request.index("y=")
            y_ampersand_index = request.index("&", y_equal_index)

            # Find the indices of "=" and the end of string for y
            theta_equal_index = request.index("theta=")
            theta_end_index = len(request)

            # Extract the substrings containing x and y values
            x_string = request[x_equal_index + 2 : x_ampersand_index]
            y_string = request[y_equal_index + 2 : y_ampersand_index]
            theta_string = request[theta_equal_index + 6 : theta_end_index]
            
            xn = int(x_string)
            yn = int(y_string)
            thetan = int(theta_string)
            
            if thetan > 360:
                while thetan > 360:
                    thetan = thetan - 360
                    
            start_time = utime.ticks_ms()  # Capture the initial time
            
            error_dist = math.sqrt(((xn - x) ** 2) + ((yn - y) ** 2))
            
            if error_dist > DIST_FINAL_ERROR:
                direction = find_direction(x, y, xn, yn)
            
                move(xn, yn, direction)
            
            rotate(thetan)
            
            end_time = utime.ticks_ms()
            
            runtime = utime.ticks_diff(end_time, start_time)
             
            f.write("%f, %f, %f, %f, %f, %f, %f \n" % (xn, yn, thetan, x, y, theta, runtime))

        
    
        html = webpage()
        client.send(html)
        client.close()
 
try:
    ip = connect()
    connection = open_socket(ip)
    serve(connection)
    
except KeyboardInterrupt:
            
    PWMA.duty_u16(0)
    PWMB.duty_u16(0)
    
    machine.reset()