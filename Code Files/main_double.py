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

x = 0 # x coordinate of robot
y = 0 # y coordinate of robot
theta = 0 # angle orientation of robot
alpha = 0 # angle between line to follow and x-axis
angle_wheel = 0 # angle that the wheels are turned to, starting at y-axis
DIST_PROX_ERROR = 0.01
DIST_FINAL_ERROR = 0.001
ANGLE_PROX_ERROR = 5
ANGLE_FINAL_ERROR = 0.1
WHEEL_SPEED_NORMAL = 25000 # PWM frequency
WHEEL_SPEED_PROX = 11000
RIGHT_ADJUST_THETA = 0
LEFT_ADJUST_THETA = 0
MAX_DEVIATION = 10
ToMetres = 0.219/960
WheelDist = 130/1000 #metres
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
encA_mA=machine.Pin(26,machine.Pin.IN, Pin.PULL_DOWN)
encB_mA=machine.Pin(27,machine.Pin.IN, Pin.PULL_DOWN)

encA_mB=machine.Pin(9,machine.Pin.IN, Pin.PULL_DOWN)
encB_mB=machine.Pin(22,machine.Pin.IN, Pin.PULL_DOWN)

#motor pins

# motor 2 PICO4DRIVE
# Red to A
# Black to B
PWMA_mA = PWM(Pin(13)) #Forwards
PWMB_mA = PWM(Pin(12)) #Backwards

# motor 4 PICO4DRIVE
# Red to B
# Black to A
PWMA_mB = PWM(Pin(17)) #Forwards
PWMB_mB = PWM(Pin(16)) #Backwards
 
 
PWMA_mA.freq(1000) 
PWMB_mA.freq(1000)

PWMA_mB.freq(1000) 
PWMB_mB.freq(1000)


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

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def rotate(end_angle):
    global theta
    
    rotate_wheels(0)
        
    count_A = 0 #Number of RE pulses of the A channel of encoder of Motor A
    count_B = 0 #Number of RE pulses of the A channel of encoder of Motor B
    
    dist_A=0
    dist_B=0
    
    encA_mA_old = encA_mA.value() #RE for pulse Motor A
    encA_mB_old = encA_mB.value() #RE for pulse Motor B

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
        direction_A = -1
        direction_B = 1

        PWMB_mA.duty_u16(WHEEL_SPEED_NORMAL)
        PWMA_mB.duty_u16(WHEEL_SPEED_NORMAL)
        
    elif rot < 0:
        direction_A = 1
        direction_B = -1

        PWMA_mA.duty_u16(WHEEL_SPEED_NORMAL)
        PWMB_mB.duty_u16(WHEEL_SPEED_NORMAL)
    
    while True:
                             
        # Update the encoder, count the RE and calculate the current angle
        if (encA_mA_old == 0 and encA_mA.value() == 1):
            count_A += 1
            dist_A = count_A * ToMetres
            encA_mA_old = encA_mA.value()
        if (encA_mA_old == 1 and encA_mA.value() == 0):
            encA_mA_old = encA_mA.value()
            
        if (encA_mB_old == 0 and encB_mB.value() == 1):
            count_B += 1
            dist_B = count_B * ToMetres
            encA_mB_old = encA_mB.value()
        if (encA_mB_old == 1 and encA_mB.value() == 0):
             encA_mB_old = encA_mB.value()
        
        dist_avg = (dist_A + dist_B)/2
        delta_theta = 360 * dist_avg/(WheelDist*math.pi)
                     
        # If the angle approaches the end_angle reduce speed, and then stop
        if (delta_theta > (abs(rot) - ANGLE_PROX_ERROR)):
            if direction_A == 1:
                PWMA_mA.duty_u16(WHEEL_SPEED_PROX)
                PWMB_mB.duty_u16(WHEEL_SPEED_PROX)
            elif direction_A == -1:
                PWMB_mA.duty_u16(WHEEL_SPEED_PROX)
                PWMA_mB.duty_u16(WHEEL_SPEED_PROX)
        
        if (delta_theta > (abs(rot) - ANGLE_FINAL_ERROR)):
            
            PWMA_mA.duty_u16(0)
            PWMB_mA.duty_u16(0)
            PWMA_mB.duty_u16(0)
            PWMB_mB.duty_u16(0)
            break
    
    theta = (theta+(direction_B * delta_theta))
    
    return

def rotate_wheels(end_angle):
    global angle_wheel
    if angle_wheel == end_angle:
        return
    end_angle = -end_angle
    pos = (end_angle + 120 + 24) * 100/24
    ctrl.move(1, pos, SERVO_SPEED)
    ctrl.move(2, pos+10, SERVO_SPEED)
    sleep(SERVO_SPEED/1000)
    return

def move(xf, yf, direction_A):
    global x
    global y
    global theta 
    global alpha
    global LEFT_ADJUST_THETA
    global RIGHT_ADJUST_THETA

    error_dist = math.sqrt(((xf - x) ** 2) + ((yf - y) ** 2))
    
    # 960 for one full spin of the wheel
    # 68mm diameter -> 219mm perimeter -> 0,219 m perimeter
    # 960 pulses per 0.2m
    # count = d*960/0.219
        
    count_A = 0 #Number of RE pulses of the A channel of encoder of Motor A
    count_B = 0 #Number of RE pulses of the A channel of encoder of Motor B

    dist_A = 0
    dist_B = 0
    dist_avg = 0
    
    encA_mA_old = encA_mA.value() #RE for pulse Motor A
    encA_mB_old = encA_mB.value() #RE for pulse Motor B

    if direction_A == 1:
        PWMA_mA.duty_u16(WHEEL_SPEED_NORMAL)
        PWMA_mB.duty_u16(WHEEL_SPEED_NORMAL)

    elif direction_A == -1:
        PWMB_mA.duty_u16(WHEEL_SPEED_NORMAL)
        PWMB_mB.duty_u16(WHEEL_SPEED_NORMAL)
    
    while dist_avg < error_dist:
                             
        # Update the encoder, count the RE and calculate the current distance
        if (encA_mA_old == 0 and encA_mA.value() == 1):
            count_A += 1
            dist_A = count_A * ToMetres
            encA_mA_old = encA_mA.value()
        if (encA_mA_old == 1 and encA_mA.value() == 0):
            encA_mA_old = encA_mA.value()
            
        if (encA_mB_old == 0 and encB_mB.value() == 1):
            count_B += 1
            dist_B = count_B * ToMetres
            encA_mB_old = encA_mB.value()
        if (encA_mB_old == 1 and encA_mB.value() == 0):
             encA_mB_old = encA_mB.value()
        
        
        dist_avg = (dist_A + dist_B)/2
        delta_theta = (dist_B-dist_A) * 360/(WheelDist*math.pi)
        
        if delta_theta > MAX_DEVIATION:
            RIGHT_ADJUST_THETA = 0
            LEFT_ADJUST_THETA = 1000
        
        elif delta_theta < -MAX_DEVIATION:
            RIGHT_ADJUST_THETA = 1000
            LEFT_ADJUST_THETA = 0
        
        else:
            RIGHT_ADJUST_THETA = 0
            LEFT_ADJUST_THETA = 0
             
        if direction_A == 1:
            PWMA_mA.duty_u16(WHEEL_SPEED_NORMAL + LEFT_ADJUST_THETA)
            PWMA_mB.duty_u16(WHEEL_SPEED_NORMAL + RIGHT_ADJUST_THETA)
 
        elif direction_A == -1:
            PWMB_mA.duty_u16(WHEEL_SPEED_NORMAL + LEFT_ADJUST_THETA)
            PWMB_mB.duty_u16(WHEEL_SPEED_NORMAL + RIGHT_ADJUST_THETA)
                     
        # If the distance approaches the error distance reduce speed, and then stop
        if (dist_avg >= (error_dist - DIST_PROX_ERROR)):
            if direction_A == 1:
                PWMA_mA.duty_u16(WHEEL_SPEED_PROX)
                PWMA_mB.duty_u16(WHEEL_SPEED_PROX)
            elif direction_A == -1:
                PWMB_mA.duty_u16(WHEEL_SPEED_PROX)
                PWMB_mB.duty_u16(WHEEL_SPEED_PROX)
          
        if (dist_avg >= (error_dist - DIST_FINAL_ERROR)):
            
            PWMA_mA.duty_u16(0)
            PWMB_mA.duty_u16(0)
            PWMA_mB.duty_u16(0)
            PWMB_mB.duty_u16(0)
            break
        
    x = x + dist_avg*math.cos(math.radians(alpha))
    y = y + dist_avg*math.sin(math.radians(alpha))
    #theta = normalize_angle(theta+delta_theta)
    
    return

def find_direction(xi, yi, xn, yn):

    global angle_wheel
    global alpha
    global theta

    # Calculate angle of wheels for servo
    alpha = math.degrees(math.atan2(yn-yi, xn-xi))
    end_angle_wheel = alpha - 90 - theta
    
    i = 0
    direction_A = 1
    if end_angle_wheel > 100 or end_angle_wheel < -100:
      while end_angle_wheel > 100 or end_angle_wheel < -100:
          end_angle_wheel = (abs(end_angle_wheel) - 180) * (end_angle_wheel/abs(end_angle_wheel))
          i += 1
      direction_A = math.pow(-1,i)
      
    rotate_wheels(end_angle_wheel)
    angle_wheel = end_angle_wheel
      
    return direction_A

def serve(connection):
    
    global x
    global y
    global theta
    
    f= open("GoToXYTheta.txt","w+")
    f.write("Go To XYTheta: \n")
    
    pos = (0 + 120 + 24) * 100/24
    ctrl.move(1, pos, SERVO_SPEED)
    ctrl.move(2, pos+10, SERVO_SPEED)
    sleep(SERVO_SPEED/1000)

    direction_A = 1 #Direction of motor A LEFT = -1 | RIGHT = 1
    #direction_B = 1 #Direction of motor B LEFT = -1 | RIGHT = 1
    
    pos = 0
    
    PWMA_mA.duty_u16(0)
    PWMB_mA.duty_u16(0)
    
    PWMA_mB.duty_u16(0)
    PWMB_mB.duty_u16(0)
    
    while True:
        client = connection.accept()[0]
        request = client.recv(1024)
        request = str(request)
        try:
            request = request.split()[1]
        except IndexError:
            pass
        
        #print(request)
        
        if request == '/stop?':
            f.close()
        
        elif request == '/curve?':
            
            direction_A = 1
            
            delta_theta = 0
                
            count_A = 0 #Number of RE pulses of the A channel of encoder of Motor A
            count_B = 0 #Number of RE pulses of the A channel of encoder of Motor B

            dist_A = 0
            dist_B = 0
            dist_avg = 0
            
            encA_mA_old = encA_mA.value() #RE for pulse Motor A
            encA_mB_old = encA_mB.value() #RE for pulse Motor B

            if direction_A == 1:
                PWMA_mA.duty_u16(WHEEL_SPEED_NORMAL)
                PWMA_mB.duty_u16(WHEEL_SPEED_NORMAL-8000)

            elif direction_A == -1:
                PWMB_mA.duty_u16(WHEEL_SPEED_NORMAL)
                PWMB_mB.duty_u16(WHEEL_SPEED_NORMAL)
                
            left_speed = 0.3124024
            right_speed = 0.1906214
            
            angular_velocity = ((right_speed - left_speed) / WheelDist) * 180/math.pi
            
            start_time = utime.ticks_ms()
            
            while abs(delta_theta) < 180:
                                     
                # Update the encoder, count the RE and calculate the current angle
                if (encA_mA_old == 0 and encA_mA.value() == 1):
                    count_A += 1
                    dist_A = count_A * ToMetres
                    encA_mA_old = encA_mA.value()
                if (encA_mA_old == 1 and encA_mA.value() == 0):
                    encA_mA_old = encA_mA.value()
                    
                if (encA_mB_old == 0 and encB_mB.value() == 1):
                    count_B += 1
                    dist_B = count_B * ToMetres
                    encA_mB_old = encA_mB.value()
                if (encA_mB_old == 1 and encA_mB.value() == 0):
                     encA_mB_old = encA_mB.value()
                
                
                #dist_avg = (dist_A + dist_B)/2
                #delta_theta = (dist_B-dist_A) * 360/(WheelDist*2*math.pi)
                current_time = utime.ticks_ms()
                elapsed_time = utime.ticks_diff(current_time, start_time)
                if elapsed_time >= 10:
                    delta_theta = delta_theta + angular_velocity * elapsed_time/1000
                    start_time = current_time
                             
                # If the angle approaches the end_angle reduce speed, and then stop
                if (abs(delta_theta) >= 170):
                    if direction_A == 1:
                        PWMA_mA.duty_u16(WHEEL_SPEED_PROX+2000)
                        PWMA_mB.duty_u16(WHEEL_SPEED_PROX)
                    elif direction_A == -1:
                        PWMB_mA.duty_u16(WHEEL_SPEED_PROX)
                        PWMB_mB.duty_u16(WHEEL_SPEED_PROX)
                    left_speed = 0.1310341
                    right_speed = 0.1030524
                    
                    angular_velocity = ((right_speed - left_speed) / WheelDist) * 180/math.pi
                  
                if (abs(delta_theta) >= 179):
                    
                    PWMA_mA.duty_u16(0)
                    PWMB_mA.duty_u16(0)
                    PWMA_mB.duty_u16(0)
                    PWMB_mB.duty_u16(0)
                    break
            
        elif request == '/path?':
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
            
    PWMA_mA.duty_u16(0)
    PWMB_mA.duty_u16(0)
    
    PWMA_mB.duty_u16(0)
    PWMB_mB.duty_u16(0)
    
    machine.reset()
