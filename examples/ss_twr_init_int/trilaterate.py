import serial
import numpy as np
import matplotlib.pyplot as plt
import re
import threading
import math


ser = serial.Serial('COM9', 115200, timeout=1)

x_values = []
y_values = []
z_values = []
data = []
pattern2 = re.compile(r"seat_map = ([0-9A-Fa-f]+)")
pattern = re.compile(r"SEAT \d+: (\d+\.\d+), \((\d+), (\d+), (\d+)\)")


def trilaterate3D(distances):
    p1=np.array(distances[0][:3])
    p2=np.array(distances[1][:3])
    p3=np.array(distances[2][:3])       
    p4=np.array(distances[3][:3])
    r1=distances[0][-1]
    r2=distances[1][-1]
    r3=distances[2][-1]
    r4=distances[3][-1]
    e_x=(p2-p1)/np.linalg.norm(p2-p1)
    i=np.dot(e_x,(p3-p1))
    e_y=(p3-p1-(i*e_x))/(np.linalg.norm(p3-p1-(i*e_x)))
    e_z=np.cross(e_x,e_y)
    d=np.linalg.norm(p2-p1)
    j=np.dot(e_y,(p3-p1))
    x=((r1**2)-(r2**2)+(d**2))/(2*d)
    y=(((r1**2)-(r3**2)+(i**2)+(j**2))/(2*j))-((i/j)*(x))
    z1=np.sqrt(r1**2-x**2-y**2)
    z2=np.sqrt(r1**2-x**2-y**2)*(-1)
    ans1=p1+(x*e_x)+(y*e_y)+(z1*e_z)
    ans2=p1+(x*e_x)+(y*e_y)+(z2*e_z)
    dist1=np.linalg.norm(p4-ans1)
    dist2=np.linalg.norm(p4-ans2)
    if np.abs(r4-dist1)<np.abs(r4-dist2):
        return ans1
    else: 
        return ans2


def read_uart():
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                match2 = pattern2.match(line)
                if match2:
                    seat_map = bin(int(match2.group(1), 16))
                    num_network_devices = seat_map.count('1')
                    for i in range(1,num_network_devices):
                        line = ser.readline().decode('utf-8').strip()
                        if line:
                            match = pattern.match(line)
                            if match:
                                d = float(match.group(1))
                                x = int(match.group(2))
                                y = int(match.group(3))
                                z = int(match.group(4))
                                x_values.append(x)
                                y_values.append(y)
                                data.append([x,y,z,d])
                                if len(x_values) > num_network_devices:
                                    x_values.pop(0)
                                    y_values.pop(0)
                                    data.pop(0)
        except KeyboardInterrupt:
            ser.close()
            quit()
        except Exception as e:
            print(f"Error: {e}")
            ser.close()


def update_plot():
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots(figsize=(6, 6))

    while True:
        if x_values and y_values:
            ax.clear()
            ax.set_xlim(-10, 10)
            ax.set_ylim(-10, 10)
            ax.set_xticks(range(-10,11))
            ax.set_yticks(range(-10,11))

            ax.set_title('Network Device Locations')
            ax.grid(True)
            ax.scatter(x_values, y_values, color='blue')
            data.sort(key=lambda item: item[3])
            location = trilaterate3D(data)
            if ((math.isnan(location[0])==False)  and  (math.isnan(location[1])==False)):
                ax.plot(location[0], location[1], 'ro')  # 'ro' means red circle
                ax.text(location[0] + 0.1, location[1] + 0.1, f'({round(location[0],2)}, {round(location[1],2)})', fontsize=9)
            for i, (x, y) in enumerate(zip(x_values, y_values)):
                ax.text(x + 0.1, y + 0.1, f'({x}, {y})', fontsize=9)
            plt.draw()
            plt.pause(0.1)



uart_thread = threading.Thread(target=read_uart)
uart_thread.daemon = True
uart_thread.start()

try:
    update_plot()
except KeyboardInterrupt:
    print("Program terminated")
    ser.close()
    quit()
finally:
    ser.close()
    quit()