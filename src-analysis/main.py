import serial
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
from scipy.optimize import curve_fit
from matplotlib import style


def setup_serial():
    """ Setup serial connection object"""
    arduino_com_port = "/dev/tty.usbmodem14201"
    baud_rate = 115200
    serial_port = serial.Serial(arduino_com_port, baud_rate, timeout=1)
    return serial_port


def data_map(r, theta, phi):
    """remap single data value
    r: distance reading (scaled by sensor_2_distance
    theta: yaw angle (centered)
    phi: pitch angle (centered)
    returns: tulple of x,y,z coords"""
    theta_r = theta * (math.pi / 180)
    phi_r = phi * (math.pi / 180)
    cart = (r * math.sin(phi_r) * math.cos(theta_r),
            r * math.sin(phi_r) * math.sin(theta_r),
            r * math.cos(phi_r))
    return cart


def remap(old_val, old_min, old_max, new_min, new_max):
    """ remaps numbers between two ranges
    old_val: input value
    old_min: range of valid input min
    old_max: range of valid input max
    new_min: range of valid output min
    new_max: range of valid output max
    returns: output scaled value
    """
    old_range = (old_max - old_min)
    new_range = (new_max - new_min)
    new_value = (((old_val - old_min) * new_range) / old_range) + new_min
    return new_value


def calibrate_curve(x, y):
    """run a polyfit on sensor data
    x: sensor measurements as np array
    y: decided distance increments
    returns: fit and coefficients of fit
    """
    z = np.polyfit(x, y, 3)
    f = np.poly1d(z)
    return z, f


def plot_calibration(x,y):
    """run a polyfit on sensor data, and generate a plot of
    the original date alongside the fit. Blocking operation.
    x: sensor measurements as np array
    y: decided distance increments
    """
    z,f = calibrate_curve(x, y)
    x_new = np.linspace(x[0], y[-1], 50)
    y_new = f(x_new)

    print(f)
    print(z)

    fig, ax = plt.subplots()
    ax.plot(x, y, 'o', label="measured data")
    ax.plot(x_new, y_new, label=f)
    ax.set_xlim([x[0]-1, y[-1] + 1])
    ax.set_title("3 term polynomial fit of sensor data")
    plt.legend(fontsize="small")
    ax.set_xlabel("sensor reading (unitless)")
    ax.set_ylabel("distance (cm)")
    ax.grid(True, linestyle='-', color='0.75')
    fig.savefig('sensor_curve_fit.png', dpi=300)

    plt.show()


def sensor_2_distance(reading):
    """convert from reading to distance measured in cm
    reading: unscaled value from IR sensor
    returns: converted distance in cm"""
    readings = np.asarray([640, 452, 300, 232, 195, 155, 137, 125, 115, 100])
    distances = np.asarray([15, 30, 45, 60, 75, 90, 105, 120, 135, 150])

    coeffs, fit_func = calibrate_curve(readings, distances)

    return fit_func(reading)


def logger(time, distance, yaw, pitch, cart):
    """ print out information during the scanning loop
    time: serial output time
    distance: serial output distance
    yaw: serial output yaw
    pitch: serial output pitch
    cart: tuple of converted cart coords"""
    print("time:", time,
          "distance: ", distance, "cm",
          "yaw: ", yaw,
          "pitch: ", pitch,
          "cart: ", cart)


def sensor_loop_simple():
    """ Simple utility serial printer for calculating error residuals"""
    serial_handler = setup_serial()
    run = input("run experiment?")  # Python 3
    if run == "y":
        serial_handler.write(b'1')
    while run == "y":
        line_of_data = serial_handler.readline().decode()
        if line_of_data:
            distance = float(line_of_data.replace("\r\n", ""))

            scaled_distance = sensor_2_distance(distance)
            print("raw distance", distance, "scaled distance", scaled_distance, "cm")


def main_loop():
    """ Main execution loop for the script"""
    serial_handler = setup_serial()
    data_recorder = [["t", "x", "y", "z", "r", "theta", "phi"]]
    run = input("run experiment?")  # Python 3
    if run == "y":
        serial_handler.write(b'1')
    while run == "y":
        line_of_data = serial_handler.readline().decode()
        print(line_of_data)
        if line_of_data:
            # split serial input by commas
            millis, data, yaw, pitch = [float(x) for x in line_of_data.replace("\r\n", "").split(',')]

            # scale values according to calibration and angle math
            scaled_distance = sensor_2_distance(data)
            mapped_angle_theta = remap(yaw, 0, 180, -90, 90)
            mapped_angle_phi = remap(pitch, 0, 180, -90, 90)
            mapped_cart = data_map(scaled_distance,
                                   yaw,
                                   pitch)

            # logging during operation
            logger(millis,
                   scaled_distance,
                   yaw,
                   pitch,
                   mapped_cart)

            # record for later processing and saving
            data_recorder.append([millis,
                                  mapped_cart[0],
                                  mapped_cart[1],
                                  mapped_cart[2],
                                  scaled_distance,
                                  yaw,
                                  pitch])

            if yaw > 170:
                run = "n"
                break

    # convert to a dataframe and save to csv
    df = pd.DataFrame(data_recorder[1:], columns=data_recorder[0])
    df.to_csv("big_letter_1deg_step_2D", sep=',', encoding='utf-8')
    print(df)


# main_loop()
# sensor_loop()
sensor_loop_simple()
