import serial
import matplotlib.pyplot as plt
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
import os
import csv
from mpl_toolkits import mplot3d

filename = "fall" # SET THIS TO WHATEVER YOU WANT TO CALL THE CSV THAT GETS CREATED

velocity = np.zeros(3)
position = np.zeros(3)

ser = serial.Serial('COM9', 115200, timeout=10)
ser.reset_input_buffer()
gyro_x = "0"
gyro_y = "0"
gyro_z = "0"
acc_x = "0"
acc_y = "0"
acc_z = "0"
is_finished_plotting = False
calibration_complete = False
calibration_has_been_checked = False
start_time = 0

position_mag = 0
velocity_mag = 0
pos_x = np.array([])
pos_y = np.array([])
pos_z = np.array([])

vel_x = np.array([])
vel_y = np.array([])
vel_z = np.array([])

dt = 0.01 # time step
gravity = np.array([0, 0, -9.81]) # gravity vector

t = np.array([]) # time
gyro_x_data = np.array([])
gyro_y_data = np.array([])
gyro_z_data = np.array([])

acc_x_data = np.array([])
acc_y_data = np.array([])
acc_z_data = np.array([])

fig,axs = plt.subplots(2, 3)

# 3D PLOT FOR VELOCITY AND POSITION
#axs3d_1 = fig.add_subplot(3, 3, 7, projection="3d")
#axs3d_1.set_title("Position")
#axs3d_1.set_xlabel('X Axis')
#axs3d_1.set_ylabel('Y Axis')
#axs3d_1.set_zlabel('Z Axis')
#axs3d_2 = fig.add_subplot(3, 3, 8, projection="3d")
#axs3d_2.set_title("Velocity")

csv_file_name = "csv"
data = [ ["gyro x", "gyro y", "gyro z", "acc x", "acc y", "acc z"] ]


axs[0, 0].set_title("Gyroscope x")
axs[0, 0].set_xlabel("Time (s)")

axs[0, 1].set_title("Gyroscope y")
axs[0, 1].set_xlabel("Time (s)")

axs[0, 2].set_title("Gyroscope z")
axs[0, 2].set_xlabel("Time (s)")

axs[1, 0].set_title("Accelerometer x")
axs[1, 0].set_xlabel("Time (s)")

axs[1, 1].set_title("Accelerometer y")
axs[1, 1].set_xlabel("Time (s)")

axs[1, 2].set_title("Accelerometer z")
axs[1, 2].set_xlabel("Time (s)")

# hide bottom 3 axesk
#axs[2, 0].set_axis_off()
#axs[2, 1].set_axis_off()
#axs[2, 2].set_axis_off()

while is_finished_plotting != True:
    msg = ser.readline()
    decoded_msg = msg.decode('utf-8')

    if calibration_complete == True and calibration_has_been_checked == False: # check if calibration is complete, if it is, then start timer
        start_time = time.time()
        calibration_has_been_checked = True

    if "'" in decoded_msg: # this character only appears in data collecting lines, so this runs when data is being collected
        if calibration_complete == True:
            if time.time() - start_time < 10:
                formatted_msg = decoded_msg.replace(r"\r\n", "")
                msg_list = formatted_msg.split("'")

                acc_x = float(msg_list[0])
                acc_x_data = np.append(acc_x_data, acc_x) # append new data to array

                acc_y = float(msg_list[1])
                acc_y_data = np.append(acc_y_data, acc_y)

                acc_z = float(msg_list[2])
                acc_z_data = np.append(acc_z_data, acc_z)

                gyro_x = float(msg_list[3])
                gyro_x_data = np.append(gyro_x_data, gyro_x)

                gyro_y = float(msg_list[4])
                gyro_y_data = np.append(gyro_y_data, gyro_y)

                gyro_z = float(msg_list[5])
                gyro_z_data = np.append(gyro_z_data, gyro_z)
            
                # create world coordinate trajectory
                imu_orientation = R.from_euler('xyz', [gyro_x, gyro_y, gyro_z], degrees=True)
                world_accel = np.array([acc_x, acc_y, acc_z])
                linear_accel = world_accel - [0, 0, 1.00] # for some reason needed to subtract 1 for this to be calibrated?

                velocity += linear_accel * dt
                position += velocity * dt

                pos_x = np.append(pos_x, position[0])
                pos_y = np.append(pos_y, position[1])
                pos_z = np.append(pos_z, position[2])

                # add time to time array
                t = np.append(t, time.time()-start_time)

                # append new information to data for csv file later
                data.append([gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z])
                print(position)

            else:
                is_finished_plotting = True # exit loop
        else:
            calibration_complete = True
    else:
        print(decoded_msg)

# check which number to put after csv file
i = 0
while os.path.exists(f"{filename}{i}.csv"):
    i += 1

j = 0
while os.path.exists(f"{filename}{j}.png"):
    j += 1

# write data to csv file
with open(f"{filename}{i}.csv", mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerows(data) # write data to csv

axs[0, 0].plot(t, gyro_x_data)
axs[0, 1].plot(t, gyro_y_data)
axs[0, 2].plot(t, gyro_z_data)

axs[1, 0].plot(t, acc_x_data)
axs[1, 1].plot(t, acc_y_data)
axs[1, 2].plot(t, acc_z_data)

#axs3d_1.plot3D(pos_x, pos_y, pos_z, 'green')

plt.tight_layout() # adjust layout of resulting image so it looks nice and titles don't overlap
plt.savefig(f"{filename}{j}") # save plot as image
plt.show()