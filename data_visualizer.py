import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
PORT = 'rfc2217://localhost:4000'
print('Connecting to Simulation')
try:
    connection=serial.serial_for_url(PORT,baudrate=115200)
    print("Success: Connected to WOKWI")
except Exception as e :
    print("Error: {e}")
    print("Some error has occured!")
    exit()
pitch_vals =[]
roll_vals=[]
def update(frame):
    while connection.in_waiting > 0:
      try:
          line = connection.readline().decode('utf-8').strip()
          if "Pitch" in line:
              parts = line.split(',')
              p=float(parts[0].split(':')[1])
              r=float(parts[1].split(':')[1])
              pitch_vals.append(p)
              roll_vals.append(r)

              if len(pitch_vals)>50:
                  pitch_vals.pop(0)
                  roll_vals.pop(0)
      except:
          pass

# Draw the frame
    plt.cla()
    plt.plot(pitch_vals, label='Pitch', color='#007acc', linewidth=2)
    plt.plot(roll_vals, label='Roll', color='#ff5500', linewidth=2)
    plt.ylim(-90, 90)
    plt.title("Real-Time STM32 IMU Data")
    plt.ylabel("Angle (Degrees)")
    plt.legend(loc='upper right')
    plt.grid(True)

    

print("Graphing Data... Press Ctrl+C to stop")
ani=FuncAnimation(plt.gcf(),update,interval=50)
plt.show()

