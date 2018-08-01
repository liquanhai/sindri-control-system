import numpy as np
import matplotlib.pyplot as plt

sensor_names = ['Pressure', 'H2', 'Humidity', 'O2', 'Temp']

data = np.genfromtxt('output.csv', delimiter=',', skip_header=14, skip_footer=0, names=sensor_names)

pressure = data['Pressure'] 
h2 = data['H2']
humidity = data['Humidity']
o2 = data['O2']
temp = data['Temp']

plt.plot(pressure)
plt.plot(h2)
plt.plot(o2)
plt.plot(temp)

plt.show()
