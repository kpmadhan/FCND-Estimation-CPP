import pandas as pd

df = pd.read_csv('/Users/qfinnova/Documents/HandsOn/AutonomousDrones/FCND-Etimation-CPP/FCND-Estimation-CPP/config/log/Graph1.txt', header = 2)
print (df.get_values().std())

df = pd.read_csv('/Users/qfinnova/Documents/HandsOn/AutonomousDrones/FCND-Etimation-CPP/FCND-Estimation-CPP/config/log/Graph2.txt', header = 2)
print (df.get_values().std())

import numpy as np

gps_x = np.loadtxt('/Users/qfinnova/Documents/HandsOn/AutonomousDrones/FCND-Etimation-CPP/FCND-Estimation-CPP/config/log/Graph1.txt',delimiter=',',dtype='Float64',skiprows=1)[:,1]
acc_x = np.loadtxt('/Users/qfinnova/Documents/HandsOn/AutonomousDrones/FCND-Etimation-CPP/FCND-Estimation-CPP/config/log/Graph2.txt',delimiter=',',dtype='Float64',skiprows=1)[:,1]

gps_x_std  = np.std(gps_x)
print(f'GPS X Std: {gps_x_std}')
acc_x_std = np.std(acc_x)
print(f'Accelerometer X Std: {acc_x_std}')