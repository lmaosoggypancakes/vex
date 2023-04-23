import pandas as pd
import matplotlib.pyplot as plt

# read in the data from the CSV file
data = pd.read_csv('log.csv')
# extract the velocity and time data into separate lists
velocity_data = data['velocity'].tolist()
time_intervals = data['ms'].tolist()

# create a figure and axis object
fig, ax = plt.subplots()

# plot the velocity data as a function of time
ax.plot(time_intervals, velocity_data)

# set axis labels and title
ax.set_xlabel('Time (ms)')
ax.set_ylabel('Velocity')
ax.set_title('Flywheel Velocity over Time')

# display the plot
plt.show()