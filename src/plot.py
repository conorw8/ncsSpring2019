#!/usr/bin/env python
import matplotlib.pyplot as plt
import csv

x = []
y = []
x2 = []
y2 = []

with open('timeDelay1meter.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    i = 0
    for row in plots:
        y.append(float(row[0]))
        x.append(i)
        i += 1

with open('timeDelay10meter.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    k = 0
    for row in plots:
        y2.append(float(row[0]))
        x2.append(k)
        k += 1

plt.plot(x,y, label='Distance: 0 meters')
plt.plot(x2,y2, label='Distance: 10 meters')
plt.xlabel('iterations')
plt.ylabel('ms')
plt.title('Network Delay')
plt.legend()
plt.show()
