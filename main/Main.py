import matplotlib.pyplot as plt
import numpy as np
import csv
import math

# Configuring the lib
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
plt.autoscale()

file = open('./Third FUcking Working Coordinates.csv')
type(file)
csvreader = csv.reader(file)

# Reading the file and putting the values in 3 arrays, one for each dimension
xs = []
ys = []
zs = []
colors = []
for row in csvreader:
        xs.append(float(row[0]))
        ys.append(float(row[1]))
        zs.append(float(row[2]))
        colors.append('b')

file.close()

ranges = []

currentCoord = 0

# Calculating all coordinates ranges sum from all other coordinates
for (x1,y1,z1,color) in zip(xs, ys, zs, colors):
        rangesSum = 0
        for (x2,y2,z2) in zip(xs, ys, zs):
                # Adding the current coordinate range to the main coordinate ranges sum
                rangesSum += math.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) +pow(z1 - z2, 2))

        ranges.append(rangesSum)
        print("Coord number " ,currentCoord, " ranges sum has been calculated")
        currentCoord+=1

# Sorting all coordinates by their ranges
sorted_pairs = sorted(zip(xs, ys, zs, colors, ranges), key=lambda coord:coord[4])

# Extracting all sorted arrays in order to insert them to our gui
sortedXs, sortedYs, sortedZs, sortedColors, SortedRanges = [ list(tuple) for tuple in  zip(*sorted_pairs)]


# Letting the highest 60 ranges be in red color
for i in range(60):
        if i <= 60:
                colors[colors.__len__() - i -1] = 'r'
        else:
                break

# Inserting the coordinates + colors
ax.scatter(sortedXs, sortedYs, sortedZs, s = 5, depthshade = False,zdir = 'z', c =colors)



# Inserting the limits in order to see the model well
ax.set_xlim3d(-0.1,0.1)
ax.set_ylim3d(-0.1,0.1)
ax.set_zlim3d(-0.1,0.1)

plt.show()
