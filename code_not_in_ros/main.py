from map_gen import FrobomindMap
from bug2 import Bug2Algorithm
import numpy

Map = FrobomindMap(visualize=True)
"""
m = numpy.zeros((300,300))
#Map.genMap()
#Map.getLocalMap(3,4,m)
#Map.writeLocalMap(3,4,m)
Map.visualize_now(0,2)
"""
"""
x = 1
y = 1

for i in range(0,20):
    inkey = raw_input("Press wsad: ")
    if inkey=='w':
        print("up")
        x = x - 1
    elif inkey=='s':
        print "down"
        x = x + 1
    elif inkey=='d':
        print "right"
        y = y + 1
    elif inkey=='a':
        print "left"
        y = y - 1
    else:
        print "not an arrow key!"
    print(x,y)
    Map.visualize_now(x,y)
"""
algorithm = Bug2Algorithm()
algorithm.run_bug()