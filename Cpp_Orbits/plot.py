import math
import sys
import random
import matplotlib.pyplot as plot
from mpl_toolkits.mplot3d import Axes3D

def plot_output(bodies, outfile = None):
    fig = plot.figure()
    colours = ['r','b','g','y','m','c']
    ax = fig.add_subplot(1,1,1, projection='3d')
    for current_body in bodies: 
        max_range = max(max(current_body["x"]),max(current_body["y"]),max(current_body["z"]))
        ax.plot(current_body["x"], current_body["y"], current_body["z"], c = random.choice(colours), label = current_body["name"])        
    
    ax.set_xlim([-max_range,max_range])    
    ax.set_ylim([-max_range,max_range])
    ax.set_zlim([-max_range,max_range])
    ax.legend()        

    if outfile:
        plot.savefig(outfile)
    else:
        plot.show()

def read_bodies(files = []):
    bodies = []
    for file in files:
        with open(file) as f: 
            name = str(f.readline()).strip('\n')
            x = []
            y = []
            z = []

            for line in f:
                input = line.strip('\n').split(',')
                x.append(float(input[0]))
                y.append(float(input[1]))                
                z.append(float(input[2]))
        bodies.append({"name":str(name), "x":x, "y":y, "z":z})

    return bodies
        
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print ("Please include list of body files as arguments")
    names = sys.argv[1:]
    bodies = read_bodies(names)
    plot_output(bodies)
    