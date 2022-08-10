from ast import Num
from ctypes import pointer
from turtle import color
#from turtle import color
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from numpy import double, true_divide
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import pandas as pd
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots(subplot_kw={'projection': '3d'})



import os

def main():
    ##Plotted
    vertices_of_the_3Dboxes = []

    with open("export_3d.txt", "r") as file:
        for line in file:
            #Traitons une ligne de trois intervalles dans chaque direction
            list_of_intervals = line.strip().split(';') 
            lower_bounds = [] #list of lower boundaries in each direction
            widths       = []  
            for i in range(len(list_of_intervals)): #On traite chaque direction une à une
                B = list_of_intervals[i].replace('[','')
                B = B.replace(']','')
                B = B.split(',')
                w = double(B[1]) - double(B[0])
                lb = double(B[0]) #lower bound
                lower_bounds.append(lb)
                widths.append(w)
            vertices_of_the_3Dboxes.append([lower_bounds[0], lower_bounds[1], lower_bounds[2]])
            vertices_of_the_3Dboxes.append([lower_bounds[0]+widths[0], lower_bounds[1], lower_bounds[2]])
            vertices_of_the_3Dboxes.append([lower_bounds[0]+widths[0], lower_bounds[1]+widths[1], lower_bounds[2]])
            vertices_of_the_3Dboxes.append([lower_bounds[0], lower_bounds[1]+widths[1], lower_bounds[2]])
            vertices_of_the_3Dboxes.append([lower_bounds[0], lower_bounds[1], lower_bounds[2]+widths[2]])
            vertices_of_the_3Dboxes.append([lower_bounds[0]+widths[0], lower_bounds[1], lower_bounds[2]+widths[2]])
            vertices_of_the_3Dboxes.append([lower_bounds[0]+widths[0], lower_bounds[1]+widths[1], lower_bounds[2]+widths[2]])
            vertices_of_the_3Dboxes.append([lower_bounds[0], lower_bounds[1]+widths[1], lower_bounds[2]+widths[2]])

    boundaries_info = []
    with open("csv_files/boundaries.txt", "r") as file:
        for line in file:
            grade_data = line
            boundaries_info.append(double(grade_data))


    #Boundaries info
    #Plot starting and goal points
    ax.set_xlim3d(boundaries_info[0], boundaries_info[1])
    ax.set_ylim3d(boundaries_info[2], boundaries_info[3])
    ax.set_zlim3d(boundaries_info[4], boundaries_info[5])


    #Ploting vertices_of_the_3Dboxes
    Z=[]
    for i in range(len(vertices_of_the_3Dboxes)):
        Z.append(vertices_of_the_3Dboxes[i]) #On lui donne les coordonnees de chaque sommet
        if(len(Z)==8):                                                #Avec la position des 8 sommets il dessinne le polygone
            faces = [[Z[0],Z[1],Z[2],Z[3]], 
                    [Z[4],Z[5],Z[6],Z[7]],
                    [Z[0],Z[1],Z[5],Z[4]],
                    [Z[2],Z[3],Z[7],Z[6]],
                    [Z[1],Z[2],Z[6],Z[5]],
                    [Z[4],Z[7],Z[3],Z[0]]]
            collection = ax.add_collection3d(Poly3DCollection(faces, facecolors = 'magenta' ,linewidths=1, edgecolors='blue', alpha=.20))
            # collection.set_facecolor('cyan')
            Z = []


    plt.show()  


if __name__ == '__main__':
    main()
  
