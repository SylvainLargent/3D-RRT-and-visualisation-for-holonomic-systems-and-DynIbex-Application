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
    trajectory_b  = False                 #Plot the trajectory found by the path planning ?
    plot_dynibex = False                  #Post treatment data for dynibex visualisation
    plot_actual_position = False          #Plot the trajectory measured
    plot_position_animation = False
    plot_start_end = True

    if(trajectory_b):
        animation             = False
        plot_tree             = False
        animation_trajectory  = False
        plot_trajectory_bool  = True    
    else :
        animation             = True
        plot_tree             = True
        animation_trajectory  = True
        plot_trajectory_bool  = True


    ##Plotted
    trajectory = []
    tree = []
    obstacles = []
    position = []

    if(plot_trajectory_bool):
        with open("csv_files/trajectory.txt", "r") as file:
            for line in file:
                trajectoryString = line.strip().split(',')
                vector = []
                for i in range(len(trajectoryString)):
                    vector.append(double(trajectoryString[i]))
                trajectory.append(vector)
        Number_of_iteration = trajectory.pop()   #Vector containing an integer, number of iteration till goal
        print(Number_of_iteration)

    if(plot_tree == True):
        with open("csv_files/tree.txt", "r") as file:
            for line in file:
                treeString = line.strip().split(',')
                vector = []
                for i in range(len(treeString)):
                    vector.append(double(treeString[i]))
                tree.append(vector)

    with open("csv_files/obstacles.txt", "r") as file:
        for line in file:
            obstacleString = line.strip().split(',')
            vector = []
            for i in range(len(obstacleString)):
                vector.append(double(obstacleString[i]))
            obstacles.append(vector)

    boundaries_info = []
    with open("csv_files/boundaries.txt", "r") as file:
        for line in file:
            boundariesString = line
            boundaries_info.append(double(boundariesString))

    start_destination = []
    with open("csv_files/start_destination.txt", "r") as file:
        for line in file:
            start_or_destination_String = line.strip().split(',')
            vectorPOS = []
            for i in range(len(start_or_destination_String)):
                start_destination.append(double(start_or_destination_String[i]))

    #Boundaries info
    #Plot starting and goal points
    ax.set_xlim3d(boundaries_info[0], boundaries_info[1])
    ax.set_ylim3d(boundaries_info[2], boundaries_info[3])
    ax.set_zlim3d(boundaries_info[4], boundaries_info[5])

    if(plot_start_end):
        ax.scatter(start_destination[0],start_destination[1],start_destination[2], color = 'r',s = 200)
        ax.scatter(start_destination[3],start_destination[4],start_destination[5], color = 'r', s = 200)

    #Ploting obstacles
    Z=[]
    for i in range(len(obstacles)):
        Z.append([obstacles[i][0], obstacles[i][1], obstacles[i][2]]) #On lui donne les coordonnees de chaque sommet
        if(len(Z)==8):                                                #Avec la position des 8 sommets il dessinne le polygone
            faces = [[Z[0],Z[1],Z[2],Z[3]], 
                    [Z[4],Z[5],Z[6],Z[7]],
                    [Z[0],Z[1],Z[5],Z[4]],
                    [Z[2],Z[3],Z[7],Z[6]],
                    [Z[1],Z[2],Z[6],Z[5]],
                    [Z[4],Z[7],Z[3],Z[0]]]
            collection = ax.add_collection3d(Poly3DCollection(faces, facecolors = 'orange' ,linewidths=1, edgecolors='g', alpha=.20))
            # collection.set_facecolor('cyan')
            Z = []

    #Plotting the tree
    if(plot_tree == True):
        for i in range(1,len(tree),2):
            xt =[tree[i][0], tree[i-1][0]]
            yt =[tree[i][1], tree[i-1][1]]
            zt =[tree[i][2], tree[i-1][2]] 
            ax.plot(xt, yt, zt, label='tree', color = 'b')
            if(animation):
                plt.pause(0.000000000000000001)

    #Plotting the trajectory
    if(plot_trajectory_bool):
        xs = []
        ys = []
        zs = []
        for i in range(0,len(trajectory)):
            xs.append(trajectory[i][0])
            ys.append(trajectory[i][1])
            zs.append(trajectory[i][2])
            ax.plot(xs, ys, zs, label='parametric curve %i' %Number_of_iteration[0], color = 'r')
            if(animation_trajectory):
                    plt.pause(0.000000000000000001)

    
   
    if(plot_actual_position):
        if(plot_position_animation):
            def animate(i):
                line = pd.read_csv('csv_files/position.txt', sep=";|,", engine="python", skipfooter=1, header = None, usecols=[0,1,2], names=['x', 'y','z'])
                x = line["x"]
                y = line["y"]
                z = line["z"]
                ax.plot3D(x, y, z, 'green')
            ani = FuncAnimation(plt.gcf(), animate, interval=500)

        if(not plot_position_animation):
            with open("csv_files/position.txt", "r") as file:
                for line in file:
                    grade_data = line.strip().split(',')
                    vector = []
                    for i in range(len(grade_data)):
                        vector.append(double(grade_data[i]))
                    position.append(vector)

            ##Plotting the position
                xs = []
                ys = []
                zs = []
                for i in range(0,len(position)):
                    if(i%100 ==0):
                        xs.append(position[i][0])
                        ys.append(position[i][1])
                        zs.append(position[i][2])
                        ax.plot(xs, ys, zs, label='parametric curve %i' %Number_of_iteration[0], color = 'g')




    if(plot_dynibex):
        #Real Trajectory
        trajectory_boxes = []
        with open("trajectory_boxes.txt", "r") as file:
            for line in file:
                #Traitons une ligne de trois intervalles dans chaque direction
                line = line.replace( '(' , '' )
                line = line.replace( ')' , '' )
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
                trajectory_boxes.append([lower_bounds[0], lower_bounds[1], lower_bounds[2]])
                trajectory_boxes.append([lower_bounds[0]+widths[0], lower_bounds[1], lower_bounds[2]])
                trajectory_boxes.append([lower_bounds[0]+widths[0], lower_bounds[1]+widths[1], lower_bounds[2]])
                trajectory_boxes.append([lower_bounds[0], lower_bounds[1]+widths[1], lower_bounds[2]])
                trajectory_boxes.append([lower_bounds[0], lower_bounds[1], lower_bounds[2]+widths[2]])
                trajectory_boxes.append([lower_bounds[0]+widths[0], lower_bounds[1], lower_bounds[2]+widths[2]])
                trajectory_boxes.append([lower_bounds[0]+widths[0], lower_bounds[1]+widths[1], lower_bounds[2]+widths[2]])
                trajectory_boxes.append([lower_bounds[0], lower_bounds[1]+widths[1], lower_bounds[2]+widths[2]])

        #Ploting trajectory_boxes
        V=[]
        for i in range(len(trajectory_boxes)):
                V.append(trajectory_boxes[i]) #On lui donne les coordonnees de chaque sommet
                if(len(V)==8):                                                #Avec la position des 8 sommets il dessinne le polygone
                    faces = [[V[0],V[1],V[2],V[3]], 
                            [V[4],V[5],V[6],V[7]],
                            [V[0],V[1],V[5],V[4]],
                            [V[2],V[3],V[7],V[6]],
                            [V[1],V[2],V[6],V[5]],
                            [V[4],V[7],V[3],V[0]]]
                    if(i%(17)==0):
                        collection = ax.add_collection3d(Poly3DCollection(faces, facecolors = 'red' ,linewidths=0.1, edgecolors='orange', alpha=0.5))
                    V = []


        ##PLOTTING THE DYNIBEX SIMULATION
        vertices_of_the_3Dboxes = []

        with open("export_3d.txt", "r") as file:
            for line in file:
                #Traitons une ligne de trois intervalles dans chaque direction
                line = line.replace( '(' , '' )
                line = line.replace( ')' , '' )
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
                if(i%17 == 0):
                    collection = ax.add_collection3d(Poly3DCollection(faces, facecolors = 'cyan' ,linewidths=0.075, edgecolors='blue', alpha=0.03))
                # plt.pause(0.00000000000000000000001)
                Z = []
        


    plt.savefig('PLOT.png')
    plt.show()  


if __name__ == '__main__':
    main()
  
