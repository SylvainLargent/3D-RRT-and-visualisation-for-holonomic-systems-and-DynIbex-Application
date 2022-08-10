from ast import Num
from ctypes import pointer
from decimal import MAX_EMAX
from click import command
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from numpy import double, true_divide
from matplotlib.animation import FuncAnimation
import  numpy as np
import sys




def main():
    np.set_printoptions(threshold=sys.maxsize)
    commandX = np.array([])
    commandY = np.array([])
    commandZ = np.array([])

    posX = np.array([])
    posY = np.array([])
    posZ = np.array([])

    list_sum_deltaX = np.array([])
    list_sum_deltaY = np.array([])
    list_sum_deltaZ = np.array([])

    objX = np.array([])
    objY = np.array([])
    objZ = np.array([])

    diff_command_realityX = np.array([])
    diff_command_realityY = np.array([])
    diff_command_realityZ = np.array([])

    sum_deltaX = 0
    sum_deltaY = 0
    sum_deltaZ = 0
    
    Kp = 1.8
    Ki = 3.2
    satruration = 0.8


    first_loop = True
    with open("csv_files/data3.txt", "r") as file:
        for line in file:
            list_of_info = line.strip().split(',')
            if(first_loop):
                first_loop = False
                index = double(list_of_info[9])
                take_off_index = double(list_of_info[9])

            elif(double(list_of_info[9]) != take_off_index):
                ##Recuperate the command
                commandX = np.append(commandX, double(list_of_info[0]))
                commandY = np.append(commandY, double(list_of_info[1]))
                commandZ = np.append(commandZ, double(list_of_info[2]))
                ##Recuperate the position
                posX = np.append(posX, double(list_of_info[3]))
                posY = np.append(posY, double(list_of_info[4]))
                posZ = np.append(posZ, double(list_of_info[5]))
                ##Recuperate objective
                objX = np.append(objX, double(list_of_info[6]))
                objY = np.append(objY, double(list_of_info[7]))
                objZ = np.append(objZ, double(list_of_info[8]))

                if(index == double(list_of_info[9])):
                    sum_deltaX += (objX[-1] - posX[-1])
                    sum_deltaY += (objY[-1] - posY[-1])
                    sum_deltaZ += (objZ[-1] - posZ[-1])
                    list_sum_deltaX = np.append(list_sum_deltaX, sum_deltaX)
                    list_sum_deltaY = np.append(list_sum_deltaY, sum_deltaY)
                    list_sum_deltaZ = np.append(list_sum_deltaZ, sum_deltaZ)
                else :
                    sum_deltaX = 0
                    sum_deltaY = 0
                    sum_deltaZ = 0
                    sum_deltaX += (objX[-1] - posX[-1])
                    sum_deltaY += (objY[-1] - posY[-1])
                    sum_deltaZ += (objZ[-1] - posZ[-1])
                    list_sum_deltaX = np.append(list_sum_deltaX, sum_deltaX)
                    list_sum_deltaY = np.append(list_sum_deltaY, sum_deltaY)
                    list_sum_deltaZ = np.append(list_sum_deltaZ, sum_deltaZ)
                    index = double(list_of_info[9])


    real_commandX = ( (Kp*(objX - posX) + Ki*list_sum_deltaX) > 0.8) * 0.8 + ( (Kp*(objX - posX) + Ki*list_sum_deltaX) < -0.8) * (-0.8) + ( Kp*(objX - posX) + Ki*list_sum_deltaX ) * ( abs(Kp*(objX - posX) + Ki*list_sum_deltaX)  < 0.8)
    real_commandY = ( (Kp*(objY - posY) + Ki*list_sum_deltaY) > 0.8) * 0.8 + ( (Kp*(objY - posY) + Ki*list_sum_deltaY) < -0.8) * (-0.8) + ( Kp*(objY - posY) + Ki*list_sum_deltaY ) * ( abs(Kp*(objY - posY) + Ki*list_sum_deltaY)  < 0.8)
    real_commandZ = ( (Kp*(objZ - posZ) + Ki*list_sum_deltaZ) > 0.8) * 0.8 + ( (Kp*(objZ - posZ) + Ki*list_sum_deltaZ) < -0.8) * (-0.8) + ( Kp*(objZ - posZ) + Ki*list_sum_deltaZ ) * ( abs(Kp*(objZ - posZ) + Ki*list_sum_deltaZ)  < 0.8)

    diff_command_realityX = commandX - real_commandX  
    diff_command_realityY = commandY - real_commandY 
    diff_command_realityZ = commandZ - real_commandZ 

    minX = min(diff_command_realityX)
    maxX = max(diff_command_realityX)
    minY = min(diff_command_realityY)
    maxY = max(diff_command_realityY)
    minZ = min(diff_command_realityZ)
    maxZ = max(diff_command_realityZ)

    #let's figure out the naccuracy interval
    # print(list_sum_deltaX)
    # print("Intervalle for max X ", maxX)
    # print("Intervalle for min X ", minX)
    # print("Intervalle for max Y ", maxY)
    # print("Intervalle for min Y ", minY)
    # print("Intervalle for max Z ", maxZ)
    # print("Intervalle for min Z ", minZ)

    print(maxX)
    print(minX)
    print(maxY)
    print(minY)
    print(maxZ)
    print(minZ)

if __name__ == '__main__':
    main()   



    