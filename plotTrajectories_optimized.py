#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
 MPI-based Swarm motion using simplified Vicsek model
 © Copyright, UbiHPC
 Developped by Leonardo Camargo Forero, UbiHPC CEO
 email: lecf.77@gmail.com
 2019
 This is a simplified version of multi-robot swarming motion based on Vicsek model.
 For a complete version based on The ARCHADE, contact UbiHPC at https://ubihpc.com/contact-us
"""

### imports ##########################
import os.path
import sys
import argparse
import subprocess
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from mpl_toolkits.mplot3d import Axes3D
######################################

### Imports from software modules
from etc.config import PLOT_SIZE, LAT_LINE, ALT_LINE, COLOR_MAP
######################################

###############################################################################################################################
###############################################################################################################################

def check_files():

    """
    Waiting for telemetry files availability
    """
    while True:
        files_exist = []
        for rank in range(1,q_of_vehicles+1):
            files_exist.append(os.path.exists(telemetry_folder+'/rank_'+str(rank)+'.xls'))

        if all(file==True for file in files_exist): break

def getCurrentPosition():

    """
    Getting current position of each vehicle
    """
    positions = {}
    for rank in range(1,q_of_vehicles+1):
        telemetry_file = telemetry_folder+'/rank_'+str(rank)+'.xls'
        positions[rank] = subprocess.check_output(['tail', '-1', telemetry_file]).split()[LAT_LINE:ALT_LINE+1]
    return positions

def plot_trajectories():

    """
    Plotting trajectories
    """
    fig = plt.figure(figsize=PLOT_SIZE)
    ax = fig.add_subplot(111,projection='3d')
    ax.set_xlabel('Latitude [°]')
    ax.set_ylabel('Longitude [°]')
    ax.set_zlabel('Altitude [m]')
    ax.set_yticklabels([])
    ax.set_xticklabels([])
    ax.set_zticklabels([])
    def update(i):
        cp = getCurrentPosition()
        print(cp)
        if cp:
            for vehicle in cp:
                ax.scatter(float(cp[vehicle][0]), float(cp[vehicle][1]), float(cp[vehicle][2]), c=color_mapping(vehicle-1), marker='o')
    a = anim.FuncAnimation(fig, update, repeat=False)
    plt.title('HPRC multi-vehicle motion according to Vicsek model')
    plt.show()

def set_trajectory_colors():

    """
    Setting trajectory colors
    """
    global color_mapping
    color_mapping = plt.cm.get_cmap(COLOR_MAP, q_of_vehicles)

###############################################################################################################################
###############################################################################################################################

def main():

    """
    The ARCHADE -- Parallel bioinspired Swarm motion based on Vicsek model -- trajectory plotter
    """
    global q_of_vehicles, telemetry_folder
    global trajectories, counter
    global color_mapping
    counter = -1
    parser = argparse.ArgumentParser(description='Multi-robot Vicsek plotting')
    parser.add_argument('-q','--q_of_vehicles',help='Quantity of vehicles')
    parser.add_argument('-t','--telemetry_folder',help='Folder to store telemetry files')
    arguments = parser.parse_args()
    q_of_vehicles = options.q_of_vehicles
    telemetry_folder = options.telemetry_folder
    if q_of_vehicles == None:
        print('Missing quantity of vehicles')
        sys.exit(0)
    else: q_of_vehicles = int(q_of_vehicles)
    if telemetry_folder == None:
        print('Missing telemetry folder')
        sys.exit(0)
    set_trajectory_colors()
    print('-------------------------------------------------')
    print('HPRC swaming motion trajectories plotter')
    print('-------------------------------------------------')
    print('Quantity of vehicles is: '+str(q_of_vehicles))
    print('Waiting for telemetry files')
    check_files()
    print('Telemetry files are ready')
    plot_trajectories()

if __name__ == "__main__":

    """
    The ARCHADE Parallel swarm motion -- trajectory plotter
    """
    main()
