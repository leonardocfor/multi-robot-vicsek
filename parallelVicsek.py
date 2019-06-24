#!/usr/bin/env python
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

import sys
import optparse
from os import popen
from mpi4py import MPI
from time import sleep
from math import cos, sin, radians
from pymavlink import mavutil
from dronekit import connect, VehicleMode
from datetime import datetime
from multiprocessing import Process

######################################

### Imports from The ARCHADE modules
from etc.config import *
from lib.physics import getDistanceBetweenPoints, getPoint
######################################

###############################################################################################################################
###############################################################################################################################

def connect_to_vehicle():

    global vehicle
    try:
        print(rankMsg+' Connecting to autopilot')
        vehicle = connect('127.0.0.1:'+ARDUPILOT_PORT, wait_ready=True)
    except:
        print('Unable to connect to vehicle')
        comm.send(False,dest=0)
        sys.exit(2)

def introduce_myself():

    """
    Software introduction
    """
    pass

def plot_trajectories():

    """
    Plotting swarming trajectories
    """
    kml_trajectories_file = telemetry_folder+'/'+KML_TRAJECTORIES_FILE
    popen('touch '+kml_trajectories_file)
    while True:

        kml_tf = open(kml_trajectories_file,'w')
        for line in BEFORE_PLACEMARK_LINES:
            kml_tf.write(line+'\n')
        for r in range(1,size):
            current_rank_file = telemetry_folder+'/rank_'+str(r)+'.xls'
            for line in PLACEMARK_LINES:
                if 'vehicle_name' in line: line=line.replace('vehicle_name',vehicle_type+'_'+str(r))
                kml_tf.write(line+'\n')
                if '<coordinates>' in line:
                    for location_line in open(current_rank_file):
                        kml_tf.write(','.join(location_line.split()[LAT_LINE:ALT_LINE+1])+'\n')
        for line in AFTER_PLACEMARK_LINES:
            kml_tf.write(line+'\n')

def start_sitl():

    """
    Starting Ardupilot SITL
    """
    print(rankMsg+' Starting ArduPilot SITL')
    ardupilot_vehicle = 'ArduCopter' if vehicle_type == 'drone' else 'APMrover2'
    command = SITL_COMMAND+' '+ardupilot_vehicle+' -l '+LOCATION_FLAG
    print(command)
    popen(command)

def startVehicle():

    """
    starting the vehicle
    """
    try:
        if not vehicle.armed:
            print(rankMsg+' Arming vehicle')
            while not vehicle.is_armable: sleep(1)
            vehicle.mode = VehicleMode('GUIDED')
            vehicle.armed = True
        if vehicle_type == 'drone':
            print(rankMsg+' Taking off')
            vehicle.simple_takeoff(DRONE_ALT)
            while abs(vehicle.location.global_relative_frame.alt - DRONE_ALT) > 1: pass
    except:
        print('Unable to arm the vehicle')
        comm.send(False,dest=0)
        sys.exit(2)

def vicsek():

    """
    Simulation of simplified Vicsek model
    Noise excluded
    """
    heading=INITIAL_HEADING
    step=0

    for x in range(0,sim_time):

        vx=VEHICLES_SPEED*cos(radians(heading))
        vy=VEHICLES_SPEED*sin(radians(heading))
        vz=0
        velocity_msg = vehicle.message_factory.set_position_target_global_int_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111000111,
            0,
            0,
            0,
            vx,
            vy,
            vz,
            0, 0, 0,
            0, 0)
        vehicle.send_mavlink(velocity_msg)
        heading_msg = vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading,
            0,
            1,
            0,
            0, 0, 0)
        vehicle.send_mavlink(heading_msg)
        cP=vehicle.location.global_relative_frame.__dict__
        cla=cP['lat']; clo=cP['lon']; cal=cP['alt']
        writeTelemetryFile(cla,clo,cal)
        if step > INIT_STEPS:
            for r in range(1,size):
                if r != rank: comm.send([cla,clo,cal,heading], dest=r)
            heading=0.0
            neighbors=0
            for r in range(1,size):
                if r != rank:
                    eTel=comm.recv(source=r)
                    ecla= eTel[0]; eclo= eTel[1]; ecal=eTel[2]; ehea=eTel[3]
                    eDist=getDistanceBetweenPoints(cla,clo,ecla,eclo)
                    if eDist <= VICSEK_RADIUS:
                        neighbors+=1
                        print(rankMsg+' Vehicle rank '+str(r)+' is a neighbor of mine')
                        heading+=ehea
            if neighbors > 0: heading=heading/neighbors
        step+=1
        sleep(1)

def writeTelemetryFile(cla,clo,cal):

    tFile=open(telemetryFile,'a')
    msTime=datetime.now().strftime('%Y-%m-%d_%H:%M:%S.%f')
    velocity=vehicle.velocity
    vx=velocity[0]
    vy=velocity[1]
    vz=velocity[2]
    heading=vehicle.heading
    groundspeed=vehicle.groundspeed
    airspeed=vehicle.airspeed
    telemetryLine=str(msTime)+' '+str(cla)+' '+str(clo)+' '+str(cal) ## Time and location [lat,lon,alt]
    telemetryLine+=' '+str(groundspeed)+' '+str(airspeed)
    telemetryLine+=' '+str(vx)+' '+str(vy)+' '+str(vz)
    telemetryLine+=' '+str(heading)
    tFile.write(telemetryLine+'\n')
    tFile.close()

def usage():

    print('swarm.py -e <entities_file> -l <lat-lon-alt-heading> -a <ALTITUDE> -s <on> ')
    print('or')
    print('swarm.py --entities <entities_file> --location <location> --altitude <ALTITUDE> --startautopilot <on> ')

###############################################################################################################################optparse
###############################################################################################################################

def main():

    """
    The ARCHADE -- Parallel bioinspired Swarm motion based on Vicsek model
    """
    ### Global variables
    global comm, rank, size, rankMsg
    global vehicle, vehicle_type, sim_time, telemetryFile, telemetry_folder
    ###############################################################################################################################

    ### Parallel process rank assignment
    comm = MPI.COMM_WORLD
    size = comm.Get_size()
    rank = comm.Get_rank()
    rankMsg = '[Rank '+str(rank)+' msg]'
    ###############################################################################################################################

    ### Parameters reading
    parser = optparse.OptionParser()
    parser.add_option('-v','--vehicleType',dest='vehicle_type',help='Type of vehicle for swarming (drone, rover)')
    parser.add_option('-s','--simtime',dest='sim_time',help='Simulation time [s]')
    parser.add_option('-t','--telemetryFolder',dest='telemetry_folder',help='Folder to store telemetry files')
    (options,arguments) = parser.parse_args()
    vehicle_type = options.vehicle_type
    sim_time = options.sim_time
    telemetry_folder = options.telemetry_folder
    if telemetry_folder == None:
        if rank == 0: print('Missing telemetry folder. Exiting')
        sys.exit(0)
    if vehicle_type == None: vehicle_type = DEFAULT_VEHICLE_TYPE
    else:
        if rank == 0: print('Vehicle type: '+vehicle_type)
    if sim_time == None: sim_time = SIM_TIME
    else:
        if rank == 0: print('Simulation time: '+sim_time+' [s]')
    sim_time = int(sim_time)
    ###############################################################################################################################

    comm.Barrier()
    ### Ground station area
    if rank == 0:

        plotting_process=Process(target=plot_trajectories,args=())
    	introduce_myself()
        print(rankMsg+' Starting swarm motion simulation based on simplified Vicsek model')
        print(rankMsg+' Waiting for vehicles\' status ...')
        veh_stats = []
        for r in range(1,size):
            vehicleReady=comm.recv(source=r)
            veh_stats.append(vehicleReady)
            if vehicleReady: print(rankMsg+' Vehicle rank '+str(r)+' is ready for simulation')
        if False in veh_stats:
            print('Vehicles starting had errors. Exiting simulation')
            for r in range(1,size): comm.send(False,dest=r)
            sys.exit(2)
        else:
            for r in range(1,size): comm.send(True,dest=r)
        sleep(3)
        plotting_process.start()
        for r in range(1,size):
            rankDone=comm.recv(source=r)
            print('Rank '+str(r)+' has finished its vehicle simulation')
        plotting_process.terminate()
        print(rankMsg+' Finishing simulation')
    ###############################################################################################################################

    ### Vehicles area
    else:
        sitl_process = Process(target=start_sitl,args=())
        sitl_process.start()
        sleep(5)
        connect_to_vehicle()
        startVehicle()
        comm.send(True,dest=0)
        print(rankMsg+' Waiting for confirmation to start swarming')
        start_sim = comm.recv(source=0)
        print(rankMsg+' Ready for swarming')
        if start_sim:
            telemetryFile=telemetry_folder+'/rank_'+str(rank)+'.xls'
            popen('touch '+telemetryFile)
            vicsek()
            print(rankMsg+' Going home')
            vehicle.mode = VehicleMode("RTL")
            comm.send(True,dest=0,tag=43)
            sitl_process.terminate()
        else:
            print(rankMsg+' Error in simulation starting')
            sys.exit(2)

    ###############################################################################################################################

if __name__ == "__main__":

    """
    The ARCHADE Parallel swarm motion
    """
    main()
