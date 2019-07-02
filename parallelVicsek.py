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
from dronekit import connect, VehicleMode, LocationGlobalRelative
from datetime import datetime
######################################

### Imports from software modules
from etc.config import *
from lib.physics import get_distance_between_points, get_point
######################################

###############################################################################################################################
###############################################################################################################################

def compute_initial_location():

    """
    Computing initial location
    """
    print(rank_msg+' Computing initial location')
    global home_location
    home_location = vehicle.home_location.__dict__
    home_lat  =  home_location['lat']
    home_lon  =  home_location['lon']
    home_alt  =  home_location['alt']
    print(rank_msg+'Home location is lat: '+str(home_lat)+', lon: '+str(home_lon)+', alt: '+str(home_alt))
    splitted_angle=360/(size-1)
    return get_point(home_lat,home_lon,DRONE_ALT,(rank-1)*splitted_angle,SEPARATION_RADIUS)

def connect_to_vehicle():

    global vehicle
    try:
        print(rank_msg+' Connecting to autopilot')
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

def move(lat,lon,alt):

    """
    Moving to selected location
    """
    stage = 'init'
    print(rank_msg+' Moving to selected location, lat: '+str(lat)+', lon: '+str(lon)+', alt: '+str(alt))
    vehicle.simple_goto(LocationGlobalRelative(lat, lon, alt))
    while True:
        cP=vehicle.location.global_relative_frame.__dict__
        cla=cP['lat']; clo=cP['lon']; cal=cP['alt']
        writeTelemetryFile(cla,clo,cal,stage)
        if get_distance_between_points(cla,clo,lat,lon) < 1: break

def plot_trajectories_kml():

    """
    Plotting swarming trajectories via KML File
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

def startVehicle():

    """
    starting the vehicle
    """
    try:

        if not vehicle.armed:
            print(rank_msg+' Arming vehicle')
            while not vehicle.is_armable: sleep(1)
            vehicle.mode = VehicleMode('GUIDED')
            vehicle.armed = True
        if vehicle_type == 'drone':
            stage = 'take_off'
            print(rank_msg+' Taking off')
            vehicle.simple_takeoff(DRONE_ALT)
            while abs(vehicle.location.global_relative_frame.alt - DRONE_ALT) > 1:
                cP=vehicle.location.global_relative_frame.__dict__
                cla=cP['lat']; clo=cP['lon']; cal=cP['alt']
                writeTelemetryFile(cla,clo,cal,stage)
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
    stage = 'swarming'

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
        writeTelemetryFile(cla,clo,cal,stage)
        if step > INIT_STEPS:
            for r in range(1,size):
                if r != rank: comm.send([cla,clo,cal,heading], dest=r)
            heading=0.0
            neighbors=0
            for r in range(1,size):
                if r != rank:
                    eTel=comm.recv(source=r)
                    ecla= eTel[0]; eclo= eTel[1]; ecal=eTel[2]; ehea=eTel[3]
                    eDist=get_distance_between_points(cla,clo,ecla,eclo)
                    if eDist <= VICSEK_RADIUS:
                        neighbors+=1
                        #print(rank_msg+' Vehicle rank '+str(r)+' is a neighbor of mine')
                        heading+=ehea
            if neighbors > 0:
                print(rank_msg+' I have '+str(neighbors)+' neighbors')
                heading=heading/neighbors
        step+=1
        sleep(1)

def writeTelemetryFile(cla,clo,cal,stage):

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
    telemetryLine+=' '+stage
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
    global comm, rank, size, rank_msg
    global vehicle, vehicle_type, sim_time, telemetryFile, home_location, telemetry_folder
    ###############################################################################################################################

    ### Parallel process rank assignment
    comm = MPI.COMM_WORLD
    size = comm.Get_size()
    rank = comm.Get_rank()
    rank_msg = '[Rank '+str(rank)+' msg]'
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
        from multiprocessing import Process
        plottingProcess=Process(target=plot_trajectories_kml,args=())
        introduce_myself()
        print(rank_msg+' Starting swarm motion simulation based on simplified Vicsek model')
        print(rank_msg+' Waiting for vehicles\' status ...')
        veh_stats = []
        for r in range(1,size):
            vehicleReady=comm.recv(source=r)
            veh_stats.append(vehicleReady)
            if vehicleReady: print(rank_msg+' Vehicle rank '+str(r)+' is ready for simulation')
        if False in veh_stats:
            print('Vehicles starting had errors. Exiting simulation')
            for r in range(1,size): comm.send(False,dest=r)
            sys.exit(2)
        else:
            for r in range(1,size): comm.send(True,dest=r)
        sleep(3)
        plottingProcess.start()
        for r in range(1,size):
            rankDone=comm.recv(source=r)
            print('Rank '+str(r)+' has finished its vehicle simulation')
        plottingProcess.terminate()
        print(rank_msg+' Finishing simulation')
    ###############################################################################################################################

    ### Vehicles area
    else:

        telemetryFile=telemetry_folder+'/rank_'+str(rank)+'.xls'
        connect_to_vehicle()
        startVehicle()
        init_lat,init_lon,init_alt=compute_initial_location()
        move(init_lat,init_lon,init_alt)
        sleep(3)
        comm.send(True,dest=0)
        print(rank_msg+' Waiting for confirmation to start swarming')
        start_sim = comm.recv(source=0)
        print(rank_msg+' Ready for swarming')
        if start_sim:

            vicsek()
            print(rank_msg+' Going home')
            vehicle.mode = VehicleMode("RTL")
            home_lat  =  home_location['lat']
            home_lon  =  home_location['lon']
            home_alt  =  home_location['alt']
            print(rank_msg+' Going home')
            stage='going_home'
            while True:
                cP=vehicle.location.global_relative_frame.__dict__
                cla=cP['lat']; clo=cP['lon']; cal=cP['alt']
                writeTelemetryFile(cla,clo,cal,stage)
                if get_distance_between_points(cla,clo,home_lat,home_lon) < 1 and abs(cal-home_alt) <1: break
            print(rank_msg+' At home :D')
            comm.send(True,dest=0,tag=43)
        else:
            print(rank_msg+' Error in simulation starting')
            sys.exit(2)

    ###############################################################################################################################

if __name__ == "__main__":

    """
    The ARCHADE Parallel swarm motion
    """
    main()
