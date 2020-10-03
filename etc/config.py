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
ARDUPILOT_PORT='14550'
HEADING='0'
TELEMETRY_FOLDER='/TAC/data'
VEHICLES_SPEED=10
INITIAL_HEADING=90
VICSEK_RADIUS=3
SIM_TIME=60
INIT_STEPS=5
DRONE_ALT=30
DEFAULT_VEHICLE_TYPE = 'drone'
LAT_LINE = 1
ALT_LINE = 3
SEPARATION_RADIUS = 1
PLOT_SIZE = (10,8)
COLOR_MAP = 'hsv'
