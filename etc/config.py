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
VICSEK_RADIUS=5
SIM_TIME=60
INIT_STEPS=5
DRONE_ALT=30
DEFAULT_VEHICLE_TYPE = 'drone'
KML_TRAJECTORIES_FILE='swarming.kml'
BEFORE_PLACEMARK_LINES = [
'<?xml version="1.0" encoding="UTF-8"?>',
'<kml xmlns="http://www.opengis.net/kml/2.2">',
'   <Document>',
'       <name>HPRC-Vicsek</name>',
'       <description>HPRC cluster flying as a bird\'s swarm based on simplified Vicsek model</description>',
'       <Style id="blueLineGreenPoly">',
'           <LineStyle>',
'               <color>7dff0000</color>',
'               <width>256</width>',
'               <gx:physicalWidth>0.1</gx:physicalWidth>',
'           </LineStyle>',
'           <PolyStyle>',
'               <color>7f00ff00</color>',
'           </PolyStyle>',
'       </Style>',
# '       <Style id="greenLineGreenPoly">',
# '           <LineStyle>',
# '               <color>7f00ff00</color>',
# '               <width>1024</width>',
# '           </LineStyle>',
# '           <PolyStyle>',
# '               <color>7f00ff00</color>',
# '           </PolyStyle>',
# '       </Style>',
]
PLACEMARK_LINES = [
'       <Placemark>',
'           <name>vehicle_name</name>',
'           <description>Vehicle moving as part of  a bird\'s swarm</description>',
'           <styleUrl>#blueLineGreenPoly</styleUrl>',
'           <LineString>',
'               <extrude>1</extrude>',
'               <tessellate>1</tessellate>',
'               <altitudeMode>relative</altitudeMode>',
'               <coordinates>',
'               </coordinates>',
'           </LineString>',
'       </Placemark>'
]
AFTER_PLACEMARK_LINES = [
'   </Document>'
'</kml>'
]
LAT_LINE = 1
ALT_LINE = 3
SEPARATION_RADIUS = 2
