#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
 MPI-based Swarm motion based on simplified Vicsek model
 © Copyright, UbiHPC
 Developped by Leonardo Camargo Forero, UbiHPC CEO
 email: lecf.77@gmail.com
 2019
 This is a simplified version of multi-robot swarming motion based on Vicsek model.
 For a complete version based on The ARCHADE, contact UbiHPC at https://ubihpc.com/contact-us
"""


from geographiclib.geodesic import Geodesic


def getDistanceBetweenPoints(lat1,lon1,lat2,lon2):

 	"""
	Returns the distance in meters between two points (lat1,lon1) and (lat2,lon2)
 	"""
 	return Geodesic.WGS84.Inverse(lat1,lon1, lat2, lon2)['s12']

def getPoint(lat,lon,alt,deg,dist):

	"""
	Returns the latitude and longitude of a point at a distance dist with a degree deg from lat,lon at the same alt
	"""
	return [Geodesic.WGS84.Direct(lat,lon,deg,dist)['lat2'],Geodesic.WGS84.Direct(lat,lon,deg,dist)['lon2'],alt]
