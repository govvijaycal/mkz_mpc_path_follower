#!/usr/bin/env python
import utm
import math

def parse_sentence(sent_str):
	spl = sent_str.split()

	lat = float(spl[3])
	lon = float(spl[4])
	alt = float(spl[5])

	lat_sd = float(spl[8])
	lon_sd = float(spl[9])
	alt_sd = float(spl[10])

	return lat, lon, alt, lat_sd, lon_sd, alt_sd


def convert_latlon_to_xy_spherical(o_lat, o_lon, LAT0, LON0):
	# Adapted from Stochastic LC, MPC Lab (Greg Marcil)
	R = 6371000  # radius of Earth in meters
	delta_lat = math.radians(o_lat[0] - LAT0)
	delta_lon = math.radians(o_lon[0] - LON0)

    # Convert GPS to meters
    x[0] = R * delta_lon * math.cos(0.5*(math.radians(lat1) + math.radians(lat0)))
	y[0] = R * delta_lat

	x[1] = o_lat[1]
	y[1] = o_lon[1]

	return x,y

def convert_latlon_to_xy_utm(o_lat, o_lon, LAT0, LON0):
	lat = o_lat[0]
	lon = o_lon[0]

	easting,northing, _, _ = utm.from_latlon(o_lat[0], o_lon[0])
	e_0, n_0, _, _ = utm.from_latlon(LAT0, LON0)

	x[0] = easting - e_0
	y[0] = northing - n_0

	x[1] = o_lat[1]
	y[1] = o_lon[1]

	return x,y