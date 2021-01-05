#!/usr/bin/env python

# Copyright (C) 2017-2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.


# This script can be used to convert csv files into route files for use in the CARMA platform
# Currently only lat,lon, and speed limit are supported fields
# To Run:
# python routeFileFromCSVScript.py <csv_file> <yaml_file> <route_name>
# Example:
# c:\Python27\python routeFileFromCSVScript.py ATC_city_1_20m_waypoints.csv exampleRoute.yaml ExampleRoute
import csv
import sys

# Function generates a yaml block representing a waypoint with the specified lat,lon, and speedlimit
def waypointAsYAMLString(lat, lon, speedLimit):
  return (
  	'  - !gov.dot.fhwa.saxton.carma.route.RouteWaypoint\n'
    '    location: !gov.dot.fhwa.saxton.carma.geometry.geodesic.Location\n'
      '      latitude: ' + str(lat) + '\n'
      '      longitude: ' + str(lon) + '\n'
      '      altitude: 0.0\n'
    '    minCrossTrack: -10.0\n'
    '    maxCrossTrack: 10.0\n'
    '    lowerSpeedLimit: 0\n'
    '    upperSpeedLimit: ' + str(speedLimit) + '\n'
    '    laneCount: 1\n'
    '    interiorLaneMarkings: SOLID_WHITE\n'
    '    leftMostLaneMarking: SOLID_YELLOW\n'
    '    rightMostLaneMarking: SOLID_WHITE\n'
    '\n'
    )

# Main function which converts the provided csv file to the specified route file
def convertCSVToRouteFile(csvPath, routeFilePath, routeName):
	print('Converting csv file to route file...')
	with open(csvPath, 'rb') as csvfile: # Open csv file
		with open(routeFilePath, 'wb') as yamlfile: # Open yaml file
			waypoint_reader = csv.reader(csvfile, delimiter=',')
			# Write header
			yamlfile.write('!gov.dot.fhwa.saxton.carma.route.Route' + '\n')
			yamlfile.write('routeName: ' + routeName + '\n')
			yamlfile.write('maxJoinDistance: ' + '40.0' + '\n')
			yamlfile.write('waypoints:\n')
			latIndex = 2
			lonIndex = 3
			speedLimitIndex = 4
			firstLine = True
			for row in waypoint_reader:
				if (firstLine): # Find needed column index and skip header row
					index = 0
					for col_name in row:
						if (col_name == 'Latitude'):
							latIndex = index
						elif (col_name == 'Longitude'):
							lonIndex = index
						elif (col_name == 'Speed'):
							speedLimitIndex = index
						index += 1
					firstLine = False
					continue
				lat = float(row[latIndex])
				lon = float(row[lonIndex])
				speedLimit = int(row[speedLimitIndex])
				# Write waypoint to file
				yamlfile.write(waypointAsYAMLString(lat,lon,speedLimit))

	print('Done converting csv file to route file')

# Main execution
if __name__ == "__main__":
    
    # Check if all arguments are provided
    if (len(sys.argv) <= 3):
    	print('Please rerun with command line arguments <input csv file path> <output route yaml file path> <route name>')
    else: # Run the converter
    	convertCSVToRouteFile(sys.argv[1], sys.argv[2], sys.argv[3])
