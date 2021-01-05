#!/usr/bin/python

# Copyright (C) 2018-2021 LEIDOS.
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

import xml.etree.ElementTree as ET
import xml.dom.minidom
import json
import argparse
from math import *

def convertKMLToWaypoints(filename):
    """
    Convert the point data located in filename into a lane element
    """

    out = []
    xml = ET.parse(filename)
    root = xml.getroot()

    for coord in root.findall(".//{http://www.opengis.net/kml/2.2}coordinates"):
        (lon, lat, _) = coord.text.split(",")
	out.append((lat.strip(), lon.strip()))

    return out

def main():
    parser = argparse.ArgumentParser("Convert a KML file into a CSV file, to further process with RouteCreation_CSV2Yaml.py")
    parser.add_argument("kml_input", help="KML input containing the desired route geometry")
    parser.add_argument("default_speed", help="The default speed limit along the route")
    parser.add_argument("csv_output", help="The file to write the CSV out to")
    args = parser.parse_args()
    print("Converting {} to CSV as {}".format(args.kml_input, args.csv_output))
    output = open(args.csv_output, "w")
    output.write("Latitude,Longitude,Speed\n")

    waypoints = convertKMLToWaypoints(args.kml_input)
    print("{} waypoints found. Using speed {} for all. Writing output...".format(len(waypoints), args.default_speed))
    count = 0
    for waypoint in waypoints:
        output.write("{},{},{}\n".format(waypoint[0], waypoint[1], args.default_speed))
	count += 1
    print("Wrote {} entries to {}.".format(count, args.csv_output))


if __name__ == "__main__":
    print("Running RouteCreation_KML2CSV.py...")
    main()

