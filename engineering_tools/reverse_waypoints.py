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

# This script can be used to reverse all waypoints order from a CARMA route file
# To Run:
# Create a text file and only copy and paste lines after 'waypoints:'
# Add an empty line at the start of the file
# Run "python reverse_waypoints.py <file_name>" to get a text file with reversed points

import sys

def main():
    filepath = sys.argv[1]
    waypoints = []
    with open(filepath, 'r') as fp:
        line = fp.readline()
        counter = 0
        while line:
            if len(line) == 1:
                counter += 1
                waypoint = []
                waypoint.append(line)
                waypoints.append(waypoint)
            else:
                waypoints[counter - 1].append(line)
            line = fp.readline()
        print("Processed %d waypoints" % len(waypoints))
    output_path = filepath + '_output'
    with open(output_path, 'w') as fp:
        for wp in reversed(waypoints):
            for line in wp:
                fp.write(line)
    print("Done!")

if __name__ == "__main__":
    print("Running ReverseWaypoints.py...")
    main()
