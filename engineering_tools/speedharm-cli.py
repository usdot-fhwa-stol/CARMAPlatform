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


from __future__ import print_function
import json
import requests
import re
import urlparse

# Disable annoying insecure requests warnings, we're using a self-signed cert
from requests.packages.urllib3.exceptions import InsecureRequestWarning
requests.packages.urllib3.disable_warnings(InsecureRequestWarning)

version_id = "v0.1" # Version string for cli client
running = True # Running flag for program will execute main loop as long as true
base_url = "http://35.153.64.44:8081" # Base url of for AWS server, defaults to localhost.
#base_url = "http://localhost:8081" # Base url of VM localhost.
rel_url = ""

# Constants
vehicles_url = "/rest/vehicles"
experiments_url = "/rest/experiments"
algorithms_url = "/rest/algorithms"


def print_response(resp):
    try:
        print(resp.status_code)
        print(resp.headers)
        print(resp.json())
    except:
        print("Unable to decode the rest of the response.")

def unknown_command(cmd):
    print(cmd + " is not a recognized command. Try \"help\" for help.")

def process_set_command(parts):
    exec("global " + parts[1] + ";" + parts[1] + " = " + parts[2])

def process_get_command(parts):
    full_url = get_cur_url()
    if (len(parts) > 1):
        # A url has been specified
        full_url = urlparse.urljoin(full_url, parts[1])

    r = requests.get(full_url, verify=False)
    print_response(r)

def process_delete_command(parts):
    full_url = get_cur_url()
    if (len(parts) > 1):
        full_url = urlparse.urljoin(full_url, parts[1])

    ack = raw_input("Are you sure you want to delete " + full_url +"?\n(y/N): ")
    if ack.lower() == "y":
        r = requests.delete(full_url, verify=False)
        print_response(r)

def cd(new_url):
    global rel_url
    rel_url = urlparse.urljoin(rel_url, new_url)

def process_cd_command(parts):
    cd(parts[1])
    
def process_post_command(parts):
    data_start = 1
    full_url = get_cur_url()
    # if len(parts) > 2:
    #     data_start = 2
    #     full_url = urlparse.urljoin(full_url, parts[1])
    
    data = eval(" ".join(parts[data_start:]))

    headers = { "Content-Type" : "application/json" }

    r = requests.post(full_url, data=json.dumps(data), headers=headers, verify=False)
    print_response(r)

def process_create_vehicle():
    uniqVehId = raw_input("Unique Vehicle ID: ")
    description = raw_input("Vehicle Description: ")
    full_url = urlparse.urljoin(base_url, vehicles_url)
    headers = { "Content-Type" : "application/json" }
    r = requests.post(full_url, data=json.dumps({"uniqVehId": uniqVehId, "description": description}), headers=headers, verify=False)
    print_response(r)

def process_create_experiment():
    description = raw_input("Experiment Description: ")
    location = raw_input("Experiment Location: ")
    full_url = urlparse.urljoin(base_url, experiments_url)
    headers = { "Content-Type" : "application/json" }
    r = requests.post(full_url, data=json.dumps({"description": description, "location": location}), headers=headers, verify=False)
    print_response(r)
    
def process_create_algorithm():
    # Get the available algorithms
    r = requests.get(urlparse.urljoin(base_url, "/rest/"), verify=False)
    algos = r.json()["availableAlgorithms"]
    print("Select Algorithm: ")
    for a in zip(algos, range(len(algos))):
        print("{}. {}".format(a[1] + 1, a[0]))

    idx = input("Choice #? ") - 1

    full_url = urlparse.urljoin(base_url, algorithms_url)
    headers = { "Content-Type" : "application/json" }
    data = json.dumps({"className": algos[idx]})

    r = requests.post(full_url, data=data, headers=headers, verify=False)
    print_response(r)
    
def process_create_command(parts):
    if parts[1] == "vehicle":
        process_create_vehicle()
    elif parts[1] == "experiment":
        process_create_experiment()
    elif parts[1] == "algorithm":
        process_create_algorithm()
    else:
        print("Unknown entity to create: " + parts[1])

def select_vehicle():
    r = requests.get(urlparse.urljoin(base_url, vehicles_url), verify=False).json()

    print("Select Vehicle:")
    for veh in r:
        print("{}. Description: {}; Unique ID: {}".format(veh["id"], veh["description"], veh["uniqVehId"]))

    return raw_input("Vehicle ID#? ")

def process_assign_experiment():
    veh_id = select_vehicle()

    r = requests.get(urlparse.urljoin(base_url, experiments_url), verify=False).json()

    print("Select Experiment: ")
    for exp in r:
        print("{}. Description: {}; Location: {}".format(exp["id"], exp["description"], exp["location"]))
    exp_id = raw_input("Experiment ID#? ")

    headers = { "Content-Type" : "application/json" }
    data = json.dumps({"id": int(veh_id)})

    r = requests.post(urlparse.urljoin(base_url, experiments_url + "/" + exp_id + "/vehicles"), data=data, headers=headers, verify=False)
    print_response(r)

def process_assign_algorithm():
    veh_id = select_vehicle()
    
    r = requests.get(urlparse.urljoin(base_url, algorithms_url), verify=False).json()

    print("Select Algorithm: ")
    for exp in r:
        print("{}. Class: {}".format(exp["id"], exp["className"]))
    algo_id = raw_input("Algorithm ID#? ")

    headers = { "Content-Type" : "application/json" }
    data = json.dumps({"id": int(veh_id)})

    r = requests.post(urlparse.urljoin(base_url, algorithms_url + "/" + algo_id + "/vehicles"), data=data, headers=headers, verify=False)
    print_response(r)

def process_assign_command(parts):
    if parts[1] == "experiment":
        process_assign_experiment()
    elif parts[1] == "algorithm":
        process_assign_algorithm()
    else:
        print("Unknown relationship to assign: " + parts[1])


def process_list_vehicles():
    r = requests.get(urlparse.urljoin(base_url, vehicles_url), verify=False).json()

    print("Vehicles:")
    for veh in r:
        print("{}. Description: {}; Unique ID: {}".format(veh["id"], veh["description"], veh["uniqVehId"]))

def process_list_experiments():
    r = requests.get(urlparse.urljoin(base_url, experiments_url), verify=False).json()

    print("Experiments:")
    for veh in r:
        print("{}. Description: {}; Location: {}".format(veh["id"], veh["description"], veh["location"]))
    
def process_list_algorithms():
    r = requests.get(urlparse.urljoin(base_url, algorithms_url), verify=False).json()

    print("Algorithms:")
    for veh in r:
        print("{}. ClassName: {}".format(veh["id"], veh["className"]))

def process_list_command(parts):
    if parts[1] == "vehicles":
        process_list_vehicles()
    elif parts[1] == "experiments":
        process_list_experiments()
    elif parts[1] == "algorithms":
        process_list_algorithms()
    else:
        print("Unknown entity to list: " + parts[1])

def process_fetch_command(parts):
    if parts[1] == "vehicle":
        full_url = urlparse.urljoin(base_url, vehicles_url)
    elif parts[1] == "experiment":
        full_url = urlparse.urljoin(base_url, experiments_url)
    elif parts[1] == "algorithm":
        full_url = urlparse.urljoin(base_url, algorithms_url)
    else:
        print("Unknown entity to fetch: " + parts[1])
        return

    r = requests.get(full_url + "/" + parts[2], verify=False)
    print_response(r)
    
def parse_input(cmd):
    """
    Parse the command input and execute the appropriate action
    """

    parts = cmd.split(' ') # Split on spaces

    args = {"set" : "variable value", "get" : "[url]", "cd" : "rel_url",
            "post" : "data", "delete" : "[url]", "create" : "(vehicle | experiment | algorithm)",
            "assign" : "(experiment | algorithm)", "list" : "(vehicle | experiment | algorithm)",
            "fetch" : "(vehicle | experiment | algorithm) id", "help": "", "exit" : ""}
    
    try:
        if parts[0] == "exit":
            global running
            running = False
            exit(0)
        elif parts[0] == "set":
            process_set_command(parts)
        elif parts[0] == "get":
            process_get_command(parts)
        elif parts[0] == "cd":
            process_cd_command(parts)
        elif parts[0] == "post":
            process_post_command(parts)
        elif parts[0] == "delete":
            process_delete_command(parts)
        elif parts[0] == "create":
            process_create_command(parts)
        elif parts[0] == "assign":
            process_assign_command(parts)
        elif parts[0] == "list":
            process_list_command(parts)
        elif parts[0] == "fetch":
            process_fetch_command(parts)
        elif parts[0] == "help":
            print("Commands: ")
            for cmd, usage in args.items():
                print("{} {}".format(cmd, usage))
        else:
            unknown_command(cmd)
    except IndexError:
        print("Incorrect usage of: " + parts[0])
        print("Usage: " + parts[0] + " " + args[parts[0]])
    except requests.exceptions.ConnectionError:
        print("Unable to connect to " + base_url + ".")

def get_cur_url():
    return urlparse.urljoin(base_url, rel_url)

def main():
    print("Speed Harmonization Command Line Configurator " + version_id + ".")
    while running:
        parse_input(raw_input(get_cur_url() + ">> "))

if __name__ == "__main__":
    main()

