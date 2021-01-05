#!/usr/bin/env python

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


from __future__ import print_function
import json
import requests
import re
import urlparse
from datetime import datetime
import time
import calendar

# Disable annoying insecure requests warnings, we're using a self-signed cert
from requests.packages.urllib3.exceptions import InsecureRequestWarning
requests.packages.urllib3.disable_warnings(InsecureRequestWarning)

version_id = "v0.1" # Version string for cli client
base_url = "http://35.153.64.44:8081" # Base url of server, defaults to CARMA AWS server

# Constants
vehicles_url = "/rest/vehicles"
experiments_url = "/rest/experiments"
algorithms_url = "/rest/algorithms"
default_algorithm = "gov.dot.fhwa.saxton.speedharm.executive.algorithms.ReduceSpeedAlgorithm"
sleep_duration_millis = 1000

def log(msg):
    cur_datetime = datetime.utcnow()
    unix_ts = calendar.timegm(cur_datetime.utctimetuple())
    print("[" + str(unix_ts) + "] " + str(msg))

def print_response(resp):
    log(resp.status_code)
    log(resp.headers)
    log(resp.json())

def get_active_experiment_url():
    r = requests.get(urlparse.urljoin(base_url, experiments_url), verify=False).json()
    for exp in r:
        if exp["description"].find("AUTO-CONFIG") > -1:
            return urlparse.urljoin(base_url, experiments_url + "/" + exp["id"])
    
    return None

def create_experiment(desc, loc):
    full_url = urlparse.urljoin(base_url, experiments_url)
    headers = { "Content-Type" : "application/json" }
    r = requests.post(full_url, data=json.dumps({"description": desc, "location": loc}), headers=headers, verify=False)
    print_response(r)
    return r.headers["Location"]

def assign_experiment(veh_id, experiment_url):
    headers = { "Content-Type" : "application/json" }
    data = json.dumps({"id": int(veh_id)})
    r = requests.post(urlparse.urljoin(experiment_url + "/vehicles", ""), data=data, headers=headers, verify=False)

def get_registered_veh_data():
    r = requests.get(urlparse.urljoin(base_url, vehicles_url), verify=False)
    print_response(r)
    return r.json()

def create_algorithm(algo_name):
    full_url = urlparse.urljoin(base_url, algorithms_url)
    headers = { "Content-Type" : "application/json" }
    data = json.dumps({"className": algo_name})

    r = requests.post(full_url, data=data, headers=headers, verify=False)
    print_response(r)
    return r.headers["Location"]

def assign_algorithm(veh_id, algo_url):
    headers = { "Content-Type" : "application/json" }
    data = json.dumps({"id": int(veh_id)})
    r = requests.post(urlparse.urljoin(algo_url + "/vehicles", ""), data=data, headers=headers, verify=False)
    print_response(r)

def main():
    log("Speed Harmonization Auto Configurator " + version_id + ".")
    log("Connecting to " + base_url + "...")
    while True:
        for veh in get_registered_veh_data():
            if not "expId" in veh:
                # Vehicle isn't yet assigned to an experiment, assume it's also not assigned to an algorithm
                log("Detected new vehicle ID=" + str(veh["id"]) + " with no experiment, assigning to experiment.")
                active_experiment_url = get_active_experiment_url()
                if not active_experiment_url:
                    cur_datetime = datetime.utcnow()
                    unix_ts = calendar.timegm(cur_datetime.utctimetuple())
                    active_experiment_url = create_experiment("AUTO-CONFIG Experiment Generated @ " + str(unix_ts), "UNSPECIFIED")
                    log("Created new experiment at " + active_experiment_url)
                else:
                    log("Discovered active experiment at " + active_experiment_url)
                assign_experiment(veh["id"], active_experiment_url)
                log("Assigned vehicle ID=" + str(veh["id"]) + " to experiment " + active_experiment_url)
                # Vehicle isn't yet assigned to an algorithm
                log("Detected new vehicle ID=" + str(veh["id"]) + " with no algorithm, assigning to algorithm.")
                algo_url = create_algorithm(default_algorithm)
                log("Created new algorithm of type " + default_algorithm + " at " + algo_url)
                assign_algorithm(veh["id"], algo_url)
                log("Assigned vehicle ID=" + str(veh["id"]) + " to algorithm " + algo_url)
        log("Sleeping " + str(sleep_duration_millis) + "ms...")
        time.sleep(sleep_duration_millis / 1000.0)

if __name__ == "__main__":
    main()
