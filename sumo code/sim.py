from mpro import *
import traci
import random
import os
import sys
import optparse
from sumolib import checkBinary
import socket

class Simulation:
    def __init__(self, network: str, demand: str, detectors: str, output: str, seed: int = None) -> None:
        
        self.control = True
        self.test = False
        
        self.network = sumolib.net.readNet(network)
        self.links = self.network.getEdges()
        self.demand = demand
        self.output = output
        self.seed = seed if seed is not None else random.randint(0, 9999)
        
        self.detectors = []
        with open(directory+"/detectors.txt") as file:
            self.detectors = [line.rstrip() for line in file]
    
        print("detectors: ", detectors)

        self.rampnodes = []
        self.Es, self.Er, self.Eh, self.Er_merge = set(), set(), set(), set()
        self.prepare_simulation()
        
        
        self.cc = {d : 0 for d in detectors}
        self.passed = {d : 0 for d in detectors}
    
        self.detectors_abbrv = dict()
        self.vehIds = dict()
        #counts = dict()
        #occupancies = dict()
        
        for det in self.detectors:
            self.vehIds[det] = set()
            #counts[det] = 0
            #occupancies[det] = 0
            # necessary due to limitations of PSQL database
            self.detectors_abbrv[det] = det.replace("Demand", "D").replace("Down", "Do").replace("Pass", "P").replace("Green", "G").replace("Merge", "M").replace("det", "d")

        sumoBinary = checkBinary('sumo')
        self.commands = [
            sumoBinary,
            "-n", network, 
            "-r", demand, 
            "--time-to-teleport", "-1", 
            "--step-length", "0.05", 
            "--tripinfo-output", self._get_seeded_output(),
            "--seed", str(self.seed),
            "--random"
        ]

    def _get_seeded_output(self):
        # Modify output filename to include seed information
        base, ext = os.path.splitext(self.output)
        return f"{base}_seed{self.seed}{ext}"

    def prepare_simulation(self):
        for link in self.links:
            linkstr = link.getID()

            if "ramp" in linkstr and "start" in linkstr.lower():
                self.Er.add(link)
                self.rampnodes.append(link.getToNode())
            elif "ramp" in linkstr and "end" in linkstr.lower():
                self.Er_merge.add(link)
            else:
                self.Eh.add(link)
        
        self.E_entry = self.Er.union({self.network.getEdge("entry")})

    def run(self, T: int = 7200) -> None:
        print(f"Starting simulation with seed {self.seed}")
        self.connect()
        
        
        
        traci.start(self.commands)

        t = 0
        CYCLE = 30
        MPRO_REFRESH = 30

        green_number = {r.getID(): 0 for r in self.rampnodes}
        green_number[self.rampnodes[0].getID()] = 1
        red_number = {r.getID(): 0 for r in self.rampnodes}
        rampnode_redTimes={r.getID(): 0 for r in self.rampnodes}

        while t < 7200:

            
            if t % MPRO_REFRESH == 0:
                for rampnode in self.rampnodes:
                    r = rampnode.getID()
                    rampnode_redTimes[r]=0
                    ramp_rate = optimal_control(rampnode)
                    
                    green_times_per_cycle = int(np.ceil(ramp_rate * CYCLE/3))
                    red_duration_per_cycle = round((CYCLE - green_times_per_cycle * 3) / max(green_times_per_cycle, 1),1)
                    rampnode_redTimes[r] = red_duration_per_cycle

            for rampnode in self.rampnodes:
                r = rampnode.getID()
                if green_number[r] == 3:
                    if red_number[r] < rampnode_redTimes[r]:
                        traci.trafficlight.setPhase(r, 1)
                        red_number[r] = round(red_number[r] + 0.1, 1) 
                    if red_number[r] == rampnode_redTimes[r]:
                        green_number[r] = 0
                        red_number[r] = 0
                else:
                    traci.trafficlight.setPhase(r, 0)
                    green_number[r] = round(green_number[r] + 0.1, 1)
                print(f"t={t:.1f}, rampnode={r}, green_number={green_number[r]}, red_number={red_number[r]}, red_duration_per_cycle={rampnode_redTimes[r]}")        
            t = round(t + 0.1, 1)
            traci.simulationStep(t)

        traci.close()
            
    def connect(self):
        self.connection = None
        
        if self.control and not self.test:
            
            host = "localhost"
            port = 5452
        
            print("connecting to IRIS server ", host, port);
            
            while(True):
                try:
                    self.connection = socket.socket()
                    self.connection.connect((host, port))
                    break
                except ConnectionRefusedError:
                    print("Waiting for IRIS server")
                    time.sleep(1)
                    
            print("connected")
        else:
            print("test only, no server conection")