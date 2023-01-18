from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random


if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  
import traci  


def generate_routefile():
    random.seed(42) 
    N = 4000  

    p1 = 1. / 15
    p2 = 1. / 15
    p3 = 1. / 15
    p4 = 1. / 30
    with open("data/tl.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="carA" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" \
guiShape="passenger"/>

        <route id="one" edges="-4588222#1 753316505#0 4588219#0" />
        <route id="two" edges="-109902441#1 753316506 753316505#0 168935756#0 292429056#0" />
        <route id="three" edges="-168935757#2 753316505#0 4588219#0" />
        <route id="four" edges="23755739#0 681299268 681299267#0 168935756#0 4588220#0" />""", file=routes)
        vehNr = 0
        for i in range(N):
            if random.uniform(0, 1) < p1:
                print('    <vehicle id="one_%i" type="carA" route="one" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p2:
                print('    <vehicle id="two_%i" type="carA" route="two" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p3:
                print('    <vehicle id="three_%i" type="carA" route="three" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p4:
                print('    <vehicle id="four_%i" type="carA" route="four" depart="%i" color="0,1,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
        print("</routes>", file=routes)


def run():
    ph0 = []
    ph1 = []
    l_h  = 4
    ph0_d = 25
    ph1_d = 25
    ph0_m = 0
    ph1_m = 0
    p_time = 0

    n_time = traci.trafficlight.getNextSwitch("25661966")
    l_d_time = n_time
    traci.simulation.step(n_time)
    traci.trafficlight.setPhase("25661966", 0)
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulation.step()
        if traci.trafficlight.getPhase("25661966") == 0:
            ph0.append(traci.lanearea.getJamLengthVehicle("e2Detector_0"))
            if len(ph0) > l_h :
                ph0.pop(0)
        else:
            ph1.append(traci.lanearea.getJamLengthVehicle("e2Detector_1"))
            if len(ph1) > l_h :
                ph1.pop(0)
        if l_d_time > 150 :
            if (min(ph0_d,ph1_d) > 5)&(max(ph0_d,ph1_d) < 45):
                p_time = n_time
                ph0_m = sum(ph0)/len(ph0)
                ph1_m = sum(ph1)/len(ph1)
                if ph0_m > ph1_m :
                    ph0_d += 2.5
                    ph1_d -= 2.5
                elif ph0_m < ph1_m :
                    ph0_d -= 2.5
                    ph1_d += 2.5
        if traci.trafficlight.getPhase("25661966") == 1:
            traci.trafficlight.setPhaseDuration("25661966",ph1_d)
        else:
            traci.trafficlight.setPhaseDuration("25661966",ph0_d)
        n_time = traci.trafficlight.getNextSwitch("25661966")
        traci.simulation.step(n_time)
        l_d_time = n_time - p_time
    traci.close()
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options



if __name__ == "__main__":
    options = get_options()

    
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    generate_routefile()

    
    traci.start([sumoBinary, "-c", "data/tl.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])
    
    run()
