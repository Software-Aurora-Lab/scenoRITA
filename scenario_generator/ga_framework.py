import random
import subprocess
from deap import base
from deap import creator
from deap import tools
from map_info_parser import *
from result_printer import *
import os
import time
import json

obs_folder = "/apollo/modules/tools/perception/obstacles/"

#Return a mapping from x,y coor to laneIds and viceVersa (info extracted from routing_map of sunnyvale_loop)
ptl_dict, ltp_dict, diGraph=initialize()
obstacle_type=["VEHICLE","PEDESTRIAN","BICYCLE"]

def check_trajectory(p_index1,p_index2):
    valid_path=False
    while not valid_path:
        valid_path=validatePath(p_index1,p_index2,ptl_dict,ltp_dict,diGraph)
        if not valid_path:
            p_index1=random.randint(0,len(ptl_dict.keys()))
            p_index2=random.randint(0,len(ptl_dict.keys()))
    return p_index1,p_index2

def check_obs_type(length,width,height,speed,type_index):
    obs_type=obstacle_type[type_index]
    if obs_type is "VEHICLE":
        if length < 4.0 or length > 14.5:
            length=random.uniform(4.0,14.5)
        if height < 1.5 or height > 4.7:
            height=random.uniform(1.5,4.7)
        if width < 1.5 or width > 2.5:
            width=random.uniform(1.5,2.5)
        if speed < 2.5 or speed > 35:
            speed=random.uniform(2.5,35)
        diversity_counter["V"]+=1
    if obs_type is "PEDESTRIAN":
        if length < 0.2 or length > 0.5:
            length=random.uniform(0.2,0.5)
        if height <0.97 or height > 1.87:
            height=random.uniform(0.97,1.87)
        if width < 0.3 or width > 0.8:
            width=random.uniform(0.3,0.8)
        if speed < 1.25 or speed > 3:
            speed=random.uniform(1.25,3)
        diversity_counter["P"]+=1
    if obs_type is "BICYCLE":
        if length < 1 or length > 2.5:
            length=random.uniform(1,2.5)
        if height < 1 or height > 2.5:
            height=random.uniform(1,2.5)
        if width < 0.5 or width > 1:
            width=random.uniform(0.5,1)
        if speed < 1.75 or speed > 8.5:
            speed=random.uniform(1.75,8.5)
        diversity_counter["B"]+=1
    return length,width,height,speed

def runScenario(deme,scenario_counter,record_name):
    #to start with a fresh set of obstacles for the current scnerio
    os.system("rm -f /apollo/modules/tools/perception/obstacles/*")
    global diversity_counter
    diversity_counter={"V":0,"P":0,"B":0}

    for ind in deme:
        p_index1,p_index2=check_trajectory(ind[1],ind[2])
        ind[1]=p_index1
        ind[2]=p_index2
        #get the x,y coor coorespoding to index i1 and i2
        p1=list(ptl_dict.keys())[p_index1]
        p2=list(ptl_dict.keys())[p_index2]
        #get the correspodning lane id where the points reside
        lane1=ptl_dict[p1]
        lane2=ptl_dict[p2]
        #find the path between two lanes
        path=nx.shortest_path(diGraph, source=lane1, target=lane2)

        #verify that obstacle type and size is realistic
        ind[4],ind[5],ind[6],ind[7]=check_obs_type(ind[4],ind[5],ind[6],ind[7],ind[8])

        #ensure there are no two obstacles with similar id
        unique_obs_id=False
        while not unique_obs_id:
            if os.path.exists(os.path.join(obs_folder,"sunnyvale_loop_obs{}.json".format(ind[0]))):
                ind[0]=random.randint(0,30000)
            else:
                unique_obs_id=True

        #generate the desc files (each desc file corresponds to one individual/obstacle)
        desc=generateObsDescFile(ind[0],ind[3],ind[4],ind[5],ind[6],ind[7],obstacle_type[ind[8]])
        desc=produceTrace(p1,p2,path,ltp_dict,desc)
        filename="sunnyvale_loop_obs"+str(desc["id"])+".json"
        with open(os.path.join(obs_folder, filename), 'w') as outfile:
            json.dump(desc,outfile)

    # ------- sending valid adc routing -------
    sortedInd=tools.selBest(deme,len(deme))
    valid_path=False
    for ind in sortedInd:
        adc_loc_tuple=(ind[9],ind[10])
        if not (adc_loc_tuple in adc_loc_decider[scenario_counter]):
            valid_path=validatePath(ind[9],ind[10],ptl_dict,ltp_dict,diGraph)
        if valid_path:
            p_index1=ind[9]
            p_index2=ind[10]
            break
    #if none of the obstacles in the current scenario have a valid adc loc, generate a new random adc loc
    while not valid_path:
        p_index1=random.randint(0,len(ptl_dict.keys()))
        p_index2=random.randint(0,len(ptl_dict.keys()))
        adc_loc_tuple=(p_index1,p_index2)
        if not (adc_loc_tuple in adc_loc_decider[scenario_counter]):
            valid_path=validatePath(p_index1,p_index2,ptl_dict,ltp_dict,diGraph)
    p1=list(ptl_dict.keys())[p_index1]
    p2=list(ptl_dict.keys())[p_index2]
    adc_routing=p1.replace('-',',')+","+p2.replace('-',',')
    # ------- end of sending valid adc routing -------

    # run the scneario
    scenario_player_cmd='bazel run --experimental_ui_limit_console_output=1 //automation:run_automation -- -rv '+adc_routing+' -o '+record_name
    scenario_player_output = subprocess.check_output(scenario_player_cmd, shell=True)
    scenario_player_output=str(scenario_player_output)[2:-3]
    scenario_player_output=scenario_player_output.split('\\n')

    min_distance=json.loads(scenario_player_output[0].replace("'","\""))
    lanes=scenario_player_output[1].split(" ")
    min_speed=scenario_player_output[2]
    offroad=scenario_player_output[3]
    accl=scenario_player_output[4]
    hardbreak=scenario_player_output[5]
    speed_limit=scenario_player_output[6]

    return min_distance,set(lanes),float(min_speed),p_index1,p_index2,adc_routing,float(offroad),float(accl),float(hardbreak),speed_limit

# ------- Main Function -------
def main():
    # ------- GA Definitions -------
    # Fitness and Individual generator 
    creator.create("MultiFitness", base.Fitness, weights=(-1.0,-1.0,1.0,-0.1,1.0,-1.0))
    creator.create("Individual", list, fitness=creator.MultiFitness)
    toolbox = base.Toolbox()
    # Attribute generator (9 obstacle attributes + 2 ADC loc attributes)
    toolbox.register("id", random.randint, 0, 30000)
    toolbox.register("start_pos", random.randint, 0, len(ptl_dict.keys()))
    toolbox.register("end_pos", random.randint, 0, len(ptl_dict.keys()))
    toolbox.register("theta", random.uniform, -3.14, 3.14)
    toolbox.register("length", random.uniform, 0.2, 14.5)
    toolbox.register("width", random.uniform, 0.3, 2.5)
    toolbox.register("height", random.uniform, 0.97,4.7)
    toolbox.register("speed", random.uniform, 1, 35)
    toolbox.register("type", random.randint, 0, 2)
    toolbox.register("adc_start", random.randint, 0, len(ptl_dict.keys()))
    toolbox.register("adc_end", random.randint, 0, len(ptl_dict.keys()))
    # Structure initializers
    toolbox.register("individual", tools.initCycle, creator.Individual,
            (toolbox.id, toolbox.start_pos, toolbox.end_pos ,toolbox.theta, toolbox.length, toolbox.width, toolbox.height, toolbox.speed, toolbox.type,toolbox.adc_start,toolbox.adc_end), n=1)
    # define the deme to be a list of individuals (obstacles)
    toolbox.register("deme", tools.initRepeat, list, toolbox.individual)
    toolbox.register("mate", tools.cxTwoPoint)
    toolbox.register("mutate", tools.mutUniformInt, low=(0,0,0,-3,0,0,1,1,0,0,0), up=(30000,442106,442106,3,15,3,5,35,2,442106,442106), indpb=0.05)
    toolbox.register("select", tools.selNSGA2)

    random.seed(64)
    NGEN=40
    NP=10
    OBS_MAX=50
    OBS_MIN=1
    TOTAL_LANES=3061
    GLOBAL_LANE_COVERAGE=set()
    DEME_SIZES = [random.randint(OBS_MIN,OBS_MAX) for p in range(0,NP)]
    CXPB, MUTPB, ADDPB, DELPB = 0.5, 0.2, 0.05, 0.05
    
    global adc_loc_decider
    pop = [toolbox.deme(n=i) for i in DEME_SIZES]
    hof = tools.HallOfFame(NP) #best ind in each scenario
    lane_coverage = {scenario_num:set() for scenario_num in range(1,NP+1)} 
    adc_loc_decider = {scenario_num:set() for scenario_num in range(1,NP+1)}
    scenario_counter=1  
    g=0
    
    os.system("rm -rf /apollo/automation/oracles/Safety_Violations/*")

    print("Start of evolution")
    start_time=time.time()

    for deme in pop:
        record_name="Generation{}_Scenario{}".format(g,scenario_counter)
        min_distance,lanes,min_speed,p_index1,p_index2,adc_routing,offroad,accl,hardbrake,speed_limits=runScenario(deme,scenario_counter,record_name) 
        lanes.remove('')
        GLOBAL_LANE_COVERAGE.update(lanes)
        lane_coverage[scenario_counter]=lane_coverage[scenario_counter].union(lanes)
        adc_loc_decider[scenario_counter].add((p_index1,p_index2))
        for ind in deme:
            #assign the new adc loc
            ind[9]=p_index1
            ind[10]=p_index2
            obs_min_dist=min_distance[str(ind[0])]
            ind.fitness.values = (obs_min_dist,min_speed,len(lane_coverage[scenario_counter]),offroad,accl,hardbrake,)
        scenario_counter+=1
    
    print("  Evaluated %i individuals" % len(pop))

    while len(GLOBAL_LANE_COVERAGE) <= TOTAL_LANES:
        g = g + 1
        scenario_counter=1
        print("-- Generation %i --" % g)
        
        for deme in pop:
            offspring = list(map(toolbox.clone, deme))

            # Apply crossover and mutation on the offspring
            for child1, child2 in zip(offspring[::2], offspring[1::2]):
                if random.random() < CXPB:
                    toolbox.mate(child1, child2)
                    del child1.fitness.values
                    del child2.fitness.values

            for mutant in offspring:
                if random.random() < MUTPB:
                    toolbox.mutate(mutant)
                    del mutant.fitness.values
                if random.random() < DELPB:
                    worst_ind=tools.selWorst(offspring,1)[0]
                    offspring.remove(worst_ind)
                    del mutant.fitness.values
                if random.random() < ADDPB and len(hof)!=0:
                    best_ind=hof[random.randint(0,int(len(hof)/2))]
                    offspring.append(best_ind)
                    del mutant.fitness.values
            
            record_name="Generation{}_Scenario{}".format(g,scenario_counter)
            min_distance,lanes,min_speed,p_index1,p_index2,adc_routing,offroad,accl,hardbrake,speed_limits=runScenario(offspring,scenario_counter,record_name)
            lanes.remove('')
            GLOBAL_LANE_COVERAGE.update(lanes)
            lane_coverage[scenario_counter]=lane_coverage[scenario_counter].union(lanes)
            adc_loc_decider[scenario_counter].add((p_index1,p_index2))

            sum=0
            c_counter=0
            for ind in offspring:
                ind[9]=p_index1
                ind[10]=p_index2
                obs_min_dist=min_distance[str(ind[0])]
                ind.fitness.values = (obs_min_dist,min_speed,len(lane_coverage[scenario_counter]),offroad,accl,hardbrake,)    
                print("Obs:%s Distance_to_ADC:%.2f" % (ind,ind.fitness.values[0]))
                sum+=obs_min_dist
                hasCollision=reportCollison(ind[0],g,scenario_counter,obs_min_dist,adc_routing)
                if hasCollision:
                    c_counter+=1
            reportViolations(g,scenario_counter,min_speed,offroad,accl,hardbrake,adc_routing,speed_limits,c_counter)
            
            print("-"*130)
            print("Generation%s_Scenario%s #Obs:%s Scenario_Diversity:%s AVG_OBS2ADC_Dist_Score:%.2f Speed_Below_Limit:%.2f ADC2LaneBound_Dist:%.2f FaseAccl:%.2f HardBrake:%.2f LaneCoverage:%s" 
                    % (g,scenario_counter,len(offspring),diversity_counter,sum/len(offspring),min_speed,offroad,accl,hardbrake,len(lane_coverage[scenario_counter])))
            print("-"*130)

            hof.insert(tools.selBest(offspring,1)[0])
            deme = toolbox.select(deme+offspring,len(offspring))
            scenario_counter+=1
        print("Global_Lane_Coverage:{}".format(len(GLOBAL_LANE_COVERAGE)))
    print("-- End of (successful) evolution --")
    
    # ------- Final Results -------
    end_time = time.time()
    print("-- Execution Time: %.2f  seconds --\n" % (end_time-start_time))
    print("*** Total Num. of Lanes Covered:%s out of 3061 ***\n" % len(GLOBAL_LANE_COVERAGE))
    printResults(lane_coverage,pop,ptl_dict,obstacle_type)

if __name__ == "__main__":
    main()

