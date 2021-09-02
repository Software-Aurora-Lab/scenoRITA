import random
import subprocess
import os
import sys
import time
import json
from map_info_parser import *
from feature_generator import *
from automation.auxiliary.map import map_tools

obs_folder = "/apollo/modules/tools/perception/obstacles/"
dest="/apollo/automation/scenario_generator"
features_file="rand_features.csv"
ga_file="ga_output.csv"
timer_file="execution_time.csv"

#return a mapping from x,y coor to laneIds and viceVersa (info extracted from routing_map of sunnyvale_loop)
ptl_dict, ltp_dict, diGraph=initialize()
obstacle_type=["PEDESTRIAN","BICYCLE","VEHICLE"]

def check_trajectory(p_index1,p_index2):
    valid_path=False
    while not valid_path:
        valid_path=validatePath(p_index1,p_index2,ptl_dict,ltp_dict,diGraph) and longerTrace(list(ptl_dict.keys())[p_index1],list(ptl_dict.keys())[p_index2],ptl_dict,ltp_dict,diGraph)         
        if not valid_path:
            p_index1=random.randint(0,len(ptl_dict.keys())-1)
            p_index2=random.randint(0,len(ptl_dict.keys())-1)
    return p_index1,p_index2

def check_obs_type(type_index):
    obs_type=obstacle_type[type_index]
    if obs_type is "VEHICLE":
        length=random.uniform(4.0,14.5)
        height=random.uniform(1.5,4.7)
        width=random.uniform(1.5,2.5)
        speed=random.uniform(2.5,20)
        diversity_counter["V"]+=1
    if obs_type is "PEDESTRIAN":
        length=random.uniform(0.2,0.5)
        height=random.uniform(0.97,1.87)
        width=random.uniform(0.3,0.8)
        speed=random.uniform(1.25,3)
        diversity_counter["P"]+=1
    if obs_type is "BICYCLE":
        length=random.uniform(1,2.5)
        height=random.uniform(1,2.5)
        width=random.uniform(0.5,1)
        speed=random.uniform(1.75,7)
        diversity_counter["B"]+=1
    return length,width,height,speed

def runScenario(obs_num,record_name):
    #to start with a fresh set of obstacles for the current scnerio
    os.system("rm -f /apollo/modules/tools/perception/obstacles/*")
    global diversity_counter
    diversity_counter={"V":0,"P":0,"B":0}
    scenario=[]
    for obs in range(obs_num):
        obs_id=random.randint(0, 30000)
        #ensure there are no two obstacles with similar id
        unique_obs_id=False
        while not unique_obs_id:
            if os.path.exists(os.path.join(obs_folder,"sunnyvale_loop_obs{}.json".format(obs_id))):
                obs_id=random.randint(0,30000)
            else:
                unique_obs_id=True
        theta=random.uniform(-3.14, 3.14)
        #verify that obstacle type and size is realistic
        obs_type=random.randint(0, 2)
        length,width,height,speed=check_obs_type(obs_type)  
        p_index1,p_index2=check_trajectory(random.randint(0, len(ptl_dict.keys())-1),random.randint(0, len(ptl_dict.keys())-1))
        #get the x,y coor coorespoding to index i1 and i2
        p1=list(ptl_dict.keys())[p_index1]
        p2=list(ptl_dict.keys())[p_index2]
        #get the correspodning lane id where the points reside
        lane1=ptl_dict[p1]
        lane2=ptl_dict[p2]
        #find the path between two lanes
        path=nx.shortest_path(diGraph, source=lane1, target=lane2)
        #generate the desc files (each desc file corresponds to one individual/obstacle)
        desc=generateObsDescFile(obs_id,theta,length,width,height,speed,obstacle_type[obs_type])
        desc=produceTrace(p1,p2,path,ltp_dict,desc)
        filename="sunnyvale_loop_obs"+str(desc["id"])+".json"
        with open(os.path.join(obs_folder, filename), 'w') as outfile:
            json.dump(desc,outfile)

        scenario.append([obs_id,p_index1,p_index2,theta,length,width,height,speed,obs_type])
    failed=True
    num_runs=0
    while failed:
        #if scenario has been restarted x times, restart the moodules and sim control 
        if num_runs % 10 == 0 and num_runs != 0:
            os.system("bash /apollo/scripts/bootstrap.sh stop")
            time.sleep(10)
            os.system("bash /apollo/scripts/bootstrap.sh start")
            time.sleep(10)
            os.system("bash /apollo/automation/auxiliary/modules/start_modules.sh")
            time.sleep(10)
            os.system("source /apollo/cyber/setup.bash")
            time.sleep(2)
            print("attempted %s run" % num_runs)
        # ------- sending valid adc routing -------
        valid_path=False
        while not valid_path:
            p_index1=random.randint(0,len(ptl_dict.keys())-1)
            p_index2=random.randint(0,len(ptl_dict.keys())-1)
            start_point=tuple(map(float,list(ptl_dict.keys())[p_index1].split('-')))
            if not map_tools.all_points_not_in_junctions(start_point):
                p1=list(ptl_dict.keys())[p_index1]
                p2=list(ptl_dict.keys())[p_index2]
                continue
            valid_path=validatePath(p_index1,p_index2,ptl_dict,ltp_dict,diGraph)
        p1=list(ptl_dict.keys())[p_index1]
        p2=list(ptl_dict.keys())[p_index2]
        adc_routing=p1.replace('-',',')+","+p2.replace('-',',')
        # ------- running the scneario -------
        scenario_player_cmd='bazel run --experimental_ui_limit_console_output=1 //automation/scenario_player:run_automation -- -rv '+adc_routing+' -o '+record_name
        scenario_player_output = subprocess.check_output(scenario_player_cmd, shell=True)
        scenario_player_output=str(scenario_player_output)[2:-3]
        num_runs=num_runs+1
        #print(scenario_player_output)
        #if the adc didn't move or the adc was travelling outside the map boundaries, then re-run scenrio with new routing info  
        if scenario_player_output == 'None':
            continue
        scenario_player_output=scenario_player_output.split('\\n')
        min_distance=eval(scenario_player_output[0])
        #the return number of obstacles must match the ones in the individual 
        if len(min_distance) != obs_num:
            continue
        else:
            failed=False
    #scenario run successfully
    sim_time=float(scenario_player_output[8])*num_runs
    orcle_time=float(scenario_player_output[9])*num_runs
    lanes,min_distance,speeding_min,uslc_min,fastAccl_min,hardBrake_min=runOracles(scenario_player_output,record_name,scenario)
    return lanes,min_distance,speeding_min,uslc_min,fastAccl_min,hardBrake_min,sim_time,orcle_time,num_runs

# ------- Main Function -------
def main():
    OBS_MAX=15
    OBS_MIN=1
    TOTAL_LANES=60
    ETIME=43200 # execution time end (in seconds) after 12 hours 
    GLOBAL_LANE_COVERAGE=set()  
    
    #store features output and evolution output
    labels="record_name,c_x,c_y,c_type,adc_heading,adc_speed,obs_id,obs_heading,obs_speed,obs_type,obs_len,obs_wid,obs_height,"\
    "speeding_x,speeding_y,speeding_value,speeding_duration,speeding_heading,lanes_speed_limit,uslc_x,uslc_y,uslc_duration,uslc_heading,"\
    "fastAccl_x,fastAccl_y,fastAccl_value,fastAccl_duration,fastAccl_heading,hardBrake_x,hardBrake_y,hardBrake_value,hardBrake_duration,hardBrake_heading,"\
    "c_counter,speeding_counter,uslc_counter,fastAccl_counter,hardBrake_counter,totalV\n"
    with open(os.path.join(dest,features_file),'w') as ffile:
      ffile.write(labels)
    labels="RecordName,ObsNum,P,B,V,AVG_OBS2ADC_Distance,Speed_Below_Limit,ADC2LaneBound_Distance,FastAccl,HardBrake\n"
    with open(os.path.join(dest,ga_file),'a+') as gfile:
      gfile.write(labels)
    labels="RecordName,Simulation,Oracles,MISC,E2E,RetryNo\n"
    with open(os.path.join(dest,timer_file),'a+') as tfile:
        tfile.write(labels)

    os.system("rm -rf /apollo/automation/grading_metrics/Safety_Violations/*")
    print("Start of scenoRITA Random:")
    start_time=time.time()
    scenario_counter=1
    while len(GLOBAL_LANE_COVERAGE) < TOTAL_LANES and (time.time()-start_time) <= ETIME :
        e2e_time=time.time()
        record_name="Scenario{}".format(scenario_counter)
        #generate number of obstacles per current scenario 
        obs_num=random.randint(OBS_MIN,OBS_MAX)
        #populate the description files of obstacles and run scenario
        lanes,min_distance,speeding_min,uslc_min,fastAccl_min,hardBrake_min,sim_time,orcle_time,num_runs=runScenario(obs_num,record_name)
        lanes.remove('')
        GLOBAL_LANE_COVERAGE.update(lanes)
        sum=0
        for obs_id in min_distance:
            obs_min_dist=min_distance[obs_id]    
            sum+=obs_min_dist
        with open(os.path.join(dest,ga_file),'a+') as gfile:
            gfile.write("%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n"
            % (record_name,obs_num,diversity_counter["P"],diversity_counter["B"],diversity_counter["V"],sum/obs_num,speeding_min,uslc_min,fastAccl_min,hardBrake_min))
        e2e_time=time.time()-e2e_time
        misc_time=e2e_time-sim_time-orcle_time
        with open(os.path.join(dest,timer_file),'a+') as tfile:
            tfile.write("{},{:.2f},{:.2f},{:.2f},{:.2f},{}\n".format(record_name,sim_time,orcle_time,misc_time,e2e_time,num_runs))
        scenario_counter+=1
    print("-- End of (successful) evolution --")
     
    # ------- Final Results -------
    end_time = time.time()
    print("-- Execution Time: %.2f  seconds --\n" % (end_time-start_time))
    print("*** Total Num. of Lanes Covered:%s out of %s ***\n" % (len(GLOBAL_LANE_COVERAGE),TOTAL_LANES))

if __name__ == "__main__":
    main()
