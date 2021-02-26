import sys
import os 

violations_folder="/apollo/automation/oracles/Safety_Violations"
obs_folder = "/apollo/modules/tools/perception/obstacles/*"

def reportCollison(obs_id,g,scenario_counter,obs_min_dist,adc_routing):
    sub_folder_name="Generation{}_Scenario{}".format(g,scenario_counter)
    dest=os.path.join(violations_folder,sub_folder_name)
    report_name="safety_violations.txt"
    if obs_min_dist <= 0:
        if not (os.path.exists(dest)):
            os.mkdir(dest)
            os.system("mv {} {}".format(obs_folder,dest))
        with open(os.path.join(dest,report_name), 'a+') as file:
            file.write("Violation_Type:Collision,ADC_Routing:{},OBS_ID:{},OBS2ADC_Distance:{}\n".format(adc_routing,obs_id,obs_min_dist))
        return True
    return False

def writer(dest,obs_folder,report_name,adc_routing,violationName,violationDesc,violationValue):
    if not (os.path.exists(dest)):
        os.mkdir(dest)
        os.system("mv {} {}".format(obs_folder,dest))
    with open(os.path.join(dest,report_name), 'a+') as file:
        file.write("Violation_Type:{},ADC_Routing:{},{}:{}\n".format(violationName,adc_routing,violationDesc,violationValue))

def reportViolations(g,scenario_counter,min_speed,offroad,accl,hardbrake,adc_routing,speed_limits,c_counter):
    sub_folder_name="Generation{}_Scenario{}".format(g,scenario_counter)
    dest=os.path.join(violations_folder,sub_folder_name)
    report_name="safety_violations.txt"
    s_counter=0
    f_counter=0
    a_counter=0
    b_counter=0

    if min_speed < -5:
        writer(dest,obs_folder,report_name,adc_routing,"Speeding","Speed_Above_Limit",min_speed)
        s_counter+=1
    if offroad <= 0:
        writer(dest,obs_folder,report_name,adc_routing,"Off-Road","ADC2LaneBound_Distance",offroad)
        f_counter+=1
    if accl >= 4:
        writer(dest,obs_folder,report_name,adc_routing,"Fast-Acceleration","Acceleration_Above",accl)
        a_counter+=1
    if hardbrake <= -4:
        writer(dest,obs_folder,report_name,adc_routing,"Hard-Braking","Acceleration_Below",hardbrake)
        b_counter+=1
    
    with open("all_violations.txt", 'a+') as file:
        file.write("Generation{},Scenario{},{},{},{},{},{},{}\n".format(g,scenario_counter,c_counter,s_counter,f_counter,a_counter,b_counter,speed_limits))

def printResults(lane_coverage,pop,ptl_dict,obstacle_type):
    most_coverage_scenario=pop[0]
    collision_risk_scenario=pop[0]
    speeding_risk_scenario=pop[0]
    offroad_risk_scenario=pop[0]
    fast_accl_scenario=pop[0]
    hard_brake_scenario=pop[0]

    dist_min=sys.maxsize
    max_coverage=-1
    scenario_counter=1

    MOST_laneCoverage_Num=1
    CD_laneCoverage_Num=1
    SD_laneCoverage_Num=1
    OR_laneCoverage_Num=1
    FA_laneCoverage_Num=1
    HB_laneCoverage_Num=1

    for deme in pop:
        #scenario coverage:
        scenario_coverage=len(lane_coverage[scenario_counter])

        if scenario_coverage>max_coverage:
            max_coverage=scenario_coverage
            most_coverage_scenario=deme
            MOST_laneCoverage_Num=scenario_counter

        #scenario collision detection:
        obs2adc_min=sys.maxsize
        for ind in deme:
            obs2adc_min=min(obs2adc_min,ind.fitness.values[0])

        if obs2adc_min<dist_min:
            dist_min=obs2adc_min
            collision_risk_scenario=deme
            CD_laneCoverage_Num=scenario_counter

        #scenario speed detection:
        scenario_SD=deme[0].fitness.values[1]
        temp_SD=speeding_risk_scenario[0].fitness.values[1]

        if scenario_SD<temp_SD:
            speeding_risk_scenario=deme
            SD_laneCoverage_Num=scenario_counter

        #scenario offroad detection:
        scenario_OR=deme[0].fitness.values[3]
        temp_OR=offroad_risk_scenario[0].fitness.values[3]

        if scenario_OR<temp_OR:
            offroad_risk_scenario=deme
            OR_laneCoverage_Num=scenario_counter

        #scenario fast acceleration:
        scenario_FA=deme[0].fitness.values[4]
        temp_FA=fast_accl_scenario[0].fitness.values[4]

        if scenario_FA>temp_FA:
            fast_accl_scenario=deme
            FA_laneCoverage_Num=scenario_counter

        #scenario hard braking:
        scenario_HB=deme[0].fitness.values[5]
        temp_HB=hard_brake_scenario[0].fitness.values[5]

        if scenario_HB<temp_HB:
            hard_brake_scenario=deme
            HB_laneCoverage_Num=scenario_counter

        scenario_counter+=1

    print("Scenario#%s has the most LANE COVERAGE ..." % MOST_laneCoverage_Num)
    print("-"*65)
    scenario_fitness=0
    for ind in most_coverage_scenario:
        scenario_fitness+=ind.fitness.values[0]
        print("obs_id:%s start_loc:%s end_loc:%s heading:%.2f length:%.2f width:%.2f height:%.2f speed:%.2f type:%s dis(obs2adc):%.2f" %
                (ind[0],list(ptl_dict.keys())[ind[1]],list(ptl_dict.keys())[ind[2]],ind[3],ind[4],ind[5],ind[6],ind[7],obstacle_type[ind[8]],ind.fitness.values[0]))
    scenario_fitness=scenario_fitness/len(most_coverage_scenario)
    print("#Obstacles:%s AVG_OBS2ADC_Distance_Score:%.2f Lane_Coverage:%s/3061 Speed_Below_Limit:%.2f ADC2LaneBound_Distance:%.2f FastAccl:%.2f HardBrake:%.2f" %
            (len(most_coverage_scenario),scenario_fitness,len(lane_coverage[MOST_laneCoverage_Num]),
                most_coverage_scenario[0].fitness.values[1],most_coverage_scenario[0].fitness.values[3],most_coverage_scenario[0].fitness.values[4],most_coverage_scenario[0].fitness.values[5]))
    print("-"*65)
    print()

    print("Scenario#%s has the highest COLLISION RISK ..." % CD_laneCoverage_Num)
    print("-"*65)
    scenario_fitness=0
    for ind in collision_risk_scenario:
        scenario_fitness+=ind.fitness.values[0]
        print("obs_id:%s start_loc:%s end_loc:%s heading:%.2f length:%.2f width:%.2f height:%.2f speed:%.2f type:%s dis(obs2adc):%.2f" %
                (ind[0],list(ptl_dict.keys())[ind[1]],list(ptl_dict.keys())[ind[2]],ind[3],ind[4],ind[5],ind[6],ind[7],obstacle_type[ind[8]],ind.fitness.values[0]))
    scenario_fitness=scenario_fitness/len(collision_risk_scenario)
    print("#Obstacles:%s AVG_OBS2ADC_Distance_Score:%.2f Lane_Coverage:%s/3061 Speed_Below_Limit:%.2f ADC2LaneBound_Distance:%.2f FastAccl:%.2f HardBrake:%.2f" %
            (len(collision_risk_scenario),scenario_fitness,len(lane_coverage[CD_laneCoverage_Num]),
                collision_risk_scenario[0].fitness.values[1],collision_risk_scenario[0].fitness.values[3],collision_risk_scenario[0].fitness.values[4],collision_risk_scenario[0].fitness.values[5]))
    print("-"*65)
    print()

    print("Scenario#%s has the highest SPEED VIOLATION RISK ..." % SD_laneCoverage_Num)
    print("-"*65)
    scenario_fitness=0
    for ind in speeding_risk_scenario:
        scenario_fitness+=ind.fitness.values[0]
        print("obs_id:%s start_loc:%s end_loc:%s heading:%.2f length:%.2f width:%.2f height:%.2f speed:%.2f type:%s dis(obs2adc):%.2f" %
                (ind[0],list(ptl_dict.keys())[ind[1]],list(ptl_dict.keys())[ind[2]],ind[3],ind[4],ind[5],ind[6],ind[7],obstacle_type[ind[8]],ind.fitness.values[0]))
    scenario_fitness=scenario_fitness/len(speeding_risk_scenario)
    print("#Obstacles:%s AVG_OBS2ADC_Distance_Score:%.2f Lane_Coverage:%s/3061 Speed_Below_Limit:%.2f ADC2LaneBound_Distance:%.2f FastAccl:%.2f HardBrake:%.2f" %
            (len(speeding_risk_scenario),scenario_fitness,len(lane_coverage[SD_laneCoverage_Num]),
                speeding_risk_scenario[0].fitness.values[1],speeding_risk_scenario[0].fitness.values[3],speeding_risk_scenario[0].fitness.values[4],speeding_risk_scenario[0].fitness.values[5]))
    print("-"*65)
    print()

    print("Scenario#%s has the highest OFFROAD VIOLATION RISK ..." % OR_laneCoverage_Num)
    print("-"*65)
    scenario_fitness=0
    for ind in offroad_risk_scenario:
        scenario_fitness+=ind.fitness.values[0]
        print("obs_id:%s start_loc:%s end_loc:%s heading:%.2f length:%.2f width:%.2f height:%.2f speed:%.2f type:%s dis(obs2adc):%.2f" %
                (ind[0],list(ptl_dict.keys())[ind[1]],list(ptl_dict.keys())[ind[2]],ind[3],ind[4],ind[5],ind[6],ind[7],obstacle_type[ind[8]],ind.fitness.values[0]))
    scenario_fitness=scenario_fitness/len(offroad_risk_scenario)
    print("#Obstacles:%s AVG_OBS2ADC_Distance_Score:%.2f Lane_Coverage:%s/3061 Speed_Below_Limit:%.2f ADC2LaneBound_Distance:%.2f FastAccl:%.2f HardBrake:%.2f" %
            (len(offroad_risk_scenario),scenario_fitness,len(lane_coverage[OR_laneCoverage_Num]),
                offroad_risk_scenario[0].fitness.values[1],offroad_risk_scenario[0].fitness.values[3],offroad_risk_scenario[0].fitness.values[4],offroad_risk_scenario[0].fitness.values[5]))
    print("-"*65)
    print()

    print("Scenario#%s has the highest FAST ACCELERATION RISK ..." % FA_laneCoverage_Num)
    print("-"*65)
    scenario_fitness=0
    for ind in fast_accl_scenario:
        scenario_fitness+=ind.fitness.values[0]
        print("obs_id:%s start_loc:%s end_loc:%s heading:%.2f length:%.2f width:%.2f height:%.2f speed:%.2f type:%s dis(obs2adc):%.2f" %
                (ind[0],list(ptl_dict.keys())[ind[1]],list(ptl_dict.keys())[ind[2]],ind[3],ind[4],ind[5],ind[6],ind[7],obstacle_type[ind[8]],ind.fitness.values[0]))
    scenario_fitness=scenario_fitness/len(fast_accl_scenario)
    print("#Obstacles:%s AVG_OBS2ADC_Distance_Score:%.2f Lane_Coverage:%s/3061 Speed_Below_Limit:%.2f ADC2LaneBound_Distance:%.2f FastAccl:%.2f HardBrake:%.2f" %
            (len(fast_accl_scenario),scenario_fitness,len(lane_coverage[FA_laneCoverage_Num]),
                fast_accl_scenario[0].fitness.values[1],fast_accl_scenario[0].fitness.values[3],fast_accl_scenario[0].fitness.values[4],fast_accl_scenario[0].fitness.values[5]))
    print("-"*65)
    print()

    print("Scenario#%s has the highest HARD BRAKING RISK ..." % HB_laneCoverage_Num)
    print("-"*65)
    scenario_fitness=0
    for ind in hard_brake_scenario:
        scenario_fitness+=ind.fitness.values[0]
        print("obs_id:%s start_loc:%s end_loc:%s heading:%.2f length:%.2f width:%.2f height:%.2f speed:%.2f type:%s dis(obs2adc):%.2f" %
                (ind[0],list(ptl_dict.keys())[ind[1]],list(ptl_dict.keys())[ind[2]],ind[3],ind[4],ind[5],ind[6],ind[7],obstacle_type[ind[8]],ind.fitness.values[0]))
    scenario_fitness=scenario_fitness/len(hard_brake_scenario)
    print("#Obstacles:%s AVG_OBS2ADC_Distance_Score:%.2f Lane_Coverage:%s/3061 Speed_Below_Limit:%.2f ADC2LaneBound_Distance:%.2f FastAccl:%.2f HardBrake:%.2f" %
            (len(hard_brake_scenario),scenario_fitness,len(lane_coverage[HB_laneCoverage_Num]),
                hard_brake_scenario[0].fitness.values[1],hard_brake_scenario[0].fitness.values[3],hard_brake_scenario[0].fitness.values[4],hard_brake_scenario[0].fitness.values[5]))
    print("-"*65)

