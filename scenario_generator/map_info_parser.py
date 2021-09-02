#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Parser to extract nodes and edges from a map"""

__author__ = "Sumaya Almanee"
__copyright__ = "Copyright 2020, UCI"

import sys,getopt
import json 
import networkx as nx
import random
from collections import defaultdict

def parseLanes(lanes):
    #generate a mapping from points to lanes ids and vice versa
    ptl={}
    ltp=defaultdict(list)
    for lane in lanes:
        for point in lane["central_curve"]["segment"]["line_segment"]:
            ptl[str(point["x"])+"-"+str(point["y"])]=lane["lane_id"] 
            ltp[lane["lane_id"]].append(str(point["x"])+"-"+str(point["y"]))
    return ptl,ltp

def connectLanes(edges):
    DG = nx.DiGraph()	
    for edge in edges:
        DG.add_edge(edge["from_lane_id"],edge["to_lane_id"], weight=edge["cost"])

    return DG

def initialize():
    data=json.load(open('/apollo/automation/scenario_generator/maps/borregas_ave/routing_map.json','r'))
    #data=json.load(open('/apollo/automation/scenario_generator/maps/san_mateo/routing_map.json','r'))
    #data=json.load(open('/apollo/automation/scenario_generator/maps/sunnyvale_loop/routing_map.json','r'))
    ptl_dict, ltp_dict=parseLanes(data["nodes"])
    diGraph=connectLanes(data["edges"])
    return ptl_dict, ltp_dict, diGraph

def validatePath(i1,i2,ptl_dict,ltp_dict,diGraph):
    global p1,p2,lane1,lane2,path
    
    if i1==i2:
        return False
    
    #get the x,y coor coorespoding to index i1 and i2 
    p1=list(ptl_dict.keys())[i1]
    p2=list(ptl_dict.keys())[i2]
    #get the correspodning lane id where the points reside
    lane1=ptl_dict[p1]
    lane2=ptl_dict[p2]

    #check whether the source and dest nodes exist and if there's a path between them 
    if (not diGraph.has_node(lane1)) or (not diGraph.has_node(lane2)) or (not nx.has_path(diGraph,lane1,lane2)):
        return False

    return True

def longerTrace(p1,p2,ptl_dict,ltp_dict,diGraph):
    #iterate through the path between two lanes and generate all coordinates of lanes in path
    lane1=ptl_dict[p1]
    lane2=ptl_dict[p2]
    #find the path between two lanes
    path=nx.shortest_path(diGraph, source=lane1, target=lane2)

    pt_count=0
    for i in range(0,len(path)):
        laneCoor=list(ltp_dict[path[i]])

        if p1 in laneCoor and p2 in laneCoor and laneCoor.index(p1)>laneCoor.index(p2):
            temp=p1
            p1=p2
            p2=p1
            
        if i==0:
            pt_count+=len(laneCoor[laneCoor.index(p1):-1])
        elif i==len(path)-1:
            pt_count+=len(laneCoor[:laneCoor.index(p2)])
        else:
            pt_count+=len(laneCoor[:-1])
    
    #print("pt_count %s" % pt_count)
    if pt_count >= 20:
      return True
    else:
      return False

def printTrace(coorList,desc):
    for coor in coorList:
        x_y=list(map(float,coor.split("-")))
        x_y.insert(2,0.0)
        desc["trace"].append(x_y)
    return desc

def produceTrace(p1,p2,path,ltp_dict,desc):
    #iterate through the path between two lanes and print all coordinates of lanes in path
    for i in range(0,len(path)):
        laneCoor=list(ltp_dict[path[i]])

        if p1 in laneCoor and p2 in laneCoor and laneCoor.index(p1)>laneCoor.index(p2):
            temp=p1
            p1=p2
            p2=p1
            
        if i==0:
            desc=printTrace(laneCoor[laneCoor.index(p1):-1],desc)
        elif i==len(path)-1:
            desc=printTrace(laneCoor[:laneCoor.index(p2)],desc)
        else:
            desc=printTrace(laneCoor[:-1],desc)
    
    desc["position"]=desc["trace"][0]
    #print("id: %s trace_len %s" % (desc["id"],len(desc["trace"])))
    return desc

def generateObsDescFile(oid,theta,length,width,height,speed,otype):
    desc={}
    desc["id"]=oid
    desc["position"]=0
    desc["theta"]=theta
    desc["length"]=length
    desc["width"]=width
    desc["height"]=height
    desc["speed"]=speed
    desc["tracking_time"]=1.0
    desc["type"]=otype
    desc["trace"]=[]

    return desc


if __name__ == "__main__":
    #initialization:
    ptl_dict, ltp_dict, diGraph=initialize()
    rand1=random.randint(0,len(ptl_dict.keys()))
    rand2=random.randint(0,len(ptl_dict.keys()))
    vbool=validatePath(rand1,rand2,ptl_dict,ltp_dict,diGraph)
    for k in ptl_dict:
        print("%s:%s" % (k,ptl_dict[k]))

    if(not vbool):
        print("There is no links between the two points")
        sys.exit()

    desc=generateObsDescFile(random.randint(0,30000),random.uniform(-3.14,3.14),random.randint(1,7),random.randint(1,10),random.randint(1,30),random.randint(1,55))
    path=nx.shortest_path(diGraph, source=lane1, target=lane2)
    desc=produceTrace(p1,p2,path,ltp_dict,desc)

    fileName="sunnyvale_loop_obs"+str(desc["id"])+".json"
    with open(fileName, 'w') as outfile:
        json.dump(desc,outfile)

	





	
