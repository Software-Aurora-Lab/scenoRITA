![Figure 1](/images/logo.png) 

This repository contains:
* scenoRITA, a tool designed to automatically generate driving scenarios that exposes AV software to safety critical situations. 
* Video recordings of 10 case studies discussed in the paper (Section 4.2).
* An extended version of the evaluation results.

## Overview: ##
The figure below shows the overall workflow of scenoRITA. Our main goal is to create valid and effective driving scenarios that expose AVs software to safety and comfort violations. scenoRITA achieves this goal as follows:
1. It takes as an input a set of domain-specific constraints which dictates what constitutes a valid driving-scenario (e.g.,obstacles should be moving in the direction of traffic in the lane and have valid obstacle identifiers).
2. The Scenario Generator uses a genetic algorithm to produce driving scenarios with randomly generated but valid obstacles, following the domain-specific constraints. The genetic algorithm evolves the driving scenarios with the aim of finding scenarios with safety and comfort violations.
3. Generated Scenarios Player converts the genetic representation of scenarios(Generated Scenarios), from the previous step, into driving simulations where the planning output of the AV under test is producedand then recorded by Planning Output Recorder.
4. The planning outputis is then evaluated by Grading Metrics Checker for safety and comfort violations. 
5. When the evolution process terminates,the Duplicate Violations Detector inspects the violations produced by Grading Metrics Checker to eliminate any duplicate violations,and produces a set of unique safety and comfort violations.

![Figure 2](/images/approach.png) 

## scenoRITA 101: ## 
scenoRITA's structure in a nutshell:
```
|-- scenario_generator
|-- scenario_player
|-- auxiliary
|-- grading_metrics
|-- scenoRITA-Evaluation
```

* **scenario_generator**: Contains the genetic algorithm scripts needed to generate scenario representations:
  * For scenoRITA_plus use: `scenoRITA_mut.py`, for scenoRITA_minus use: `scenoRITA_immut.py`, and for scenoRITA_random use: `scenoRITA_random.py` 
  * For clustering use: `Dpython3 DBSCAN_cluster.py <features file>`
* **scenario_player**: Contains the scripts needed to launch the simulator, and transform the scenario representations into simulations.
* **auxiliary**: Auxiliary scripts used by scenario_player and scenario_generator.
* **grading_metrics**: Contains the test oracles checked during the text execution process. 
* **evaluation-results**: contains an extended version of the evaluation results presented in the paper (Section 4). Please click [here](https://figshare.com/s/0c4e2b72b4915f9fd077) for the 10 case studies described in Section 4.2.

## Prerequisites: ##
* Any linux distrbutions, preferably Ubuntu 18.04.5 LTS (Bionic Beaver)
* Python3
* Pre-installation of deap:
```
pip3 install deap
```
* Pre-installation of networkx:
```
pip3 install networkx
``` 
* Pre-installation of pandas:
```
pip3 install pandas
``` 
* Pre-installation of sklearn:
```
pip3 install sklearn
``` 
* Pre-installation of kneed:
```
pip3 install kneed
``` 
* Pre-installation of [Apollo Docker containers](https://github.com/ApolloAuto/apollo/blob/a29a563e95944b603ab9be338cce46e6486a89be/docs/quickstart/apollo_software_installation_guide.md)

## Using scenoRITA: ##
* To run scenorRITA from End-To-End, change dicrectory to `scenario_generator`:
``` 
cd scenario_generator/
```
* Modify `scenoRITA_mut.py` (e.g., No. of generations, domain specific constrains etc). Then run the framework using the followiing command:
``` 
bazel run scenoRITA_mut 
```
