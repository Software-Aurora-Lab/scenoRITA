![Figure 1](/images/logo.png) 

This repository contains:
* scenoRITA, a tool designed to automatically generate driving scenarios that exposes AV software to safety critical situations. 
* Video recordings of the four case studies discussed in the paper (Section 4.2).
* An extended version of the evaluation results.

## Overview: ##
The figure below shows the overall workflow of scenoRITA. Our main goal is to create valid and realistic driving scenarios that expose AVs software to safety violations. scenoRITA achieves this goal as follows:
1. It takes as an input a set of Domain-Specific Constraints which dictates what constitutes a valid and realistic driving-scenario (e.g.,obstacles should be moving in the direction of traffic in the lane and have valid obstacle identifiers).
2. The Scenario Generator uses a genetic algorithm to produce driving scenarios with randomly generated but valid obstacles, following the Domain-Specific Constraints. The genetic algorithm evolves the driving scenarios with the aim of finding scenarios with safety violations.
3. Generated Scenarios Player converts the genetic representation of scenarios(Generated Scenarios), from the previous step, into driving simulations where the planning output of the AV under test is producedand recorded by Planning Output Recorder.
4. The planning outputis then evaluated by Grading Metrics Checker for Safety and Comfort Violations. 

![Figure 2](/images/approach.png) 

## scenoRITA 101: ## 
scenoRITA's structure in a nutshell:
```
|-- scenario_generator
|-- scenario_player
|-- auxiliary
|-- grading_metrics
|-- case-studies
|-- evaluation-results
```

* **scenario_generator**: Contains the genetic algorithm scripts needed to generate scenario representations.  
* **scenario_player**: Contains the scripts needed to launch the simulator, and transform the scenario representations into simulations.
* **auxiliary**: Auxiliary scripts used by scenario_player and scenario_generator.
* **grading_metrics**: Contains the test oracles checked during the text execution process. 
* **evaluation-results**: `scenoRITA-Evaluation-Results.xlsx` under `evaluation-results` contains an extended version of the evaluation results presented in the paper (Section 4). `results` Contains a sample of recorded scenrios with safety violations (requires [Apollo CyberRT](https://github.com/ApolloAuto/apollo/blob/a29a563e95944b603ab9be338cce46e6486a89be/cyber/README.md) to re-play them). 

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
* Pre-installation of [Apollo Docker containers](https://github.com/ApolloAuto/apollo/blob/a29a563e95944b603ab9be338cce46e6486a89be/docs/quickstart/apollo_software_installation_guide.md)

## Using scenoRITA: ##
* To run scenorRITA from End-To-End, change dicrectory to `scenario_generator`:
``` 
cd scenario_generator/
```
* Modify `ga_framework.py` (e.g., No. of generations, domain specific constrains etc). Then run the framework using the followiing command:
``` 
python3 ga_framework.py
```
