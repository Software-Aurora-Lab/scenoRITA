![Figure 1](/images/logo.png) 

This repository contains:
* scenoRITA, a tool designed to automatically generate driving scenarios that exposes AV software to safety critical situations. 
* Video recordings of 10 case studies discussed in the paper (Section 4.2).
* An extended version of the evaluation results.

## Overview: ##
The figure below shows the overall workflow of scenoRITA. Our main goal is to create valid and effective driving scenarios that expose AVs software to safety and comfort violations. scenoRITA achieves this goal as follows:
1. It takes as an input a set of domain-specific constraints which dictates what constitutes a valid driving-scenario (e.g.,obstacles should be moving in the direction of traffic in the lane and have valid obstacle identifiers).
2. The Scenario Generator uses a genetic algorithm to produce driving scenarios with randomly generated but valid obstacles, following the domain-specific constraints. The genetic algorithm evolves the driving scenarios with the aim of finding scenarios with safety and comfort violations.
3. Generated Scenarios Player converts the genetic representation of scenarios(Generated Scenarios), from the previous step, into driving simulations where the planning output of the AV under test is produced and recorded by Planning Output Recorder.
4. The planning output is then evaluated by Grading Metrics Checker for safety and comfort violations. 
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
  * For clustering use: `python3 DBSCAN_cluster.py <features file>`
* **scenario_player**: Contains the scripts needed to launch the simulator, and transform the scenario representations into simulations.
* **auxiliary**: Auxiliary scripts used by scenario_player and scenario_generator.
* **grading_metrics**: Contains the test oracles checked during the text execution process. 
* **evaluation-results**: contains an extended version of the evaluation results presented in the paper (Section 4). Please click [here](https://figshare.com/s/0c4e2b72b4915f9fd077) for the 10 case studies described in Section 4.2.

## Prerequisites: ##
* Any linux distributions, preferably Ubuntu 18.04.5 LTS (Bionic Beaver)
* Apollo [prerequisites](https://github.com/UCI-SORA-LAB/apollo#prerequisites)
* Python3
* We made some changes to Apollo 6.0 to enable scenoRITA to run. Clone a copy of the modified version of Apollo found [here](https://github.com/UCI-SORA-LAB/apollo)
* Follow the instructions found [here](https://github.com/UCI-SORA-LAB/apollo/tree/automation/docs/demo_guide) to start and build apollo.
* Inside the Apollo docker, run the following script to clone scenoRITA and install all its dependencies:
```
./scenoRITA.sh
```
* Run the following commands to start dreamview and the needed modules:
```
bash /apollo/scripts/bootstrap.sh start
bash /apollo/automation/auxiliary/modules/start_modules.sh
source /apollo/cyber/setup.bash
```

## Using scenoRITA: ##
* To run scenoRITA from end-to-end, change directory to `scenario_generator`:
``` 
cd scenario_generator/
```
* Modify `scenoRITA_mut.py` (e.g., No. of generations, domain specific constraints etc). Then run the framework using the following command:
``` 
bazel run scenoRITA_mut 
```

## How to cite scenoRITA: ##
Authors of scientific papers using scenoRITA are encouraged to cite the following paper.
```
@misc{scenorita2022,
      title={scenoRITA: Generating Less-Redundant, Safety-Critical and Motion Sickness-Inducing Scenarios for Autonomous Vehicles},
      author={Sumaya Almanee, Xiafa Wu, Yuqi Huai, Qi Alfred Chen and Joshua Garcia},
      year={2022},
      eprint={2112.09725},
      archivePrefix={arXiv},
      primaryClass={cs.SE}
}
```

