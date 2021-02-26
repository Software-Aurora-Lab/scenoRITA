## Setup

If the planning module pauses and complains “Failed to updated reference line after rerouting”, try the hard-reset approach as following.

1. Implement a function that performs hard reset
```
void OnLanePlanning::HardReset() {
	if (reference_line_provider_) {
		reference_line_provider_->Stop();
	}
	planner_->Stop();
	injector_->frame_history()->Clear();
	injector_->history()->Clear();
	injector_->planning_context()->mutable_planning_status()->Clear();
	last_routing_.Clear();
	injector_->ego_info()->Clear();

	AERROR << "PLANNING MODULE HARD RESET";

	Init(config_);
}
```

2. Call the function after the error is triggered, which is located at `OnLanePlanning::RunOnce` from `modules/planning/on_lane_planning.cc` .
```
	  // early return when reference line fails to update after rerouting
		if (failed_to_update_reference_line) {
			std::string msg("Failed to updated reference line after rerouting.");
			AERROR << msg;
			ptr_trajectory_pb->mutable_decision()
					->mutable_main_decision()
					->mutable_not_ready()
					->set_reason(msg);
			status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
			ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
			FillPlanningPb(start_timestamp, ptr_trajectory_pb);
			GenerateStopTrajectory(ptr_trajectory_pb);
			
			// Call the hard reset function here
			HardReset();
			// Call the hard reset function here

			return;
		}
```

To enable SimControl by default when starting Dreamview, Insert `Start();` by the end of `SimControl::SimControl` at `modules/dreamview/backend/sim_control/sim_control.cc `

```
	SimControl::SimControl(const MapService* map_service)
		: map_service_(map_service),
		  node_(cyber::CreateNode("sim_control")),
		  current_trajectory_(std::make_shared<ADCTrajectory>()) {
	  InitTimerAndIO();
	  // start SimControl on default
	  Start();
	}
```

## Running the Scenario Simulation

To run the simulation script, issue command `bazel run run_automation -- arguments`.

### Arguments

Arguments available to use:

- `input_path`: the path to the directory containing the record files you wish you test.
- `-rv`: the x and y values for start and end point of navigation, separated by commas and without spaces. Format it as: `initial_x,initial_y,destination_x,destination_y`
   - For instance, `587120.7636406536,4141574.0292906095,587078.7256180799,4141641.2485725204`.
- `-rc`: the path to the CSV files containing all the routing points.

When both routing value and routing CSV file are provided, the script uses the CSV file instead of the routing value. If neither routing value or routing CSV file is provided, the script will use the default routing value: `587120.7636406536,4141574.0292906095,587078.7256180799,4141641.2485725204`.
