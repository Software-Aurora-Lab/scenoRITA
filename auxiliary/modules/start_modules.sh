#!/bin/bash

bash /apollo/scripts/prediction.sh start

bash /apollo/scripts/planning.sh start

bash /apollo/scripts/routing.sh start

# nohup cyber_launch start /apollo/modules/planning/launch/planning.launch
# nohup cyber_launch start /apollo/modules/prediction/launch/prediction.launch
# nohup cyber_launch start /apollo/modules/routing/launch/routing.launch
