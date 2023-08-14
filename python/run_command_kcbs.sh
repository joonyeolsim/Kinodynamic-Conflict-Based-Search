#!/bin/bash

# 환경 및 변수 설정
ENVIRONMENTS=("OpenEnv")
VARIABLES_1=(5 10 15 20 25 30)
COMMAND="../cmake-build-debug/bin/demo_MultiRobotRigidBodyPlanningWithControlsYaml"

# 각 환경과 변수를 사용하여 명령어 실행
for env in "${ENVIRONMENTS[@]}"; do
    for var1 in "${VARIABLES_1[@]}"; do
        for var2 in {0..49}; do
            "$COMMAND $env $var1 $var2"
        done
    done
done

