#!/bin/bash

# 명령어 및 시간 제한 함수
execute_with_timeout() {
    local cmd="$1"
    $cmd &  # 명령어를 백그라운드에서 실행
    command_pid=$! 

    # 5분 = 300초
    sleep 300 &  # 여기서는 테스트 목적으로 5초로 설정했습니다.
    sleep_pid=$!

    # 먼저 끝나는 프로세스의 PID를 기다립니다.
    wait -n $command_pid $sleep_pid

    # 만약 명령어 프로세스가 아직 실행 중이면 timeout입니다.
    if kill -0 $command_pid 2>/dev/null; then
        kill -9 $command_pid  # 명령어를 강제 종료
        echo "Timeout! Command '$cmd' was forcibly terminated."
    else
        # Command finished by itself, let's check its status
        wait $command_pid
        cmd_status=$?
        if [ $cmd_status -ne 0 ]; then
            echo "Unexpected termination! Command '$cmd' exited with status $cmd_status."
        fi
    fi

    # sleep 프로세스 종료
    kill -9 $sleep_pid 2>/dev/null
    wait $sleep_pid 2>/dev/null
}

# 환경 및 변수 설정
ENVIRONMENTS=("OpenEnv" "ClutteredEnv")
VARIABLES_1=(5 10 15 20 25 30)
COMMAND="../cmake-build-debug/ST_CBS"

# 각 환경과 변수를 사용하여 명령어 실행
for env in "${ENVIRONMENTS[@]}"; do
    for var1 in "${VARIABLES_1[@]}"; do
        for var2 in {0..49}; do
            execute_with_timeout "$COMMAND $env $var1 $var2"
        done
    done
done

