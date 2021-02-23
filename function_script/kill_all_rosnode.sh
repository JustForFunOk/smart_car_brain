#!/bin/bash

function kill_process()
{
    # kill with ^c
    process_name=$1
    echo 'killing '$process_name
    process_pid=$(ps -ef | grep $process_name | awk '{print $2}')  # get process pid number
    if [[ ${process_pid} != 0 ]];then
        kill -2 ${process_pid}
    fi

    # forced kill
    process_name=$1
    echo 'forced killing '$process_name
    process_pid=$(ps -ef | grep $process_name | awk '{print $2}')  # get process pid number
    if [[ ${process_pid} != 0 ]];then
        kill -9 ${process_pid}
    fi
}

kill_process ros
kill_process chassis
