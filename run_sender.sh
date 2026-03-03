#!/bin/bash
source /opt/ros/humble/setup.bash

export LOG_LEVEL=2

./build/sender_node config.json
