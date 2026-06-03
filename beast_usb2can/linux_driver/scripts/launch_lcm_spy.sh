#!/bin/bash
#sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev enp3s0
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd ${DIR}/../lcm_types/java
export CLASSPATH=${DIR}/../lcm_types/java/my_types.jar
pwd
lcm-spy
