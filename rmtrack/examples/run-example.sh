#!/bin/bash

instance=$1
method=$2
showvis=$3
avoidDeadlock=$4

java -enableassertions -XX:+UseSerialGC -Xss4m -jar ../target/robust-tracking-1.0-SNAPSHOT-jar-with-dependencies.jar -method $method -problemfile $instance.xml -timestep 800 -maxtime 600000 -dprob 10 $showvis $avoidDeadlock
