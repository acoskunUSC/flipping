#!/bin/bash

cd ../../alite-deconflictiontools

mvn clean install -DskipTests

cd ../rmtrack

mvn clean install

cd experiment

mkdir instances/ 2> /dev/null

rm -f instances/*

rm -f denv.xml

rm -f results.txt

input=env

experiment=denv

radiusForGraph=20

gridedgelen=53

dispersion=`echo "import math;print(int(math.ceil($gridedgelen*math.sqrt(2))))" | python`

connectionradius=120

java -cp ../target/robust-tracking-1.0-SNAPSHOT-jar-with-dependencies.jar tt.jointeuclid2ni.probleminstance.generator.TriangulationGenerator -problemfile $input.xml -bodyradius $radiusForGraph -dispersion $dispersion -connectionradius $connectionradius -generateinfrastructure -outfile $experiment.xml

maxtime=600000

agents="5 10 15 20"

maxspeed="0.05"

timestep=`echo "import math;print(int(math.ceil($gridedgelen/(2*$maxspeed))))" | python`

radiusForGenerating=15

echo "Preparing instanceset instances. Will use timestep $timestep."

rm instances/* 2> /dev/null

instance=0
for nagents in $agents
do	
	for seed in {1..20}
        do
            let instance=instance+1
            # create a problem instance file
            instancename="$instance"
            instancefile=instances/$instancename.xml

            ## ConflictGenerator    
            java -XX:+UseSerialGC -cp ../target/robust-tracking-1.0-SNAPSHOT-jar-with-dependencies.jar -Dlog4j.configuration="file:$PWD/log4j.custom" tt.jointeuclid2ni.probleminstance.generator.GenerateEAInstance -env $experiment.xml -nagents $nagents -radius $radiusForGenerating -maxspeed $maxspeed -seed $seed -sgnooverlap -separateRobotsExperiment1 -outfile $instancefile  

            echo Finished instance no $instance. Agents: $nagents. Seed: $seed.
        done
done

echo Done. Created $instance instances at $experiment environment. Instances stored in instances.

for fileIndex in {1..80}
do
	sed -i '4i<disturbances><polygon probability="0.85">650,1950 1050,1950 600,2400 200,2400</polygon></disturbances>' instances/$fileIndex.xml
done

timestep=`echo "import math;print(int(math.ceil($gridedgelen/(2*$maxspeed))))" | python`

timestep2=540

for fileIndex in {1..80}
do
	echo Running instance no $fileIndex.
	java -enableassertions -XX:+UseSerialGC -Xms1g -Xss1g -Xmx8g -jar ../target/robust-tracking-1.0-SNAPSHOT-jar-with-dependencies.jar -method ALLMETHODS -problemfile instances/$fileIndex.xml -timestep $timestep2 -maxtime 6000000 -drop 5 -avoidDeadlock >> results.txt
done

rm -f plots/*

octave readnplot.m


