# Bash script to run a simulation multiple times and save all logs in a folder
# This script expects swarmulator to have a kill-switch activated
cd $SWARMULATOR_HOME

# make
# min=$(date +%Y-%m-%d-%T);
# mkdir logs/batchtests_$min

min=$(date +%Y-%m-%d-%T);
d=logs/batchtest_$min
mkdir $d

for (( i = 1; i <= $1; i++ )); do
	echo "Running trial $i out of $1 for a simulation with $2 agents"
	date
	./swarmulator $2 1
	fn=$(ls -t logs| head -n1)
	mv -f -- "logs/$fn" $d/$fn
done
