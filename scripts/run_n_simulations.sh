# Bash script to run a simulation multiple times and save all logs in a folder
# This script expects swarmulator to have a kill-switch activated somewhere,
# although manual quitting (pressing the 'q' key) is also fine.

cd $SWARMULATOR_HOME

# Make logs directory
min=$(date +%Y-%m-%d-%T);
d=logs/batchtest_$min
mkdir $d

for (( i = 1; i <= $1; i++ )); do
	
	# Bash text
	echo "Running trial $i out of $1 for a simulation with $2 agents"
	date

	# Run code
	md=$(date +%Y-%m-%d-%T);
	./swarmulator $2 1
	
	# Move latest log to directory
	fn=$(ls -t logs| head -n1)
	mv -f -- logs/log_$md.txt $d/$fn

done
