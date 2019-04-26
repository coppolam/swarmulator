# Bash script to run a simulation multiple times and save all logs in a folder
# This script expects swarmulator to have a kill-switch activated somewhere,
# although manual quitting (pressing the 'q' key) is also fine.
# $1 = number of trials
# $2 = number of agents

cd ..

Make logs directory
min=$(date +%Y-%m-%d-%T);
d=logs/batchtest_$min
mkdir $d

for (( i = 1; i <= $1; i++ )); do
	
	# Bash text
	echo "Running trial $i out of $1 for a simulation with $2 agents"

	# Run code
	md=$(date +%Y-%m-%d-%T);
	ma=$(date +%Y-%m-%d-%T -d "+1 seconds") #backup time
	./swarmulator $2
	sleep 1 # Give it some time to close

	# Move latest log to directory
	fn=$(ls -t logs| head -n1)
	if mv -f -- logs/log1_$md.txt $d/log_$i.txt; then
		echo "Successfully moved file"
	else
		#if it doesn't work it's because it skipped a second
		echo "Trying again........!" 
		mv -f -- logs/log_$ma.txt $d/log_$i.txt
	fi

done
