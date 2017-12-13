# Bash script to run a simulation multiple times and save all logs in a folder
cd $SWARMULATOR_HOME

for (( i = 1; i <= $1; i++ )); do
	echo "Running trial $i out of $1 for a simulation with $2 agents"
  ./swarmulator $2 1
done
