# Bash script to run parallel batches of simulations (it launches a batch every five seconds)
# Argument 1: How many batches
# Argument 2: How many simulations per batch
# Argument 3: How many agents in the simulation

for (( i = 1; i <= $1; i++ )); do
  ./run_n_simulations.sh $2 $3 & sleep 5
done
