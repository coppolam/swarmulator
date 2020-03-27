# Bash script to recursively fix all cpp and h files to the preferred formatting
# It is best to run this before committing code

# Get the script directory so we can call it from anywhere
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
dir=${script_dir%/*} # Go to root swarmulator folder

# Run
${dir}/scripts/fix_code_style.sh "${dir}/sw/*.cpp";
${dir}/scripts/fix_code_style.sh "${dir}/sw/*.h";

echo "astyle formatted the code."