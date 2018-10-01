# Bash script to recursively fix all cpp and h files to the preferred formatting
# It is best to run this before committing code

./fix_code_style.sh "../sw/*.cpp";
./fix_code_style.sh "../sw/*.h";