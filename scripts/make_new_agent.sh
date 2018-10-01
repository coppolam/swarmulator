# Bash script to create the header file (*.h) and source (*.cpp) of a new agent
# $1: Name of the agent

if [ "$1" = "" ]; then
	echo "Please specify the name of the agent.\n";
	exit 1;
fi

folder=../sw/simulation/agents;

if [ -f $folder/$1".h" ] || [ -f $folder/$1".cpp" ]; then
	echo "An agent with this name already exists. Do you want to over write it? [yes]";
	read option;
	if [ "$option" != "yes" ]; then
		exit 0;
	fi
fi

str=$1
c=`echo $str | awk '{print toupper($0)}'`

awk -vn=$1 -vN=$c -vp=$folder 'BEGIN{
	hN = p"/"n".h";
	print "Creating header: "n".h ...";
	print "#ifndef "N"_H" > hN;
	print "#define "N"_H\n" > hN;
	print "#include <vector>" > hN;
	print "#include <stdio.h>" > hN;
	print "#include <iostream>" > hN;
	print "#include \"agent.h\"\n" > hN;
	print "using namespace std;\n" > hN
	print "class "n": public Agent" > hN;
	print "{" > hN;
	print "public:" > hN;
	print "	"n"(int i, vector<float> state, float tstep);" > hN;
	print "	void state_update();" > hN;
	print "	void animation();" > hN;
	print "};\n" > hN;
	print "#endif /*"N"_H*/" >> hN;

	cN = p"/"n".cpp";
	print "Creating file implementation: "n".cpp ...";
	print "#include \""n".h\"" > cN;
	print "#include \"draw.h\"\n" > cN;
	print n"::"n"(int i, vector<float> state, float tstep)\n{" >> cN;
	print "  state = s;\n  ID = i;\n  dt = tstep;\n  orientation = 0.0;" > cN;
	print "}\n" >> cN;
	print "void "n"::state_update()\n{ "> cN;
	print "  float v_x, v_y;"> cN;
	print "  controller.get_velocity_command(ID, v_x, v_y);" > cN;
	print "  /*** Include your model here ***/ \n}\n" > cN;
	print "void "n"::animation()\n{" > cN
	print "  draw d;" > cN
	print "  /*** Draw your agent here ***/\n}" > cN;
}'

exit 0;