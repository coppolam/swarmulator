#!/bin/bash

#echo The script that creates the header file (*.h) and source (*.cpp).

if [ "$1" = "" ]; then
	echo "Please specify the name of the controller.\n";
	exit 1;
fi

folder=../sw/simulation/controllers;

if [ -f $folder/$1".h" ] || [ -f $folder/$1".cpp" ]; then
	echo "A controller with this name already exists. Do you want to overwrite it? [yes]";
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
	print "#include \"controller.h\"\n" > hN;
	print "using namespace std;\n" > hN
	print "class "n": public Controller" > hN;
	print "{" > hN;
	print "public:" > hN;
	print "	"n"():Controller(){};" > hN;
	print "	virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);" > hN;
	print "};\n" > hN;
	print "#endif /*"N"_H*/" >> hN;

	cN = p"/"n".cpp";
	print "Creating file implementation: "n".cpp ...";
	print "#include \""n".h\"" > cN;
	print "#include \"draw.h\"\n" > cN;
	print "void "n"::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)\n{" > cN;
	print "  /*** Put your controller here ***/\n}" >> cN;
}'

exit 0;