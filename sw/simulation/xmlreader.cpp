#include "xmlreader.h"

XMLreader::XMLreader(string str)
{
	// Load XML file
	pugi::xml_parse_result result = doc.load_file(str.c_str());
	if (result.status) // False (0) = No errors
	{
		debug_msg ("Could not load parameters file.");
		program_running = false;
	}

		std::cout << "Load result: " << ", mesh name: " << doc.child("simulation").attribute("name").value() << std::endl;
};

XMLreader::~XMLreader(){};