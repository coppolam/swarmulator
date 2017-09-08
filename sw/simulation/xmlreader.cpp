#include "xmlreader.h"

XMLreader::XMLreader(string str)
{
	// Load XML file
	xml_parse_result result = doc.load_file(str.c_str());
	if (result.status) // False (0) = No errors
	{
		debug_msg ("Could not load parameters file.");
		program_running = false;
	}
};

XMLreader::~XMLreader(){};

float updatefreq; // TODO: just a sample, so remove this and put it somewhere else with the full list.
void XMLreader::runthrough(string str)
{	
	// Load the simulation node
	xml_node node = doc.child(str.c_str());

	// Go through all its children and parse them
	for (node = node.first_child(); node; node = node.next_sibling())
    {
        for (xml_attribute attr = node.first_attribute(); attr; attr = attr.next_attribute())
        {
            std::string attrName = attr.name();
        	if (!attrName.compare("name")) // We only process variable names
        	{
        		string varname = attr.value();
        		if (!varname.compare("window_width")) // TODO: hardcode all the variable names!
        		{
        			attr = attr.next_attribute();
	           		window_width = stof(attr.value());//std::cout << " " << attrName << "=" << attr.value() << std::endl;
        		}
                debug_msg("In " + str + " node, loaded " + varname + "=" + attr.value());
        	}
        }
    }

}