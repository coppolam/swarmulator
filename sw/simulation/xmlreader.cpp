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

float updatefreq;
void XMLreader::runthrough(string str)
{	
	// Load the simulation node
	xml_node node = doc.child(str.c_str());
    std::cout << node.name() << std::endl;

	// Go through all its children and parse them
	for (node = node.first_child(); node; node = node.next_sibling())
    {
        for (xml_attribute attr = node.first_attribute(); attr; attr = attr.next_attribute())
        {
            std::string attrName = attr.name();
        	if (!attrName.compare("name")) // We only process variable names
        	{
        		string varname = attr.value();
        		if (!varname.compare("updatefreq")) // I have to hardcode all the variables!
        		{
        			attr = attr.next_attribute();
	           		updatefreq = stof(attr.value());//std::cout << " " << attrName << "=" << attr.value() << std::endl;
        		}
        	}
        }
    }
}