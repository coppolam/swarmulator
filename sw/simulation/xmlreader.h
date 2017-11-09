#ifndef XMLREADER_H
#define XMLREADER_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <pugixml/pugixml.hpp>
#include <vector>
#include <thread>         // std::thread
#include <mutex>

#include "agent.h"
#include "particle.h"
#include "main.h"
#include "terminalinfo.h"

using namespace std;
using namespace pugi;

class XMLreader {
	pugi::xml_document doc;
	terminalinfo ti;
public:
	XMLreader(string str);
	~XMLreader();
	void runthrough(string str);
};

#endif /*XMLREADER_H*/
