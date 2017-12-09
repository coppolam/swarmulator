#include "xmlreader.h"

XMLreader::XMLreader(string str)
{
  // Load XML file
  xml_parse_result result = doc.load_file(str.c_str());
  if (result.status) { // False (0) = No errors
    ti.debug_msg("Could not load parameters file.");
    program_running = false;
  }
};

XMLreader::~XMLreader() {};

float updatefreq; // TODO: just a sample, so remove this and put it somewhere else with the full list.
void XMLreader::runthrough(string str)
{
  // Load the simulation node
  xml_node node = doc.child(str.c_str());

  // Go through all its children and parse them
  for (node = node.first_child(); node; node = node.next_sibling()) {
    for (xml_attribute attr = node.first_attribute(); attr; attr = attr.next_attribute()) {
      std::string attrName = attr.name();
      if (!attrName.compare("name")) { // We only process variable names
        string varname = attr.value();
        if (!varname.compare("simulation_updatefreq")) {    // TODO: hardcode all variables
          simulation_updatefreq = stof(attr.next_attribute().value());
        } else if (!varname.compare("simulation_realtimefactor")) { // TODO: hardcode all variables
          simulation_realtimefactor = stof(attr.next_attribute().value());
        }


        else if (!varname.compare("window_width")) { // TODO: hardcode all variables
          window_width = stof(attr.next_attribute().value());
        } else if (!varname.compare("window_height")) {
          window_height = stof(attr.next_attribute().value());
        } else if (!varname.compare("scale")) {
          scale = stof(attr.next_attribute().value());
        } else if (!varname.compare("mouse_drag_speed")) {
          mouse_drag_speed = stof(attr.next_attribute().value());
        } else if (!varname.compare("mouse_zoom_speed")) {
          mouse_zoom_speed = stof(attr.next_attribute().value());
        } else if (!varname.compare("animation_updatefreq")) {
          animation_updatefreq = stof(attr.next_attribute().value());
        } else if (!varname.compare("visible_centroid")) {
          visible_centroid = stof(attr.next_attribute().value());
        }

        else if (!varname.compare("logger_updatefreq")) {
          logger_updatefreq = stof(attr.next_attribute().value());
        }

        ti.info_msg("In " + str + " node, loaded " + varname + "=" + attr.next_attribute().value());
      }
    }
  }

}