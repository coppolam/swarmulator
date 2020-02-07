
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>

#include "btFile.h"

namespace BT
{

composite *loadFile(const char *file)
{
  node *root;
  nodes tasks;

  std::string line;
  std::ifstream myfile;

  size_t i_tab, currentTab = 0;

  myfile.open(file);
  if (myfile.is_open()) {
    //getfileInfo;

    // get root node
    std::getline(myfile, line);
    line.erase(std::remove(line.begin(), line.end(), ' '), line.end());   // remove white spaces
    root = decodeLine(line);
    tasks.push_back(root);

    // read rest of BT
    while (std::getline(myfile, line)) {
      line.erase(std::remove(line.begin(), line.end(), ' '), line.end());   // remove white spaces
      // determine depth
      i_tab = std::count(line.begin(), line.end(), '\t');

      // new sub task
      if (i_tab > currentTab) {
        tasks.push_back(newTask(tasks.back(), line));
        currentTab = i_tab;
        continue; // read next line
      }

      // end of sub tasks, move to new task
      else if (i_tab < currentTab) {
        while (i_tab != currentTab) {
          tasks.pop_back();
          --currentTab;
        }
      }
      // add new task
      tasks.pop_back();
      tasks.push_back(newTask(tasks.back(), line));
      //continue to next line
    }
    myfile.close();
  } else {
    printf("ERROR! Unable to open behaviour tree file:\n%s\n\n", file);
    exit(-1);
  }
  return (composite *) root;
}

bool saveFile(const char *file, composite *task)
{
  std::ofstream myfile(file);

  if (myfile.is_open()) {
    myfile << codeLine(task) << std::endl;
    if (task->type.compare("composite") == 0) {
      writeComposite(myfile, task, 1);
    }

    myfile.close();
  } else {
    std::cout << "Unable to open file" << std::endl;
    return false;
  }
  return true;
}

void writeComposite(std::ofstream &myfile, composite *parent, size_t tabs)
{
  node *task;
  nodes children;

  children = parent->getChildren();

  if (myfile.is_open()) {
    for (size_t i = 0; i < children.size(); i++) {
      task = children[i]; // write last child first
      for (size_t i = 0; i < tabs; i++) {
        myfile << '\t';
      }

      // print task line
      myfile << codeLine(task) << std::endl;

      if (task->type.compare("composite") == 0) {
        writeComposite(myfile, (composite *) task, tabs + 1);
      }
    }
  }
}

/*
  Create new child and add to parent composite
 */
node *newTask(node *Parent, std::string line)
{
  // decode line and create new task
  composite *parent = (composite *)Parent;
  node *task = decodeLine(line);

  parent->addChild(task);

  return task;
}

/*
  Decode BT task from HTML style string line
 */
node *decodeLine(std::string line)
{
  // std::stringstream ss(line);
  // std::string current;
  node *task;

  std::string param, function, vars;
  std::vector<double> values;

  // temporary place holders
  std::size_t start, end, pos1, pos2;

  // find function type
  start = line.find("<function>") + 10;
  end = line.find_first_of("<", start);
  function = line.substr(start, end - start);

  // find task type
  start = line.find("<BTtype>") + 8;
  end = line.find_first_of("<", start);
  param = line.substr(start, end - start);

  // find task variables
  start = line.find("<vars>") + 6;
  end = line.find_first_of("<", start);
  vars = line.substr(start, end - start);

  if ((start - end) > 0) {
    pos1 = 0; pos2 = 0;
    while (pos1 < vars.length()) {
      if (vars.find(",", pos1) == std::string::npos) {
        pos2 = vars.length();
      } else {
        pos2 = vars.find(",", pos1);
      }
      values.push_back(atof((vars.substr(pos1, pos2 - pos1)).c_str()));
      pos1 = pos2 + 1;
    }
  }

  if (param.compare("action") == 0) {
    task = getAction(function, values);
  } else if (param.compare("condition") == 0) {
    task = getCondition(function, values);
  } else if (param.compare("composite") == 0) {
    task = getComposite(function);
  } else {
    std::cout << "Unidentified BTtype: " << param << std::endl;
  }

  /*
    else
    {
        if (param.compare("Action") == 0)
            task = getAction( function );
        else if (param.compare("Condition") == 0)
            task = getCondition( function );
        else if (param.compare("composite") == 0)
            task = getComposite( function );
        else
            std::cout << "Unidentified BTtype: "<< param<<std::endl;
    }
   */
  // find name of task
  start = line.find("<name>") + 6;
  end = line.find_first_of("<", start + 1);
  param = line.substr(start, end - start);
  task->name = param;

  return task;
}

/*
  Code BT task to string line
 */
std::string codeLine(node *task)
{
  std::stringstream line;

  size_t k_vars = task->vars.size();

  line << "<BTtype>";
  line << task->type;
  line << "<function>";
  line << task->function;
  line << "<vars>";
  for (size_t i = 0; i < k_vars; i++) {
    if (i > 0) {
      line << "," << task->vars[i];
    } else {
      line << task->vars[i];
    }
  }
  line << "<name>";
  line << task->name;
  line << "<endl>";

  return line.str();
}

}
