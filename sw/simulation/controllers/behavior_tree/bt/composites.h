#ifndef MYCOMPOSITES_H
#define MYCOMPOSITES_H

#include "behaviour.h"

namespace BT
{
typedef std::vector<node *> nodes;

// composite behaviours
class composite : public node
{
public:
  void reset();
  void addChild(node *child, int position = -1);
  nodes setChildren(nodes children);
  node *getChild(size_t k_child);
  nodes getChildren();
  size_t kChildren();
  node *replaceChild(node *task, size_t k_child, size_t isNEW = 0);
  bool canFit(node *task, size_t parallel_found);
  void deleteChild(size_t k_child);

  virtual ~composite()
  {
    for (m_current = m_children.begin(); m_current != m_children.end(); m_current++) {
      delete *m_current;
    }
    m_children.clear();
  }

protected:
  composite(std::string Name, std::string Type, std::string Function) :
    node(Name, Type, Function)
  {
  }

  virtual void onInitialise() { m_current = m_children.begin(); }

  nodes m_children;
  nodes::iterator m_current;
};

// sequence behaviours
class sequence : public composite
{
protected:
  BT_Status update(blackboard *BLKB);
public:
  sequence():
    composite("sequence", "composite", "sequence")
  {
  }
};

// Selector behaviours
class selector : public composite
{
protected:
  BT_Status update(blackboard *BLKB);
public:
  selector():
    composite("selector", "composite", "selector")
  {
  }
};

// Parallel behaviours
class parallel : public composite
{
protected:
  BT_Status update(blackboard *BLKB);
public:
  parallel():
    composite("parallel", "composite", "parallel")
  {
  }
};

composite *getComposite(std::string composite);
//composite *getComposite(std::string composite, std::vector<double>* values);
composite *getComposite(size_t func);
composite *getComposite();

void showTree(composite *task, size_t tab);
}
#endif // MYCOMPOSITES_H
