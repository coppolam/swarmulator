#include "composites.h"
#include "conditions.h"

#include <assert.h>

namespace BT
{

size_t KCOMP = 1;         // total number of composites

// ************************************************************************
// General composite methods

// replaces a child, returns replaced child
bool composite::canFit(node *task, size_t parallel_found)
{
  // check parallel for concurrency issues, if apparent do not replace child
  if (parallel_found || (task->type.compare("action") == 0 && this->function.compare("parallel") == 0)) {
    for (size_t i = 0; i < m_children.size(); i++) {
      if (this->m_children[i]->type.compare("composite") == 0) {
        if (!((composite *)this->m_children[i])->canFit(task, 1)) {
          return false;
        }
      } else if (this->m_children[i]->type.compare("action") == 0
                 && this->m_children[i]->function.compare(task->function) != 0) {
        return false;
      }
    }
  }
  return true;
}

// add new child to back of vector
void composite::addChild(node *child, int position /* = -1 */)
{
  // check parallel for concurrency issues
  if (!this->canFit(child, 0)) {
    if (child->type.compare("composite") == 0) {
      delete (composite *)child;
    } else {
      delete child;
    }
    child = NULL;
    return;
  }

  if (position == -1) {
    m_children.push_back(child);
  } else {
    m_children.insert(m_children.begin() + position, child);
  }
}

// change children in composite, any old children returned
nodes composite::setChildren(nodes children)
{
  nodes temp = m_children;
  m_children = children;
  return temp;
}

// returns children of composite
nodes composite::getChildren()
{
  return m_children;
}
// returns child from composite
node *composite::getChild(size_t k_child)
{
  return m_children[k_child];
}
// returns number children in composite
size_t composite::kChildren()
{
  return m_children.size();
}

// replaces a child, returns replaced child
node *composite::replaceChild(node *task, size_t k_child, size_t isNew)
{
  // check parallel for concurrency issues, if apparent do not replace child
  if (!this->canFit(task, 0)) {
    if (isNew) {
      if (task->type.compare("composite") == 0) {
        delete (composite *)task;
      } else {
        delete task;
      }
    }
    return NULL;
  }

  node *old_child = m_children[k_child];
  m_children[k_child] = task;
  return old_child;
}

// delete child from composite
void composite::deleteChild(size_t k_child)
{
  if (m_children[k_child]->type.compare("composite") == 0) {
    if (m_children[k_child] != NULL) {
      delete (composite *)m_children[k_child];
    }
  } else {
    if (m_children[k_child] != NULL) {
      delete m_children[k_child];
    }
  }
  m_children.erase(m_children.begin() + k_child);
}

// reset status of all nodes in tree
void composite::reset()
{
  for (nodes::iterator it = m_children.begin(); it != m_children.end(); it++) {
    (*it)->m_eStatus = BH_INVALID;
    (*it)->tick_counter = 0;
    if ((*it)->type.compare("composite") == 0) {
      ((composite *)(*it))->reset();
    }
  }
}

// ************************************************************************
// sequence methods
BT_Status sequence::update(blackboard *BLKB)
{
  // keep running while child behaviour is running
  while (true) {
    BT_Status s = (*m_current)->tick(BLKB);
    // if child succeeds, or keeps running, do the same
    if (s != BH_SUCCESS) {
      return s;
    }

    // hit the end of the array
    if (++m_current == m_children.end()) {
      return BH_SUCCESS;
    }
  }
  std::cerr << "Unexpected loop exit" << std::endl;
  assert(1);
  return BH_INVALID;
}

// ************************************************************************
// selector methods
BT_Status selector::update(blackboard *BLKB)
{
  // keep running while child behaviour is running
  while (true) {
    BT_Status s = (*m_current)->tick(BLKB);

    // if child succeeds, or keeps running, do the same
    if (s != BH_FAILURE) {
      return s;
    }

    // hit the end of the array
    if (++m_current == m_children.end()) {
      return BH_FAILURE;
    }
  }
  std::cerr << "Unexpected loop exit" << std::endl;
  assert(1);
  return BH_INVALID;
}

// ************************************************************************
// parallel methods
// Run all children and return highest priority behaviour
BT_Status parallel::update(blackboard *BLKB)
{
  BT_Status myStatus = BH_INVALID;
  // keep running while child behaviour is running
  while (true) {
    BT_Status s = (*m_current)->tick(BLKB);

    // Store highest priority status
    if (s > myStatus) {
      myStatus = s;
    }

    // hit the end of the array
    if (++m_current == m_children.end()) {
      return myStatus;
    }
  }
  std::cerr << "Unexpected loop exit" << std::endl;
  assert(1);
  return BH_INVALID;
}

// ***************************************************************
// ***************************************************************
// Add all composites to the if else if list below
composite *getComposite(std::string Composite)
{
  composite *task;
  if (Composite.compare("sequence") == 0) {
    task = (composite *)new sequence;
  } else if (Composite.compare("selector") == 0) {
    task = (composite *)new selector;
  } else if (Composite.compare("parallel") == 0) {
    task = (composite *)new parallel;
  } else {
    task = NULL;
    std::cerr << "Something is really wrong in :BT::composite* getComposite(std::string composite)" << std::endl;
    assert(1);
  }

  return task;
}
/*
// Add all composites to the if else if list below
composite *getComposite(std::string Composite, std::vector<double>* values)
{
  composite* task;
  if (Composite.compare("sequence") == 0)
  {
    task = new sequence;
  }
  else if (Composite.compare("selector") == 0)
  {
    task = new selector;
  }
  else
  {
    task = NULL;
    std::cerr << "Something is really wrong in :BT::composite* getComposite(std::string composite, std::vector<double>* values)"<<std::endl;
    assert(1);
  }

  return task;
}
*/
// Add all actions to the if else if list below
composite *getComposite(size_t func)
{
  composite *task;
  switch (func) {
    case 0:
      task = new sequence;
      break;
    case 1:
      task = new selector;
      break;
    case 2:
      task = new parallel;
      break;
    default:
      std::cerr << "ERROR in getComposite(unsigned int func): number of composites out of bounds" << std::endl;
      assert(1);
  }
  return task;
}

// get random composite
composite *getComposite()
{
  return getComposite(rand() % KCOMP);
}

void showTree(composite *task, size_t tab)
{
  nodes children = task->getChildren();
  for (std::size_t i = 0; i < task->kChildren(); i++) {
    for (size_t j = 0; j < tab; j++) {
      std::cout << '\t';
    }

    if ((children[i])->type.compare("composite") == 0) {
      std::cout << (children[i])->function << std::endl;
      showTree((composite *)children[i], tab + 1);
    } else {
      std::cout << (children[i])->type << ": " << (children[i])->function;
      for (size_t k = 0; k < children[i]->vars.size(); k++) {
        std::cout << " " << (children[i])->vars[k];
      }
      std::cout << std::endl;

    }
  }
}

}
