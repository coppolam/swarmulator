#ifndef COMPOSITES_H
#define COMPOSITES_H

#include "node.h"

namespace BT
{
typedef std::vector<node *> nodes;

/**
 * Composite behaviors
 *
 */
class composite : public node
{
public:
  /**
   * @brief
   *
   */
  void reset();

  /**
   * @brief
   *
   * @param child
   * @param position
   */
  void addChild(node *child, int position = -1);

  /**
   * @brief Set the Children object
   *
   * @param children
   * @return nodes
   */
  nodes setChildren(nodes children);

  /**
   * @brief Get the Child object
   *
   * @param k_child
   * @return node*
   */
  node *getChild(size_t k_child);

  /**
   * @brief Get the Children object
   *
   * @return nodes
   */
  nodes getChildren();

  /**
   * @brief
   *
   * @return size_t
   */
  size_t kChildren();

  /**
   * @brief
   *
   * @param task
   * @param k_child
   * @param isNEW
   * @return node*
   */
  node *replaceChild(node *task, size_t k_child, size_t isNEW = 0);

  /**
   * @brief
   *
   * @param task
   * @param parallel_found
   * @return true
   * @return false
   */
  bool canFit(node *task, size_t parallel_found);

  /**
   * @brief
   *
   * @param k_child
   */
  void deleteChild(size_t k_child);

  /**
   * @brief Destroy the composite object
   *
   */
  virtual ~composite()
  {
    for (m_current = m_children.begin(); m_current != m_children.end(); m_current++) {
      delete *m_current;
    }
    m_children.clear();
  }

protected:
  /**
   * @brief Construct a new composite object
   *
   * @param Name
   * @param Type
   * @param Function
   */
  composite(std::string Name, std::string Type, std::string Function) :
    node(Name, Type, Function)
  {
  }

  /**
   * @brief
   *
   */
  virtual void onInitialise() { m_current = m_children.begin(); }

  nodes m_children;
  nodes::iterator m_current;
};

/**
 * @brief Sequence behaviors
 *
 */
class sequence : public composite
{
protected:
  /**
   * @brief
   *
   * @param BLKB
   * @return BT_Status
   */
  BT_Status update(blackboard *BLKB);
public:
  /**
   * @brief Construct a new sequence object
   *
   */
  sequence():
    composite("sequence", "composite", "sequence")
  {
  }
};

/**
 * @brief Selector behaviors
 *
 */
class selector : public composite
{
protected:
  /**
   * @brief
   *
   * @param BLKB
   * @return BT_Status
   */
  BT_Status update(blackboard *BLKB);
public:
  selector():
    composite("selector", "composite", "selector")
  {
  }
};

/**
 * @brief Parallel behaviors
 *
 */
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

/**
 * @brief Get the Composite object
 *
 * @param composite
 * @return composite*
 */
composite *getComposite(std::string composite);
//composite *getComposite(std::string composite, std::vector<double>* values);

/**
 * @brief Get the Composite object
 *
 * @param func
 * @return composite*
 */
composite *getComposite(size_t func);

/**
 * @brief Get the Composite object
 *
 * @return composite*
 */
composite *getComposite();

/**
 * @brief
 *
 * @param task
 * @param tab
 */
void showTree(composite *task, size_t tab);
}
#endif // COMPOSITES_H
