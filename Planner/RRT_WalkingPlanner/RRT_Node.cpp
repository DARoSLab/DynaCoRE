#include "RRT_Node.h"
#include <iostream>

//using namespace std;

const double RRT_Node::NULL_COST = 0.0;


RRT_Node::RRT_Node()
{ RRT_Node(0.,0.,0.); }

//nullptr{ RRT_Node(0.,0.,0.); }

// Copy constructor -- NOT WORKING -- TODO
// RRT_Node::RRT_Node(const RRT_Node& n) // copy constructor does not copy cost_to_root_ or parent_
// {
//   std::cout << "Copy constructor invoked" << std::endl;
//   RRT_Node(n.x_, n.y_, n.theta_);
// }

RRT_Node::RRT_Node(const double x, const double y, const double theta)
{
  x_ = x;
  y_ = y;
  theta_ = remainder( theta , 2.*M_PI );
  cost_to_parent_ = NULL_COST;
  parent_ = NULL;
}

RRT_Node::~RRT_Node()
{
  // std::cout << "Destructing node ... ";
  // print_node();
} // does not delete parent_ because parent_ could still be a valid part of the tree


double RRT_Node::get_cost_to_root() const
{
  // Recursive Implementation:
  // if (parent_ == NULL)
  //   return 0.;
  // else
  //   return cost_to_parent_ + parent_->get_cost_to_root;

  // While Loop Implementation:
  RRT_Node* p = parent_;
  double total(cost_to_parent_);

  while (p != NULL)
  {
    total += p->cost_to_parent_;
    p = p->parent_;
  }

  return total;
}

std::string RRT_Node::to_string() const
{
  std::string s;
  std::stringstream ss;

  ss << std::fixed << std::setprecision(2) << x_;
  s = "x = " + ss.str();

  ss.str( std::string() );
  ss.clear();
  ss << std::fixed << std::setprecision(2) << y_;
  s += ", y = " + ss.str();

  ss.str( std::string() );
  ss.clear();
  ss << std::fixed << std::setprecision(2) << theta_;
  s += ", theta = " + ss.str();

  ss.str( std::string() );
  ss.clear();
  ss << std::fixed << std::setprecision(2) << cost_to_parent_;
  s += ", c2p = " + ss.str();

  return s;
}

void RRT_Node::print_node() const
{
  std::cout << to_string() << std::endl;
}

void RRT_Node::save_node(std::string name) const
{
  sejong::Vector node_data(3);
  node_data << x_, y_, theta_;
  sejong::saveVector(node_data, name);
}

void RRT_Node::save_node_2(std::string name) const
{
  sejong::Vector node_data(4);
  node_data << x_, y_, theta_, get_cost_to_root();
  sejong::saveVector(node_data, name);
}

bool RRT_Node::operator ==(const RRT_Node& n) const
{
  return (x_ == n.x_) && (y_ == n.y_) && (theta_ == n.theta_);
}

bool RRT_Node::operator !=(const RRT_Node& n) const
{ return !( (*this) == n );  }
