#include "RRT_Planner.h"
#include <iostream>
#include <Eigen/Geometry>
#include "PIPM_Planner_ConfSpace.h"
#include "EnvironmentSetup/Obstacle.h"
#include "EnvironmentSetup/ObstacleBuilder.h"

#define SAVE_DATA


RRT_Planner::RRT_Planner(const sejong::Vector& search_boundary,
                         const RRT_Node& start, const RRT_Node& goal,
                         const std::vector<Obstacle>& obstacle_list,
                         RRT_Analytic_Solver * RRT_analytic_solver)
{
  analytic_solver_ = RRT_analytic_solver;
  _Init(search_boundary,start,goal,obstacle_list);
}

RRT_Planner::RRT_Planner(const sejong::Vector& search_boundary,
                         const RRT_Node& start, const RRT_Node& goal,
                         const std::vector<Obstacle>& obstacle_list)
{
  analytic_solver_ = NULL;
  _Init(search_boundary,start,goal,obstacle_list);
}


void RRT_Planner::_Init(const sejong::Vector& search_boundary,
                   const RRT_Node& start, const RRT_Node& goal,
                   const std::vector<Obstacle>& obstacle_list)

{
  max_turn_ = M_PI / 4.0;
  x_p_ = 0.2;
  safety_margin_ = 0.5;
  found_solution_ = false;
  rad = 1.;
  histogram_cost_compare_ = sejong::Vector::Zero(20);

  search_boundary_ = search_boundary;
#ifdef SAVE_DATA
  sejong::saveVector(search_boundary_, "axes");
#endif

  tree_.push_back(new RRT_Node(start));
#ifdef SAVE_DATA
  start.save_node_2("tree");
#endif
  goal_ = new RRT_Node(goal);

  obstacle_list_ = obstacle_list;

  std::cout << "[RRT Planner] constructed" << std::endl;

  srand((unsigned)time(NULL));
}


RRT_Planner::~RRT_Planner() {

  std::vector<RRT_Node*>::iterator it = tree_.begin();
  for (; it != tree_.end(); ++it )
    delete *it;

  if(!found_solution_)
    delete goal_;
}

bool RRT_Planner::build_RRT(const int& num_samples) {

  RRT_Node sample;
  RRT_Node* nearest_neighbor;
  std::vector<RRT_Node*> new_branch;
  for(int i(0); i < num_samples; i++)
  {
    if (i%20 == 0 || i == num_samples-1){
      sample = *goal_;
    } else{
      rand_sample(sample);
    }

    if(i%30 == 0){
      std::cout<<i<<"th : "<<std::endl;
      sample.print_node();
}

    get_nearest_neighbor_path(sample, nearest_neighbor, new_branch);

    prune_for_collisions(new_branch);

    append_path_to_tree(new_branch, nearest_neighbor);

    // there is actually a chance that we accidentally connect to the goal in the middle of a path
    // this condition would not know that that was the case if it did happen
    if ( *(new_branch.back()) == *goal_ )
    {
      //std::cout << "Found solution after " << i << " samples" << std::endl;
      delete goal_;
      found_solution_ = true;
      goal_ = new_branch.back();
      return true; // connected start to goal
      // can now use goal_ to trace path back to start
    }
  }
  return false;
}


void RRT_Planner::get_solution(std::vector<RRT_Node*>& solution) const
{
  if(found_solution_)
  {
    solution.clear();
    RRT_Node* n = goal_;

    while (n != NULL)
    {
      solution.push_back(n);
      n = n->parent_;
    }
    std::reverse(solution.begin(),solution.end());

#ifdef SAVE_DATA
    std::vector<RRT_Node*>::iterator it = solution.begin();
    for (; it != solution.end(); ++it){
      // std::cout << "Node " << i << ": " << "x = " << (*it)->x_ << ", y = " << (*it)->y_ << ", theta = " << (*it)->theta_ << ", cost = " << (*it)->cost_to_parent_ << std::endl;
      // (*it)->print_node();
      (*it)->save_node_2("soln");
    }
    ObstacleBuilder::save_obs_vectors(obstacle_list_, solution.back()->get_cost_to_root(), 0.2);
#endif
  }

  // sejong::Vector idx = sejong::Vector::LinSpaced(20,1,20);
  // sejong::Matrix hist(20,2);
  // hist << idx, histogram_cost_compare_;

  // sejong::pretty_print(hist, std::cout, "Histogram of solutions");
}


// Warning: this will delete the rest of the tree!
void RRT_Planner::shortcut_solution(const int num_iterations, std::vector<RRT_Node*>& solution)
{
  if(found_solution_)
  {
    delete_tree_except_solution();
    get_solution(solution);

    int shortcut_length, start_index;  // Defined as index of end - index of start of shortcut
    int min_shortcut(5);  // Min of one node between the start and end of a shortcut attempt
    int max_shortcut(solution.size()-1);
    int revision_count(0);
    double p; // for probability distribution
    double variate1, variate2;
    double mean, std_dev;

    std::vector<RRT_Node*> tmp_path;
    double cost, tmp_cost;


    for(int i(0); i < num_iterations && max_shortcut >= min_shortcut; ++i)
    {
      // p = 1. - pow( 1./std::max(50.,(double)num_iterations) , 1./(double)(max_shortcut - min_shortcut + 1) );
      // _get_geometric_dist( p, shortcut_length );
      mean = std::min(30. , max_shortcut/2.);
      std_dev = std::max((mean-min_shortcut)/2., (max_shortcut-mean)/3.);

      if(i%2 == 0)  _get_normal_dist(variate1, variate2);
      else          variate1 = variate2;

      shortcut_length = (int)floor((variate1 * std_dev) + mean);
      _bound_shortcut_length(shortcut_length, min_shortcut, max_shortcut);

      start_index = SJ_RAND_INT(0, solution.size()-shortcut_length );

      // std::cout << start_index << " -> " << (start_index+shortcut_length);// << std::endl;

      cost =  (solution[start_index+shortcut_length])->get_cost_to_root()
            - (solution[start_index])->get_cost_to_root();

      // std::cout << " cost = " << cost << ", " << std::endl; //<< " -> ";

      path_type soln_type = UNKNOWN;
      get_path( *(solution[start_index]) , *(solution[start_index+shortcut_length]) , false , tmp_path , tmp_cost, soln_type );
      get_path( *(solution[start_index]) , *(solution[start_index+shortcut_length]) , true , tmp_path , tmp_cost , soln_type );

      // std::cout << "tmp_path.size() = " << tmp_path.size() << std::endl;

      prune_for_collisions(tmp_path);

      // std::cout << "tmp_path.size() = " << tmp_path.size() << std::endl;
      // (*tmp_path.back()).print_node();

      if ( tmp_path.size() != 0 && *tmp_path.back() == *(solution[start_index+shortcut_length]) && tmp_cost < cost)
      {
        revision_count++;

        std::vector<RRT_Node*>::iterator it, it_end, it_shrtct;
        it = solution.begin() + start_index;
        it_end = it + (shortcut_length + 1);  // represents the 1st node after the end of the shortcut
        it_shrtct = tmp_path.begin();

        for(; it < it_end && it_shrtct < tmp_path.end(); ++it, ++it_shrtct)
        {
          delete *it;
          *it = *it_shrtct;
        }

        if ( it != it_end ) // then it_shrtct == tmp_path.end(), so must remove nodes from soln vector
        {
          for (; it != it_end; ++it)
          {
            delete *it;
            *it = NULL;
          }
          // Warning: I don't understand the code
          //solution.erase( std::remove (solution.begin(), solution.end(), NULL), solution.end() );
        }
        else if ( it_shrtct != tmp_path.end() ) // then it == it_end, so must insert nodes into soln vector before it_end
        {
          solution.insert( it_end , it_shrtct , tmp_path.end() );
        }

        _get_cost(solution);

        if(analytic_solver_ == NULL) // Assigns parents if not done by _get_cost()
        {
          it = solution.begin();
          std::vector<RRT_Node*>::iterator it_p = it;
          it++;

          for(; it != solution.end(); ++it, ++it_p ) // starts from first new node
            (*it)->parent_ = *it_p;
        }

        tmp_path.clear();
      }
      else
      {
        std::vector<RRT_Node*>::iterator it = tmp_path.begin();
        for (; it != tmp_path.end(); ++it )
          delete *it;
        tmp_path.clear();
      }

      max_shortcut = solution.size()-1;
    }
    tree_ = solution;
    // std::cout << "Revised solution " << revision_count << " times." << std::endl;
  }
#ifdef SAVE_DATA
  std::vector<RRT_Node*>::iterator it = solution.begin();
  for (; it != solution.end(); ++it)
    {
      // std::cout << "Node " << i << ": " << "x = " << (*it)->x_ << ", y = " << (*it)->y_ << ", theta = " << (*it)->theta_ << ", cost = " << (*it)->cost_to_parent_ << std::endl;
      // (*it)->print_node();
      (*it)->save_node_2("soln2");
    }
#endif

}


void RRT_Planner::get_nearest_neighbor_path(const RRT_Node& sample, RRT_Node*& nearest_neighbor,
                                       std::vector<RRT_Node*>& path)
{

  double cost(0.);
  double tmp_cost(0.);
  path_type soln_type;
  std::vector<RRT_Node*> tmp_path;

  // int counter(0);

  std::list<RRT_Node*> nearby_nodes;
  std::list<path_type> nearby_nodes_type;
  std::list<double>    nearby_nodes_cost;

  std::vector<RRT_Node*>::iterator it = tree_.begin();
  std::list  <RRT_Node*>::iterator it_nearby_nodes;
  std::list  <path_type>::iterator it_nearby_nodes_type;
  std::list  <double   >::iterator it_nearby_nodes_cost;

  // std::cout << "\n\nTree_Size = " << tree_.size() << std::endl;

  // Using dubins path length to gather a list of the nearest 20 nodes:
  for (; nearby_nodes.size() < 20 && it != tree_.end(); ++it )
  {
    soln_type = UNKNOWN;
    get_path(**it, sample, false, path, cost, soln_type);

    it_nearby_nodes      = nearby_nodes.begin();
    it_nearby_nodes_type = nearby_nodes_type.begin();
    it_nearby_nodes_cost = nearby_nodes_cost.begin();

    // Sorted insert
    for(; it_nearby_nodes != nearby_nodes.end() && cost > *it_nearby_nodes_cost;
        ++it_nearby_nodes, ++it_nearby_nodes_type, ++it_nearby_nodes_cost ) {}
    nearby_nodes.insert(      it_nearby_nodes      , *it       );
    nearby_nodes_type.insert( it_nearby_nodes_type , soln_type );
    nearby_nodes_cost.insert( it_nearby_nodes_cost , cost      );

    // ++counter;
    // std::cout << "Inserted node " << counter << std::endl;
  }

  for (; it != tree_.end(); ++it )
  {
    get_path(**it, sample, false, path, cost, soln_type);

    it_nearby_nodes      = nearby_nodes.begin();
    it_nearby_nodes_type = nearby_nodes_type.begin();
    it_nearby_nodes_cost = nearby_nodes_cost.begin();

    // Sorted insert and delete
    for(; it_nearby_nodes != nearby_nodes.end() && cost > *it_nearby_nodes_cost;
        ++it_nearby_nodes, ++it_nearby_nodes_type, ++it_nearby_nodes_cost ) {}
    if(it_nearby_nodes != nearby_nodes.end())
    {
      nearby_nodes.insert(      it_nearby_nodes      , *it       );
      nearby_nodes_type.insert( it_nearby_nodes_type , soln_type );
      nearby_nodes_cost.insert( it_nearby_nodes_cost , cost      );

      nearby_nodes.pop_back();
      nearby_nodes_type.pop_back();
      nearby_nodes_cost.pop_back();
    }

  }


  // Calculating true time cost below:
  it_nearby_nodes      = nearby_nodes.begin();
  it_nearby_nodes_type = nearby_nodes_type.begin();
  it_nearby_nodes_cost = nearby_nodes_cost.begin();

  int idx(0);
  int soln(0);

  get_path(**it_nearby_nodes, sample, true, path, cost, *it_nearby_nodes_type);

  nearest_neighbor = *it_nearby_nodes;
  ++it_nearby_nodes;  ++it_nearby_nodes_type;  ++it_nearby_nodes_cost;  ++idx;

  for (; it_nearby_nodes != nearby_nodes.end();
       ++it_nearby_nodes, ++it_nearby_nodes_type, ++it_nearby_nodes_cost, ++idx)
  {
    get_path(**it_nearby_nodes, sample, true, tmp_path, tmp_cost, *it_nearby_nodes_type);

    if (tmp_cost < cost) {
      nearest_neighbor = *it_nearby_nodes;
      soln = idx;
    }
    _keep_min_delete_tmp(path, cost, tmp_path, tmp_cost);
  }

  // histogram_cost_compare_[soln] += 1.;

  // Lowest cost path is stored in:         std::vector<RRT_Node*>& path
  // Nearest nearest_neighbor stored in:    RRT_Node* nearest_neighbor
}


void RRT_Planner::rand_sample(RRT_Node& rand_node)
{
  rand_node = RRT_Node( SJ_RAND(search_boundary_(0),search_boundary_(1)) ,
                        SJ_RAND(search_boundary_(2),search_boundary_(3)) ,
                        SJ_RAND(      0.0          ,      2.*M_PI      ) );
}


void RRT_Planner::prune_for_collisions(std::vector<RRT_Node*>& path)
{

  // if(found_solution_)
  //   print_path(path);


  bool hit_obstacle(false);
  std::vector<RRT_Node*>::iterator it_n = path.begin();
  std::vector<RRT_Node*>::iterator it_n_remove = path.end();
  // double idx(0);
  for (; it_n != path.end(); ++it_n)
  {
    if( hit_obstacle )
    {
      // if(found_solution_)
      //   std::cout << "Node " << idx << (*it_n)->to_string() << "\n";
      delete *it_n;
    }
    else
    {
      if( _is_out_of_bounds(*it_n) )
      {
        hit_obstacle = true;
        it_n_remove = it_n;
        delete *it_n;
      }
      else
      {
        int idx(0);
        std::vector<Obstacle>::iterator it_obs = obstacle_list_.begin();
        for(; it_obs != obstacle_list_.end(); ++it_obs)
        {
          double margin = safety_margin_;
          // // Warnning: Terrible IDEA.. should be fixed later
          // if(idx == 0) margin = 0.74;
          // if(idx == 15) margin = 0.74;
          ++idx;

          if( it_obs->is_collision( (*it_n)->get_cost_to_root(), (*it_n)->x_, (*it_n)->y_,  margin) )
          {
            hit_obstacle = true;
            it_n_remove = it_n;
            delete *it_n;
            break;
          }
        }
      }
    }
    // ++idx;
  }


  path.erase(it_n_remove, path.end());
}


bool RRT_Planner::_is_out_of_bounds(const RRT_Node* node){
  return !(  (node->x_ - search_boundary_[0]) * (node->x_ - search_boundary_[1]) <= 0 &&
             (node->y_ - search_boundary_[2]) * (node->y_ - search_boundary_[3]) <= 0 );
}


void RRT_Planner::get_path(const RRT_Node& start, const RRT_Node& end, const bool& calc_nodes,
                            std::vector<RRT_Node*>& path, double& cost, path_type& soln_type)
{
#ifdef SAVE_DATA
  std::string error_str("");
  sejong::Vector err_track(8);
#endif

  // Define inscribed circles:
  sejong::Vector center_cw_0(2);
  sejong::Vector center_ccw_0(2);
  sejong::Vector center_cw_1(2);
  sejong::Vector center_ccw_1(2);

  center_cw_0  << start.x_ + rad*sin(start.theta_), start.y_ - rad*cos(start.theta_);
  center_ccw_0 << start.x_ - rad*sin(start.theta_), start.y_ + rad*cos(start.theta_);
  center_cw_1  <<   end.x_ + rad*sin(  end.theta_),   end.y_ - rad*cos(  end.theta_);
  center_ccw_1 <<   end.x_ - rad*sin(  end.theta_),   end.y_ + rad*cos(  end.theta_);

  sejong::Vector center_to_center(2);
  double theta_tangent, turn_0, turn_1;

  cost = 0.;
  double tmp_cost;
  std::vector<RRT_Node*> tmp_path;
  path_type tmp_soln_type = UNKNOWN;

  // RSR
  ////////////////////////
  //  Case 1: CW to CW  //
  ////////////////////////

  if(!calc_nodes || soln_type == RSR || soln_type == UNKNOWN)
  {
    center_to_center = center_cw_1 - center_cw_0;
    theta_tangent = atan2( center_to_center(1) , center_to_center(0) );
                          // + asin((r1-r0)/center_to_center.norm());   //uncomment end to allow for varying radii
    turn_0 = theta_tangent - start.theta_;
    turn_1 = end.theta_ - theta_tangent;

#ifdef SAVE_DATA
    err_track[0] = turn_0;
    err_track[2] = turn_1;
#endif

    _fix_angle(turn_0, RRT_Planner::NEG);
    _fix_angle(turn_1, RRT_Planner::NEG);

#ifdef SAVE_DATA
    err_track[1] = turn_0;
    err_track[3] = turn_1;
#endif

    if (1)//turn_0 + turn_1 > -2.*M_PI)  // can only be optimal if this condition is met
    {
      tmp_soln_type = RSR;
      if(calc_nodes)
      {
        _get_nodes_on_path(turn_0, center_cw_0,
                           turn_1, center_cw_1,
                           start, end, rad, path);
        cost = _get_cost(path);

      }
      else
      {
        cost = _get_dubins_path_length(turn_0, center_cw_0,
                                       turn_1, center_cw_1,
                                       start, end, rad);
      }
    }
#ifdef SAVE_DATA
    else error_str += "skipped RSR\t";
#endif
  }

  // RSL
  ////////////////////////
  // Case 2: CW to CCW  //
  ////////////////////////
  if(!calc_nodes || soln_type == RSL || soln_type == UNKNOWN)
  {
    center_to_center = center_ccw_1 - center_cw_0;
    if(center_to_center.norm() >= 2.*rad )
    {
      theta_tangent = atan2( center_to_center(1) , center_to_center(0) ) - asin((rad+rad)/center_to_center.norm());

      turn_0 = theta_tangent - start.theta_;
      turn_1 = end.theta_ - theta_tangent;

      _fix_angle(turn_0, RRT_Planner::NEG);
      _fix_angle(turn_1, RRT_Planner::POS);

      if(cost != 0.)          // If we already filled cost, then use tmp variables
      {
        if(calc_nodes)
        {
          _get_nodes_on_path(turn_0, center_cw_0,
                             turn_1, center_ccw_1,
                             start, end, rad, tmp_path);

          tmp_cost = _get_cost(tmp_path);

          if (tmp_cost < cost) tmp_soln_type = RSL;

          _keep_min_delete_tmp(path, cost, tmp_path, tmp_cost);
        }
        else
        {
          tmp_cost = _get_dubins_path_length(turn_0, center_cw_0,
                                             turn_1, center_ccw_1,
                                             start, end, rad);
          if (tmp_cost < cost)
          {
            tmp_soln_type = RSL;
            cost = tmp_cost;
          }
        }
      }
      else  // If we skipped previous cost assignment, then don't use tmp variables
      {
        tmp_soln_type = RSL;
        if(calc_nodes)
        {
          _get_nodes_on_path(turn_0, center_cw_0,
                             turn_1, center_ccw_1,
                             start, end, rad, path);
          cost = _get_cost(path);
        }
        else
        {
          cost = _get_dubins_path_length(turn_0, center_cw_0,
                                         turn_1, center_ccw_1,
                                         start, end, rad);
        }
      }
    }
#ifdef SAVE_DATA
    else error_str += "skipped RSL\t";
#endif
  }

  // LSL
  ////////////////////////
  // Case 3: CCW to CCW //
  ////////////////////////
  if(!calc_nodes || soln_type == LSL || soln_type == UNKNOWN)
  {
    center_to_center = center_ccw_1 - center_ccw_0;
    //uncomment end to allow for varying radii
    theta_tangent = atan2( center_to_center(1) , center_to_center(0) );
                          // - asin((r1-r0)/center_to_center.norm());  //uncomment end to allow for varying radii
    turn_0 = theta_tangent - start.theta_;
    turn_1 = end.theta_ - theta_tangent;

#ifdef SAVE_DATA
    err_track[4] = turn_0;
    err_track[6] = turn_1;
#endif

    _fix_angle(turn_0, RRT_Planner::POS);
    _fix_angle(turn_1, RRT_Planner::POS);

#ifdef SAVE_DATA
    err_track[5] = turn_0;
    err_track[7] = turn_1;
#endif

    if(1)//turn_0 + turn_1 < 2.*M_PI)    // can only be optimal if this condition is met
    {
      if(cost != 0.)          // If we already filled cost, then use tmp variables
      {
        if(calc_nodes)
        {
          _get_nodes_on_path(turn_0, center_ccw_0,
                             turn_1, center_ccw_1,
                             start, end, rad, tmp_path);

          tmp_cost = _get_cost(tmp_path);

          if (tmp_cost < cost) tmp_soln_type = LSL;

          _keep_min_delete_tmp(path, cost, tmp_path, tmp_cost);
        }
        else
        {
          tmp_cost = _get_dubins_path_length(turn_0, center_ccw_0,
                                             turn_1, center_ccw_1,
                                             start, end, rad);
          if (tmp_cost < cost)
          {
            tmp_soln_type = LSL;
            cost = tmp_cost;
          }
        }
      }
      else  // If we skipped previous cost assignment, then don't use tmp variables
      {
        tmp_soln_type = LSL;
        if(calc_nodes)
        {
          _get_nodes_on_path(turn_0, center_ccw_0,
                             turn_1, center_ccw_1,
                             start, end, rad, path);
          cost = _get_cost(path);
        }
        else
        {
          cost = _get_dubins_path_length(turn_0, center_ccw_0,
                                         turn_1, center_ccw_1,
                                         start, end, rad);
        }
      }
    }
#ifdef SAVE_DATA
    else error_str += "skipped LSL\t";
#endif
  }

  // LSR
  ////////////////////////
  // Case 4: CCW to CW  //
  ////////////////////////
  if(!calc_nodes || soln_type == LSR || soln_type == UNKNOWN)
  {
    center_to_center = center_cw_1 - center_ccw_0;
    if(center_to_center.norm() >= 2.*rad )
    {
      theta_tangent = atan2( center_to_center(1) , center_to_center(0)) + asin((rad+rad)/center_to_center.norm());

      turn_0 = theta_tangent - start.theta_;
      turn_1 = end.theta_ - theta_tangent;

      _fix_angle(turn_0, RRT_Planner::POS);
      _fix_angle(turn_1, RRT_Planner::NEG);

      if(cost != 0.)          // If we already filled cost, then use tmp variables
      {
        if(calc_nodes)
        {
          _get_nodes_on_path(turn_0, center_ccw_0,
                             turn_1, center_cw_1,
                             start, end, rad, tmp_path);

          tmp_cost = _get_cost(tmp_path);

          if (tmp_cost < cost) tmp_soln_type = LSR;

          _keep_min_delete_tmp(path, cost, tmp_path, tmp_cost);
        }
        else
        {
          tmp_cost = _get_dubins_path_length(turn_0, center_ccw_0,
                                             turn_1, center_cw_1,
                                             start, end, rad);
          if (tmp_cost < cost)
          {
            tmp_soln_type = LSR;
            cost = tmp_cost;
          }
        }
      }
      else  // If we skipped previous cost assignment, then don't use tmp variables
      {
        tmp_soln_type = LSR;
        if(calc_nodes)
        {
          _get_nodes_on_path(turn_0, center_ccw_0,
                             turn_1, center_cw_1,
                             start, end, rad, path);
          cost = _get_cost(path);
        }
        else
        {
          cost = _get_dubins_path_length(turn_0, center_ccw_0,
                                         turn_1, center_cw_1,
                                         start, end, rad);
        }
      }
    }
#ifdef SAVE_DATA
    else error_str += "skipped LSR\t";
#endif
  }
  soln_type = tmp_soln_type;

#ifdef SAVE_DATA
  if(tmp_soln_type == 4)
  {
    std::cout << "Sta node: " << start.to_string() << std::endl;
    std::cout << "End node: " << end.to_string() << std::endl;

    std::cout << "calc_nodes: " << calc_nodes << ", soln_type: " << soln_type << ", cost: " << cost << std::endl;
    std::cout << "error_str: " << error_str << std::endl;
    sejong::pretty_print(err_track, std::cout , "turn_vec");
  }
#endif
  // Lowest cost path is stored in:     std::vector<RRT_Node*>& path
  // Total cost of path is stored in:   double& cost

}


void RRT_Planner::_fix_angle(double& angle, int POS_or_NEG)
{
  angle = remainder( angle , 2.*M_PI );
  if (POS_or_NEG == RRT_Planner::POS && sejong::sgn(angle) == RRT_Planner::NEG)
    angle +=  2. * M_PI;
  else if (POS_or_NEG == RRT_Planner::NEG && sejong::sgn(angle) == RRT_Planner::POS)
    angle += -2. * M_PI;
}


double RRT_Planner::_get_dubins_path_length(const double& turn_0, sejong::Vector& center_0,
                                            const double& turn_1, sejong::Vector& center_1,
                                            const RRT_Node& start, const RRT_Node& end,
                                            const double& radius)
{
  double t0_length, sl_length, t1_length;
  Eigen::Rotation2D< double > rot;
  sejong::Vector rad_v(2);
  sejong::Vector start_v(2);
  sejong::Vector end_v(2);
  sejong::Vector beg_of_sl(2);
  sejong::Vector end_of_sl(2);
  sejong::Vector strait_line(2);


  start_v << start.x_, start.y_;
  end_v << end.x_, end.y_;

  rad_v = start_v - center_0;
  rot = Eigen::Rotation2D<double>( turn_0 );
  beg_of_sl = center_0 + rot * rad_v;

  rad_v = end_v - center_1;
  rot = (Eigen::Rotation2D<double>( turn_1 )).inverse();
  end_of_sl = center_1 + rot * rad_v;

  strait_line = end_of_sl - beg_of_sl;
  sl_length = strait_line.norm();

  t0_length = radius * std::abs(turn_0);
  t1_length = radius * std::abs(turn_1);

  return t0_length + sl_length + t1_length;
}


void RRT_Planner::_get_nodes_on_path(const double& turn_0, sejong::Vector& center_0,
                                       const double& turn_1, sejong::Vector& center_1,
                                       const RRT_Node& start, const RRT_Node& end,
                                       const double& radius, std::vector<RRT_Node*>& nodes)
{
  double tot_length, step_length, progress, num_steps, t0_length, sl_length, t1_length;
  Eigen::Rotation2D< double > rot;
  sejong::Vector apex(2);
  sejong::Vector rad_v(2);
  sejong::Vector start_v(2);
  sejong::Vector end_v(2);
  sejong::Vector beg_of_sl(2);
  sejong::Vector end_of_sl(2);
  sejong::Vector strait_line(2);


  start_v << start.x_, start.y_;
  end_v << end.x_, end.y_;

  rad_v = start_v - center_0;
  rot = Eigen::Rotation2D<double>( turn_0 );
  beg_of_sl = center_0 + rot * rad_v;

  rad_v = end_v - center_1;
  rot = (Eigen::Rotation2D<double>( turn_1 )).inverse();
  end_of_sl = center_1 + rot * rad_v;

  strait_line = end_of_sl - beg_of_sl;
  sl_length = strait_line.norm();

  t0_length = radius * std::abs(turn_0);
  t1_length = radius * std::abs(turn_1);

  tot_length = t0_length + sl_length + t1_length;
  num_steps = ceil( tot_length / x_p_ );
  step_length  = tot_length / num_steps;

  nodes.clear();
  nodes.push_back(new RRT_Node(start));
  progress = 0.;

  for (int i(1); i < num_steps; ++i)
  {
    progress += step_length;

    if ( progress < t0_length )
    {
      rad_v = start_v - center_0;
      rot = Eigen::Rotation2D<double>( sejong::sgn(turn_0) * progress / radius );
      apex = center_0 + rot * rad_v;

      RRT_Node* n = new RRT_Node( apex(0), apex(1), start.theta_ + rot.angle() );
      nodes.push_back(n);
    }
    else if ( progress < t0_length + sl_length )
    {
      apex = beg_of_sl + strait_line * (progress - t0_length) / sl_length;

      RRT_Node* n = new RRT_Node( apex(0), apex(1), start.theta_ + turn_0 );
      nodes.push_back(n);
    }
    else
    {
      rad_v = end_v - center_1;
      rot = (Eigen::Rotation2D<double>( sejong::sgn(turn_1) * (tot_length-progress) / radius )).inverse();
      apex = center_1 + rot * rad_v;

      RRT_Node* n = new RRT_Node( apex(0), apex(1), end.theta_ + rot.angle() );
      nodes.push_back(n);
    }
  }

  nodes.push_back(new RRT_Node(end));

}




// Returns true if successful, false if fails
// Note: Condition to connect tree with branch is:
//       (*parent_on_tree) == *new_branch.front()
// Parents will be properly set for the new_branch
// The duplicate node ( new_branch.front() ) will be deleted.
// False will not delete any nodes.
// This method will not attempt to verify that parent_on_tree does in fact exist inside the tree vector,
//    this is the responsibility of the caller.

bool RRT_Planner::append_path_to_tree(std::vector<RRT_Node*>& new_branch,
                                      RRT_Node* parent_on_tree )
{

  if ( (*parent_on_tree) ==  *(new_branch.front()) && !new_branch.empty()) // forward branch
  {
    delete new_branch.front();
    new_branch.front() = parent_on_tree;
    std::vector<RRT_Node*>::iterator it = new_branch.begin();
    std::vector<RRT_Node*>::iterator it_p = it;
    it++;

    for(; it != new_branch.end(); ++it, ++it_p ) // starts from first new node
    {
      (*it)->parent_ = *it_p;
      tree_.push_back(*it);  // Note: this assumes that new_branch.front() is already in tree_
#ifdef SAVE_DATA
      (*it)->save_node_2("tree");
#endif
    }
  }
  else return false; // Bad new_branch

  return true; // Success
}


void RRT_Planner::delete_tree_except_solution()
{
  if (found_solution_)
  {
    std::vector<RRT_Node*> soln;
    get_solution(soln);
    std::vector<RRT_Node*>::iterator it = tree_.begin();

    for (; it != tree_.end(); ++it)
    {
      if(std::find(soln.begin(), soln.end(), *it) == soln.end())
        delete *it;
    }
    // tree_.clear();
    tree_ = soln;
  }
}



void RRT_Planner::_keep_min_delete_tmp(std::vector<RRT_Node*>&     path, double&     cost,
                                       std::vector<RRT_Node*>& tmp_path, double& tmp_cost)
{
  // std::cout << "{Inside delete function}\n";

  if (tmp_cost < cost)
  {
    // swap nodes in path
    cost = tmp_cost;
    path.swap(tmp_path);
  }

  // Delete nodes in tmp_path
  for( std::vector<RRT_Node*>::iterator it = tmp_path.begin(); it != tmp_path.end(); ++it )
    delete *it;
  tmp_path.clear();
  tmp_cost = 0.0;
}

double RRT_Planner::_get_cost(std::vector<RRT_Node*>& nodes){

  //Select how to calc cost
  if( analytic_solver_ == NULL )
    return _tmp_get_cost(nodes);
  else
    return _get_time(nodes);
}

double RRT_Planner::_get_cost_eucl(std::vector<RRT_Node*>& nodes){
    return _tmp_get_cost(nodes);
}

double RRT_Planner::_get_time(std::vector<RRT_Node*>& nodes){
  double ret(analytic_solver_->Get_Cost(nodes));
  std::vector<RRT_Node*>::iterator debug_iter;
  sejong::Vector debug_vector(3);
  if(ret<0) {
    std::cout<<"negative time : exit for debugging :"<<ret<<std::endl;
    for(debug_iter = nodes.begin() + 1; debug_iter != nodes.end(); ++debug_iter){
      debug_vector[0] = (*debug_iter)->x_;
      debug_vector[1] = (*debug_iter)->y_;
      debug_vector[2] = (*debug_iter)->theta_;
      (*debug_iter)->print_node();
      std::cout<<(*debug_iter)->cost_to_parent_<<std::endl;
#ifdef SAVE_DATA
      sejong::saveVector(debug_vector,"debugging_nodes");
#endif
    }
    exit(0);
  }
  return ret;
}

double RRT_Planner::_tmp_get_cost(std::vector<RRT_Node*>& nodes)
{
  double cost(0.0);
  for(std::vector<RRT_Node*>::size_type i = 0; i < nodes.size()-1; i++)
  {
    nodes[i+1]->parent_ = nodes[i]; // Assigns parents to all except nodes[0]
    nodes[i+1]->cost_to_parent_ = sqrt(pow(nodes[i]->x_-nodes[i+1]->x_,2.)+pow(nodes[i]->y_-nodes[i+1]->y_,2.));// + step_length/4.;
    cost += nodes[i+1]->cost_to_parent_;
  }
  return cost;
}

void RRT_Planner::print_path(std::vector<RRT_Node*>& nodes)
{
  std::vector<RRT_Node*>::iterator it = nodes.begin();
  int i(0);
  for(; it != nodes.end(); ++it, ++i)
  {
    std::cout << "Node " << i << ": ";
    (*it)->print_node();
  }
}


// Probability utility functions //

void RRT_Planner::_get_normal_dist(double& variate1, double& variate2)
{
  double u1(SJ_RAND(0.,1.));
  double u2(SJ_RAND(0.,1.));
  variate1 = sqrt(-2.*log(u1))*cos(2*M_PI*u2);
  variate2 = sqrt(-2.*log(u1))*sin(2*M_PI*u2);

}

// returns pseudorandom sample from geometric distribution in variate from set k = {0,1,2,...}
void RRT_Planner::_get_geometric_dist(const double& p, int& variate)
{
  variate = (int)floor(log(SJ_RAND(0.,1.))/log(1.-p));
}

void RRT_Planner::_bound_shortcut_length(int& shortcut_length, const int& min_shortcut, const int& max_shortcut)
{
  while (shortcut_length < min_shortcut)
  {
    shortcut_length -= min_shortcut;
    shortcut_length *= -1;
    shortcut_length += min_shortcut;
    if (shortcut_length > max_shortcut)
    {
      shortcut_length -= max_shortcut;
      shortcut_length *= -1;
      shortcut_length += max_shortcut;
    }
  }

  while (shortcut_length > max_shortcut)
  {
    shortcut_length -= max_shortcut;
    shortcut_length *= -1;
    shortcut_length += max_shortcut;
    if (shortcut_length < min_shortcut)
    {
      shortcut_length -= min_shortcut;
      shortcut_length *= -1;
      shortcut_length += min_shortcut;
    }
  }
}



// ORIGINALLY FROM SHORTCUTTING
// std::vector<RRT_Node*>::iterator it, it_p, it_end, it_shrtct;
// it = solution.begin() + start_index;
// it_end = it + shortcut_length + 1;  // represents the 1st node after the end of the shortcut
// it_shrtct = tmp_path.begin();
//
//
// for(; it < it_end && it_shrtct < tmp_path.end(); ++it, ++it_shrtct)
// {
//   delete *it;
//   *it = *it_shrtct;
// }
//
// if ( it != it_end ) // then it_shrtct == tmp_path.end(), so must remove nodes from soln vector
// {
//   for (; it != it_end; ++it)
//   {
//     delete *it;
//     *it = NULL;
//   }
//   solution.erase( std::remove (solution.begin(), solution.end(), NULL), solution.end() );
// }
// else if ( it_shrtct != tmp_path.end() ) // then it == it_end, so must insert nodes into soln vector before it_end
// {
//   solution.insert( it_end , it_shrtct , tmp_path.end() );
// }
