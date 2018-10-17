#pragma once

#include <memory>
#include <vector>
#include <queue>

#include <Eigen/Dense>

#include <ros/ros.h>

#include <contact_motion_planner/planning_scene.h>
#include <contact_motion_planner/robot_dynamics/robot_dynamics_model.h>
#include <contact_motion_planner/solver/contact_optimization.h>

namespace suhan_contact_planner
{

template <class T>
class ContactModelGraph
{
  // Model state graph
  std::vector< std::vector<int> > state_adjacency_list_;
  std::vector<ContactModelPtr> contact_model_states_;
  size_t contact_models_index_;

  // Contact graph
  std::vector< std::vector<ContactPtr> > contact_nodes_;
  size_t contact_node_index_;
  std::vector< std::vector<int> > contact_adjacency_list_;

  // Total graph
  std::vector<ContactModelPtr> combinated_nodes_;
  std::vector<std::vector<int> > combinated_adjacency_list_;
  //std::vector<bool> state_nodes_visit_;
  //std::vector<bool> contact_nodes_visit_;
  std::vector<std::vector<int> > combinated_visit_;

  std::vector<int> search_visit_;

  // Result
  std::vector<ContactModelPtr> result_;
  std::vector<int> result_int_;


  PlanningScenePtr planning_scene_;
  RobotDynamicsModelPtr robot_dynamics_model_;

  std::shared_ptr<T> start_; // Initial contact information should be provided.
  std::shared_ptr<T> goal_; // Goal contact information should be provided.

  int max_depth_ {50};
  int max_combined_depth_ {8000};
  double discrete_resolution_;
  double angle_resolution_;

public:
  ContactModelGraph() : discrete_resolution_(0.1), angle_resolution_(30 * M_PI/180.) {}

  void setStart(const std::shared_ptr<T> &start) { start_ = start; }
  void setGoal(const std::shared_ptr<T> &goal) { goal_ = goal; }
  void setPlanningScene(const PlanningScenePtr &scene_ptr) { planning_scene_ = scene_ptr; }
  void setRobotDynamicsModel(const RobotDynamicsModelPtr &model_ptr) { robot_dynamics_model_ = model_ptr; }


  bool searchGoal(std::vector<ContactModelPtr>& result)
  {
    // BFS
    search_visit_.resize(combinated_nodes_.size(), false);
    result_.clear();
    std::queue<int> node_queue;
    std::queue<std::vector<int> > path_queue;
    path_queue.push(std::vector<int>());
    path_queue.front().push_back(0);
    node_queue.push(0);

    bool goal_found = false;

    while(!node_queue.empty())
    {
      std::vector<int> path = path_queue.front();
      int n = node_queue.front();
      path_queue.pop();
      node_queue.pop();

      for (int child : combinated_adjacency_list_[n])
      {
        if(!search_visit_[child])
        {
          path_queue.push(path);
          path_queue.back().push_back(child);
          node_queue.push(child);
          search_visit_[child] = true;

          if(combinated_nodes_[child]->goalNode())
          {
            ROS_INFO("GOAL FOUND!");
            result_int_ = path_queue.back();
            result_.clear();
            for(int p : result_int_)
            {
              result_.push_back(combinated_nodes_[p]);
              std::cout << "* Node -----------------------" << p << std::endl;
              std::cout << "Transform: \n" <<
                           combinated_nodes_[p]->getTransform().matrix() << std::endl;

              std::cout << "Contacts: \n";
              for (auto g : combinated_nodes_[p]->getContactRobot())
              {
                g->printContactState();
              }
              //std::cout << combinated_nodes_[p]-><< std::endl;
            }

            result = result_;
            goal_found = true;
            break;
          }
        }
      }
      if (goal_found) break;
    }
  }

  void makeCombinationGraph()
  {
    combinated_nodes_.clear();
    combinated_adjacency_list_.clear();
    //state_nodes_visit_.resize(contact_nodes_.size(), false);
    //contact_nodes_visit_.resize(contact_nodes_.size(), false);
    combinated_visit_.resize(contact_nodes_.size());
    for(auto & m : combinated_visit_)
      m.resize(contact_model_states_.size(), -1); // -1: Not visit. -2: Visited. but no solution exist.

    ContactModelPtr node = std::make_shared<T>(*(dynamic_cast<T*>(contact_model_states_[0].get())));
    node->setContactRobot(contact_nodes_[0]);
    combinated_nodes_.push_back(node);
    combinated_adjacency_list_.push_back(std::vector<int>());

    // Recursive
    for(int new_index: contact_adjacency_list_[0])
    {
      if(combinated_visit_[new_index][0] == -1)
      {
        makeContactCombinationBranch(0, new_index, 0, 1);
      }
      else if(combinated_visit_[new_index][0] >= 0)
      {
        int index = combinated_visit_[new_index][0];
        combinated_adjacency_list_[index].push_back(0);
        combinated_adjacency_list_[0].push_back(index);
      }
    }
    for(int new_index: state_adjacency_list_[0])
    {
      if(combinated_visit_[0][new_index] == -1)
      {
        makeContactCombinationBranch(new_index, 0, 0, 1);
      }
      else if(combinated_visit_[0][new_index] >= 0)
      {
        int index = combinated_visit_[0][new_index];
        combinated_adjacency_list_[index].push_back(0);
        combinated_adjacency_list_[0].push_back(index);
      }
    }
  }
  bool makeContactCombinationBranch(int model_index, int contact_index, int total_index, int depth)
  {
    if (depth > max_combined_depth_) return false;
    ContactModelPtr node = std::make_shared<T>(
          *(dynamic_cast<T*>
            (contact_model_states_[model_index].get())));

    // Pruning when it has a bottom contact with environment
    if (node->getContactEnvironment().size() > 0)
    {
      if (node->getContactEnvironment()[0]->getContactState()
          == Contact::ContactState::CONTACT_FACE)
      {
        for (auto& c: contact_nodes_[contact_index])
        {
          if( c->getContactTransform().translation()(2) <= node->getBottomPositionZ() + 0.001)
          {
            combinated_visit_[contact_index][model_index] = -2;
            return false;
          }
        }
      }
    }

    node->setContactRobot(contact_nodes_[contact_index]);
    node->copyContactRobot();
    node->setMass(start_->getMass());
    node->setFriction(start_->getFriction());

    // Optimization to find the stability
    ContactOptimization op;
    op.setModel(node);
    if(!op.solve())
    {
      //std::cout << "cannot solve" << std::endl;
      combinated_visit_[contact_index][model_index] = -2;
      // Unstable Contacts --> pruning
      return false;
    }
    //std::cout << "solved" << std::endl;

    // Add to graph
    int total_now_index = combinated_nodes_.size();
    combinated_nodes_.push_back(node);
    combinated_adjacency_list_.push_back(std::vector<int>());
    combinated_adjacency_list_[total_index].push_back(total_now_index);
    combinated_adjacency_list_[total_now_index].push_back(total_index);
    combinated_visit_[contact_index][model_index] = total_now_index;

    if(node->goalNode())
    {
      ROS_INFO("Goal!");
      return true;
    }


    // Recursive
    for(int new_index: contact_adjacency_list_[contact_index])
    {
      if(combinated_visit_[new_index][model_index] == -1)
      {
        if(makeContactCombinationBranch(model_index, new_index, total_now_index, depth+1))
          return true;
      }
      else if(combinated_visit_[new_index][model_index] >= 0)
      {
        int index = combinated_visit_[new_index][model_index];
        if(std::find(combinated_adjacency_list_[total_now_index].begin(),
                     combinated_adjacency_list_[total_now_index].end(),index) ==
           combinated_adjacency_list_[total_now_index].end())
        {
          combinated_adjacency_list_[index].push_back(total_now_index);
          combinated_adjacency_list_[total_now_index].push_back(index);
        }
      }
    }
    for(int new_index: state_adjacency_list_[model_index])
    {
      if(combinated_visit_[contact_index][new_index] == -1)
      {
        if(makeContactCombinationBranch(new_index, contact_index, total_now_index, depth+1))
          return true;
      }
      else if(combinated_visit_[contact_index][new_index] >= 0)
      {
        int index = combinated_visit_[contact_index][new_index];
        if(std::find(combinated_adjacency_list_[total_now_index].begin(),
                     combinated_adjacency_list_[total_now_index].end(),index) ==
           combinated_adjacency_list_[total_now_index].end())
        {
          combinated_adjacency_list_[index].push_back(total_now_index);
          combinated_adjacency_list_[total_now_index].push_back(index);
        }
      }
    }
    return false;
  }

  void makeObjectContactGraph()
  {
    ROS_INFO("makeObjectContactGraph");
    ContactModelPtr start_node = std::make_shared<T>(*(dynamic_cast<T*>(start_.get())));
    std::vector<ContactPtr> contact_samples;
    start_node->createContactSamples(contact_samples);

    contact_nodes_.push_back(std::vector<ContactPtr>()); // 1st: zero contact
    contact_adjacency_list_.push_back(std::vector<int>());

    for(const ContactPtr &c : contact_samples)
    {
      // 1 contact states
      std::vector<ContactPtr> node;
      node.push_back(c);
      int current_index_1 = contact_nodes_.size();  // Current
      contact_nodes_.push_back(node);
      contact_adjacency_list_.push_back(std::vector<int>());
      contact_adjacency_list_[0].push_back(current_index_1);
      contact_adjacency_list_[current_index_1].push_back(0);

      for(const ContactPtr &c2 : contact_samples)
      {
        if(c == c2)
          continue;

        std::vector<ContactPtr> node;
        node.push_back(c);
        node.push_back(c2);
        bool contact_exist = false;
        int contact_index = 0;
        for(int i=0; i<contact_nodes_.size(); i++)
        {
          if(contact_nodes_[i].size() == 2)
          {
            if((contact_nodes_[i][0] == c && contact_nodes_[i][1] == c2) ||
               (contact_nodes_[i][1] == c && contact_nodes_[i][0] == c2))
            {
              // Same contact
              contact_exist = true;
              contact_index = i;
              break;
            }
          }
        }
        if(contact_exist)
        {
          contact_adjacency_list_[current_index_1].push_back(contact_index);
          contact_adjacency_list_[contact_index].push_back(current_index_1);
        }
        else
        {
          int current_index_2 = contact_nodes_.size();
          contact_nodes_.push_back(node);
          contact_adjacency_list_.push_back(std::vector<int>());
          contact_adjacency_list_[current_index_1].push_back(current_index_2);
          contact_adjacency_list_[current_index_2].push_back(current_index_1);
        }
      }
    }
  }
  void printContactGraph()
  {
    for(int i=0; i<contact_nodes_.size();i++)
    {
      std::cout << '[' << i << ']' << " Node ----- " << std::endl;
      for(auto node : contact_adjacency_list_[i])
      {
        std::cout << "  " << node << std::endl;
      }
    }
  }
  void printTotalGraph()
  {
    for(int i=0; i<combinated_nodes_.size();i++)
    {
      std::cout << '[' << i << ']' << " Node ----- " << std::endl;
      for(auto node : combinated_adjacency_list_[i])
      {
        std::cout << "  " << node << std::endl;
      }
    }
  }
  void makeObjectPoseGraph()
  {
    ROS_INFO("makeObjectPoseGraph");
    contact_model_states_.clear();
    contact_model_states_.push_back(start_);
    state_adjacency_list_.push_back(std::vector<int>());
    contact_models_index_ = 1;

    // TODO: BFS!
    std::queue<int> bfs_node_queue;
    bfs_node_queue.push(0);

    while(!bfs_node_queue.empty())
    {
      int current_index = bfs_node_queue.front();
      bfs_node_queue.pop();

      for(int dir=ContactModel::DIR_Z; dir<ContactModel::DIR_ROLL; dir++) // Test every direction
      {
        for (int i=-1; i<2; i+=2)
        {
          ContactModelPtr new_node = std::make_shared<T>(*(dynamic_cast<T*>(contact_model_states_[current_index].get()))); // Creation of new node
          new_node->copyContactEnvironment();
          if(new_node->operate(static_cast<ContactModel::OperationDirection>(dir),
                                 discrete_resolution_ * i, angle_resolution_ * i))
            // If operation is available + operate it
            // i means sign
          {
            int index = processOperatedNode(new_node, current_index);
            if(index > 0)
            {
              bfs_node_queue.push(index);
            }
          }
        }
      }
    }
    // makeObjectStateTree(0, 0);
  }

private:
  int processOperatedNode(ContactModelPtr new_node, int current_index)
  {
    // print DEBUG
    //auto& trans = new_node->getPosition();
    //ROS_INFO("%lf %lf %lf",trans[0], trans[1], trans[2]);
    /*
    if (new_node->isSamePose(*goal_, discrete_resolution_*0.7, angle_resolution_*0.7))
    {
      ROS_INFO("arrived at goal depth = %d", depth);
      // final arrived
      int index = contact_models_index_;
      contact_model_states_.push_back(new_node);
      state_adjacency_list_.push_back(std::vector<int>());
      contact_models_index_++;
      state_adjacency_list_[index].push_back(current_index);
      state_adjacency_list_[current_index].push_back(index);

      result_.push_back(new_node);
      return true;

    }
      */
    if(contact_models_index_ > 20000) return -1;
    int index = -1;
    bool detectedSamePose = false;
    for(auto it=contact_model_states_.begin(); it!=contact_model_states_.end(); it++) //
    {
      if (new_node->isSamePose(*(*it), discrete_resolution_ * 0.5, angle_resolution_*0.5))
      {
        //ROS_INFO("same pose detected");
        detectedSamePose = true;
        int same_index = std::distance(contact_model_states_.begin(), it);
        for(int connected_index : state_adjacency_list_[same_index])
        {
          if(std::find(state_adjacency_list_[connected_index].begin(),    // If not connected yet,
                       state_adjacency_list_[connected_index].end(),
                       same_index) == state_adjacency_list_[connected_index].end())
          {
            state_adjacency_list_[connected_index].push_back(current_index);
            state_adjacency_list_[current_index].push_back(connected_index);
          }
        }
      }
    }
    if(!detectedSamePose)
    {
      //result_.push_back(new_node);
      planning_scene_->setTargetObject(new_node); // set planning scene to check whether it is possible
      if (planning_scene_->isPossible() && robot_dynamics_model_->isReachable(new_node->getPosition())) // if feasible point
      {
        if (new_node->isSamePose(*goal_, discrete_resolution_*0.5, angle_resolution_*0.5))
        {
          std::cout << "makeObjectState - Find goal" << std::endl;
          new_node->setGoalNode();
          new_node->setTransform(goal_->getTransform());
        }
        index = contact_models_index_;
        contact_model_states_.push_back(new_node);
        state_adjacency_list_.push_back(std::vector<int>());
        contact_models_index_++;
        state_adjacency_list_[index].push_back(current_index);
        state_adjacency_list_[current_index].push_back(index);

//        if(makeObjectStateTree(index, depth+1))
//        {
//          // result_.push_back(new_node);
//          return true;
//        }; // Do this at next node
      }
    }
    return index;
  }/*
  bool makeObjectStateTree(int current_index, int depth)
  {
    if (depth > max_depth_)
    {
      assert("max depth is arrived");
      return false;
    }

    //auto &current_node = contact_models_[current_index];

    for(int dir=ContactModel::DIR_Z; dir<ContactModel::DIR_ROLL; dir++) // Test every direction
    {
      ContactModelPtr new_node = std::make_shared<T>(*(dynamic_cast<T*>(contact_model_states_[current_index].get()))); // Creation of new node
      new_node->copyContactEnvironment();
      if(new_node->operate(static_cast<ContactModel::OperationDirection>(dir), discrete_resolution_, angle_resolution_)) // If operation is available + operate it
      {
        if(processOperatedNode(new_node, current_index, depth)) return true;
      }
      if(new_node->operate(static_cast<ContactModel::OperationDirection>(dir), -discrete_resolution_, -angle_resolution_)) // If operation is available + operate it
      {
        if(processOperatedNode(new_node, current_index, depth)) return true;
      }
    }
    return false;
  }*/
};

}
