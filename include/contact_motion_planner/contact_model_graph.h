#ifndef CONTACT_MODEL_GRAPH_H
#define CONTACT_MODEL_GRAPH_H

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "contact_motion_planner/planning_scene.h"
#include "robot_dynamics/robot_dynamics_model.h"
#include <contact_motion_planner/solver/contact_optimization.h>
#include <ros/ros.h>

//#ifndef MAX_NODE
//#define MAX_NODE 10000
//#endif

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
  std::vector<bool> state_nodes_visit_;
  std::vector<bool> contact_nodes_visit_;

  // Result
  std::vector<ContactModelPtr> result_;


  PlanningScenePtr planning_scene_;
  RobotDynamicsModelPtr robot_dynamics_model_;

  std::shared_ptr<T> start_; // Initial contact information should be provided.
  std::shared_ptr<T> goal_; // Goal contact information should be provided.

  int max_depth_;
  double discrete_resolution_;
  double angle_resolution_;

public:

  ContactModelGraph() : discrete_resolution_(0.1), angle_resolution_(0.62832), max_depth_(100) {}

  void setStart(const std::shared_ptr<T> &start) { start_ = start; }
  void setGoal(const std::shared_ptr<T> &goal) { goal_ = goal; }
  void setPlanningScene(const PlanningScenePtr &scene_ptr) { planning_scene_ = scene_ptr; }
  void setRobotDynamicsModel(const RobotDynamicsModelPtr &model_ptr) { robot_dynamics_model_ = model_ptr; }


  void makeCombinationGraph()
  {
    combinated_nodes_.clear();
    combinated_adjacency_list_.clear();
    state_nodes_visit_.resize(contact_nodes_.size(), false);
    contact_nodes_visit_.resize(contact_nodes_.size(), false);

    // First, start with zero contact state
    ContactModelPtr node = std::make_shared<T>(*(dynamic_cast<T*>(contact_model_states_[0].get())));
    node->setContactRobot(std::vector<ContactPtr>());
    // we need to add bottom contact

    combinated_nodes_.push_back(node);
    combinated_adjacency_list_.push_back(std::vector<int>());

    makeContactCombinationBranch(0, 0, 0);
  }
  void makeContactCombinationBranch(int model_index, int contact_index, int total_index)
  {
    ContactModelPtr node = std::make_shared<T>(*(dynamic_cast<T*>(contact_model_states_[model_index].get())));

    // Optimization to find the stability
    node->setContactRobot(contact_nodes_[contact_index]);
    ContactOptimization op;
    op.setModel(node);
    if(!op.solve())
    {
      // Unstable Contacts --> pruning
      return;
    }

    // Add to graph
    int total_now_index = combinated_nodes_.size();
    combinated_nodes_.push_back(node);
    combinated_adjacency_list_.push_back(std::vector<int>());
    combinated_adjacency_list_[total_index].push_back(total_now_index);
    combinated_adjacency_list_[total_now_index].push_back(total_index);

    // Recursive
    for(int new_index: contact_adjacency_list_[contact_index])
    {
      if(!contact_nodes_visit_[new_index])
      {
        contact_nodes_visit_[new_index] = true;
        makeContactCombinationBranch(model_index, new_index, total_now_index);
        contact_nodes_visit_[new_index] = false;
      }
    }
    for(int new_index: state_adjacency_list_[contact_index])
    {
      if(!state_nodes_visit_[new_index])
      {
        state_nodes_visit_[new_index] = true;
        makeContactCombinationBranch(new_index, contact_index, total_now_index);
        state_nodes_visit_[new_index] = false;
      }
    }
  }

  void makeObjectContactGraph()
  {
    ROS_INFO("makeObjectContactGraph");
    ContactModelPtr start_node = std::make_shared<T>(*(dynamic_cast<T*>(start_.get())));
    std::vector<ContactPtr> contact_samples;
    start_node->createContactSamples(contact_samples);

    for(const ContactPtr &c : contact_samples)
    {
      // 1 contact states
      std::vector<ContactPtr> node;
      node.push_back(c);
      int current_index_1 = contact_nodes_.size();
      contact_nodes_.push_back(node);
      contact_adjacency_list_.push_back(std::vector<int>());

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

        // FIXME: TEST!!!
        /*
        auto some_node = std::make_shared<T>(*(dynamic_cast<T*>(start_.get())));;
        //std::vector<ContactPtr> env_contact;
        //auto
        some_node->setContactRobot(node);
        ContactOptimization op;
        op.setModel(some_node);
        op.initializeConstraints();
        */
        // op.setRobot();
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
  void makeObjectPoseGraph()
  {
    ROS_INFO("makeObjectPoseGraph");
    contact_model_states_.clear();
    contact_model_states_.push_back(start_);
    state_adjacency_list_.push_back(std::vector<int>());
    contact_models_index_ = 1;
    result_.clear();

    makeObjectStateTree(0, 0);

    for(auto& node : result_)
    {
      Eigen::Isometry3d transform = node->getTransform();
      auto& trans = node->getPosition();
      ROS_INFO("TTTT %lf %lf %lf",trans[0], trans[1], trans[2]);
      std::cout << transform.linear() << std::endl;
    }
  }

private:
  bool processOperatedNode(ContactModelPtr new_node, int current_index, int depth)
  {
    // print DEBUG
    auto& trans = new_node->getPosition();
    //ROS_INFO("%lf %lf %lf",trans[0], trans[1], trans[2]);
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

    bool detectedSamePose = false;
    for(auto it=contact_model_states_.begin(); it!=contact_model_states_.end(); it++) //
    {
      if (new_node->isSamePose(*(*it), discrete_resolution_ * 0.5, angle_resolution_*0.5))
      {
        //ROS_INFO("same pose detected");
        detectedSamePose = true;
        int index = std::distance(contact_model_states_.begin(), it);
        for(int connected_index : state_adjacency_list_[index])
        {
          if(std::find(state_adjacency_list_[connected_index].begin(),    // If not connected yet,
                       state_adjacency_list_[connected_index].end(),
                       index) == state_adjacency_list_[connected_index].end())
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
        int index = contact_models_index_;
        contact_model_states_.push_back(new_node);
        state_adjacency_list_.push_back(std::vector<int>());
        contact_models_index_++;
        state_adjacency_list_[index].push_back(current_index);
        state_adjacency_list_[current_index].push_back(index);

        if(makeObjectStateTree(index, depth+1))
        {
          result_.push_back(new_node);
          return true;
        }; // Do this at next node
      }
    }
    return false;
  }
  bool makeObjectStateTree(int current_index, int depth)
  {
    if (depth > max_depth_)
    {
      assert("max depth is arrived");
      return false;
    }

    //auto &current_node = contact_models_[current_index];

    for(int dir=ContactModel::DIR_X; dir<ContactModel::DIR_ROLL; dir++) // Test every direction
    {
      ContactModelPtr new_node = std::make_shared<T>(*(dynamic_cast<T*>(contact_model_states_[current_index].get()))); // Creation of new node
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
  }


};

}
#endif
