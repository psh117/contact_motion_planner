#ifndef CONTACT_MODEL_GRAPH_H
#define CONTACT_MODEL_GRAPH_H

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "contact_motion_planner/planning_scene.h"
#include "robot_dynamics/robot_dynamics_model.h"
#include <ros/ros.h>

#ifndef MAX_NODE
#define MAX_NODE 10000
#endif

namespace suhan_contact_planner
{

template <class T>
class ContactModelGraph
{
  std::vector<int> adjacency_list_[MAX_NODE];
  std::vector<std::shared_ptr<T> > contact_models_;
  size_t contact_models_index_;
  std::vector<std::shared_ptr<T> > result_;

  PlanningScenePtr planning_scene_;
  RobotDynamicsModelPtr robot_dynamics_model_;

  std::shared_ptr<T> start_; // Initial contact information should be provided.
  std::shared_ptr<T> goal_; // Goal contact information should be provided.

  int max_depth_;
  double discrete_resolution_;
  double angle_resolution_;

public:

  ContactModelGraph() : discrete_resolution_(0.05), angle_resolution_(0.62832), max_depth_(100) {}

  void setStart(const std::shared_ptr<T> &start) { start_ = start; }
  void setGoal(const std::shared_ptr<T> &goal) { goal_ = goal; }
  void setPlanningScene(const PlanningScenePtr &scene_ptr) { planning_scene_ = scene_ptr; }
  void setRobotDynamicsModel(const RobotDynamicsModelPtr &model_ptr) { robot_dynamics_model_ = model_ptr; }


  void makeObjectContactGraph()
  {
    ROS_INFO("makeObjectContactGraph");
    

  }
  void makeObjectPoseGraph()
  {
    ROS_INFO("makeGraph");
    contact_models_.clear();
    contact_models_.push_back(start_);
    contact_models_index_ = 1;
    result_.clear();

    makeTree(0, 0);

    for(auto& node : result_)
    {
      Eigen::Isometry3d transform = node->getTransform();
      auto& trans = node->getPosition();
      ROS_INFO("TTTT %lf %lf %lf",trans[0], trans[1], trans[2]);
      std::cout << transform.linear() << std::endl;
    }
  }

private:
  bool processOperatedNode(std::shared_ptr<T> new_node, int current_index, int depth)
  {
    // print DEBUG
    auto& trans = new_node->getPosition();
    //ROS_INFO("%lf %lf %lf",trans[0], trans[1], trans[2]);
    if (new_node->isSamePose(*goal_, discrete_resolution_*0.7, angle_resolution_*0.7))
    {
      ROS_INFO("arrived at goal depth = %d", depth);
      // final arrived
      int index = contact_models_index_;
      contact_models_.push_back(new_node);
      contact_models_index_++;
      adjacency_list_[index].push_back(current_index);
      adjacency_list_[current_index].push_back(index);

      result_.push_back(new_node);
      return true;

    }

    bool detectedSamePose = false;
    for(auto it=contact_models_.begin(); it!=contact_models_.end(); it++) //
    {
      if (new_node->isSamePose(*(*it), discrete_resolution_ * 0.5, angle_resolution_*0.5))
      {
        //ROS_INFO("same pose detected");
        detectedSamePose = true;
        int index = std::distance(contact_models_.begin(), it);
        for(int connected_index : adjacency_list_[index])
        {
          if(std::find(adjacency_list_[connected_index].begin(),    // If not connected yet,
                       adjacency_list_[connected_index].end(),
                       index) == adjacency_list_[connected_index].end())
          {
            adjacency_list_[connected_index].push_back(current_index);
            adjacency_list_[current_index].push_back(connected_index);
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
        contact_models_.push_back(new_node);
        contact_models_index_++;
        adjacency_list_[index].push_back(current_index);
        adjacency_list_[current_index].push_back(index);

        if(makeTree(index, depth+1))
        {
          result_.push_back(new_node);
          return true;
        }; // Do this at next node
      }
    }
    return false;
  }
  bool makeTree(int current_index, int depth)
  {
    if (depth > max_depth_)
    {
      assert("max depth is arrived");
      return false;
    }

    //auto &current_node = contact_models_[current_index];

    for(int dir=ContactModel::DIR_X; dir<ContactModel::DIR_ROLL; dir++) // Test every direction
    {
      auto new_node = std::make_shared<T>(*(contact_models_[current_index])); // Creation of new node
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
