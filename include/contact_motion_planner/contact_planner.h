#ifndef CONTACT_PLANNER_H
#define CONTACT_PLANNER_H

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shapes_utility.h>

namespace suhan_contact_planner
{


class ObjectMotionDiscretizer
{

  double position_resolution;
  double orientation_resolution;

};



class Contact
{
public:
  enum ContactRelation {CONTACT_OBJ_ENV, CONTACT_OBJ_ROBOT};
  enum ContactState {CONTACT_FACE, CONTACT_LINE, CONTACT_POINT};
  // Contact() {}
  Contact(ContactRelation contact_relation, ContactState contact_state);
  inline const ContactState getContactState() const {return contact_state_; }

protected:
  ContactRelation contact_relation_;
  ContactState contact_state_;

private:
  // position & rotation with regard to object frame
  Eigen::Vector3d position_;  ///< is set randomly or by discretization
  Eigen::Matrix3d rotation_;  ///< is set by model and position


  Eigen::Vector3d contact_force_;
  Eigen::Vector3d contact_torque_;

};

class FaceContact : public Contact
{
  FaceContact(ContactRelation contact_relation) : Contact(contact_relation, CONTACT_FACE) // Rectangle? Assume...
  {
    fcl::Box g;
  }
};

typedef std::shared_ptr<Contact> ContactPtr;

/**
 * @brief The ContactModel class
 */
class ContactModel
{
public:
  ContactModel() : name_("") {}
  ContactModel(std::string name) : name_(name) {}

  enum OperationDirection {DIR_X, DIR_Y, DIR_Z, DIR_YAW, DIR_PITCH, DIR_ROLL};

  void contactWrenchOptimize();

  /**
   * @brief operation
   * @param dir Direction we want to move
   * @param delta
   * @return Whether it is possible operation.
   */
  bool operate(OperationDirection dir, double delta);

protected:
  std::string name_;
  Eigen::Isometry3d transform_;
  std::vector < ContactPtr > contact_candidate_;
  std::vector < ContactPtr > contact_environment_;
  std::shared_ptr< fcl::ShapeBase > fcl_model_;



  int sample_; ///< number of samples

  double mass_;
  double friction_;

  virtual void sample() = 0;
};


class BoxContactModel : public ContactModel
{
public:
  BoxContactModel(const Eigen::Vector3d &dimension);
  BoxContactModel(const std::string &name, const Eigen::Vector3d &dimension);

  // constraints? how?
protected:
  virtual void sample();
private:
  Eigen::Vector3d dimension_; ///< width, length, height

};

typedef std::shared_ptr<ContactModel> ContactModelPtr;




class PlanningScene
{
  ContactModelPtr target_object_; ///< Object we want to check whether it collide with any env.
  std::vector< std::shared_ptr<fcl::ShapeBase> > objects_; ///< Environment objects (should not be changed)

  inline void setTargetObject(const ContactModelPtr &target_object) { target_object_ = target_object; }
  bool isPossible();  ///< return whether collision between objects is free
};


class RobotDynamicsModel
{
  virtual bool isPossible(Eigen::Vector3d position);

};

template <class T>
class ContactModelGraph
{
  std::vector<int> adjacency_list_;
  std::vector<std::shared_ptr<T> > contact_models_;
  std::vector<std::shared_ptr<T> > result_;

  PlanningScene planning_scene_;

  int max_depth_;
  double discrete_resolution_;

public:

  ContactModelGraph() : discrete_resolution_(0.05) {}

  void setStart(const std::shared_ptr<T> &start);
  void setGoal(const std::shared_ptr<T> &goal);
  void makeGraph()
  {
    contact_models_.clear();
    contact_models_.push_back(start_);

    makeTree(0, 0);
  }

private:
  void makeTree(int current_index, int depth)
  {
      auto current_node = contact_models_[current_index];

      for(int dir=ContactModel::DIR_X; dir<ContactModel::DIR_YAW; dir++)
      {
        auto new_node = std::make_shared<T>(*current_node);
        if(new_node->operate(dir, discrete_resolution_)) // Operation is available
        {
          int index = contact_models_.size();
          contact_models_.push_back(new_node);
          // adjacency_list_
        }
      }
  }
  ContactModelPtr start_; // Initial contact information should be provided.
  ContactModelPtr goal_; // Goal contact information should be provided.

};


class ContactPlanner
{
  // Pose graph
  ContactModelPtr contact_model_;

  void solve();


};
/*
#include <iostream>
#include <string>
#include <unordered_map>
#include <list>

using namespace std;

int main()
{
    int vertices, edges, weight;
    string v1, v2;

    printf("Enter the Number of Vertices -\n");
    cin >> vertices;

    printf("Enter the Number of Edges -\n");
    cin >> edges;

    // Adjacency List is a map of <string, list>.
    // Where each element in the list is pair<string, int>
    // pair.first -> the edge's destination (string)
    // pair.second -> edge's weight
    unordered_map< string, list< pair<string, int> > > adjacencyList(vertices + 1);

    printf("Enter the Edges V1 -> V2, of weight W\n");
    for (int i = 1; i <= edges; ++i) {
        cin >> v1 >> v2 >> weight;

        // Adding Edge to the Directed Graph
        adjacencyList[v1].push_back(make_pair(v2, weight));
    }

    // Printing Adjacency List
    cout << endl << "The Adjacency List-" << endl;
    for (auto& value : adjacencyList) {
        string vertex = value.first;
        list< pair<string, int> > adjacentVertices = value.second;
        list< pair<string, int> >::iterator itr = adjacentVertices.begin();

        cout << "adjacencyList[" << vertex << "]";

        while (itr != adjacentVertices.end()) {
            cout << " -> " << (*itr).first << " (" << (*itr).second << ")";
            ++itr;
        }

        cout << endl;
    }

    return 0;
}

*/
}
#endif
