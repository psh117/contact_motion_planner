# contact_motion_planner
contact_motion_planner

[![CodeFactor](https://www.codefactor.io/repository/github/psh117/contact_motion_planner/badge)](https://www.codefactor.io/repository/github/psh117/contact_motion_planner)


A library for motion planning considering contact state

# Dependency
* fcl (https://github.com/flexible-collision-library/fcl)
* Eigen3
* qpOASES
* ROS

# How to use

First, create a model to move and set a start and a goal.
```cpp
  Eigen::Vector3d dim;
  dim << 0.2, 0.2, 0.2;
  shared_ptr<BoxContactModel> start = make_shared<BoxContactModel>("box", dim);
  shared_ptr<BoxContactModel> goal = make_shared<BoxContactModel>("box", dim);
  start_transform.linear() = Eigen::Matrix3d::Identity();
  goal_transform.linear() = Eigen::Matrix3d::Identity() * Eigen::AngleAxisd(90 * 3.141592/180., Eigen::Vector3d::UnitZ());
  start_transform.translation() << 0.0, 0.0, 0.1;
  goal_transform.translation() << .6, 0.5, .1;
  
  start->setTransform(start_transform);
  goal->setTransform(goal_transform);
```

And, fill the environment with some obstacles.
```cpp
  shared_ptr<fcl::Box> env_table1 = make_shared<fcl::Box>(0.7, 0.7, 0.05);
  shared_ptr<fcl::Box> env_obstacle1 = make_shared<fcl::Box>(0.1, 0.1, 0.5);
  Eigen::Affine3d env_table1_transform;
  Eigen::Affine3d env_obstacle1_transform;
  Eigen::Affine3d start_transform;
  Eigen::Affine3d goal_transform;

  env_table1_transform.translation()    << .0, .0, -0.0251;
  env_obstacle1_transform.translation() << .25, .0, .25;
  env_table1_transform.linear() = Eigen::Matrix3d::Identity();
  env_obstacle1_transform.linear() = Eigen::Matrix3d::Identity();
```
Finally, get an object of ContactModelGraph and make the object pose graph.

```cpp
  ContactModelGraph<BoxContactModel> g;
  PlanningScenePtr scene = make_shared<PlanningScene>();
  RobotDynamicsModelPtr robot_model = make_shared<DexterousRobotModel>();
  scene->addSceneObject(env_table1, env_table1_transform);
  scene->addSceneObject(env_obstacle1, env_obstacle1_transform);
  g.setRobotDynamicsModel(robot_model);
  g.setPlanningScene(scene);
  g.setStart(start);
  g.setGoal(goal);
  g.makeObjectPoseGraph();
```
