from openravepy import *
import numpy as np
import openravepy as orpy
import time
# from osr_openrave import kinematics, planning
env = orpy.Environment()
env.SetViewer('qtcoin')
env.Load('/workspace/catkin_ws/src/osr_course_pkgs/osr_openrave/worlds/pick_and_place.env.xml')
env.SetCollisionChecker(RaveCreateCollisionChecker(env, 'ode'))
def create_box(T, color = [0, 0.6, 0]):
  box = orpy.RaveCreateKinBody(env, '')
  box.SetName('box')
  box.InitFromBoxes(np.array([[0,0,0,0.035,0.03,0.005]]), True)
  g = box.GetLinks()[0].GetGeometries()[0]
  g.SetAmbientColor(color)
  g.SetDiffuseColor(color)
  box.SetTransform(T)
  env.Add(box,True)
  return box
T = np.eye(4)
container_center = np.array([0.4, 0.2, 0.195])
# Destination
T[:3, 3] = container_center + np.array([0, -0.4, 0])
destination0 = create_box(T, color = [0, 0, 0.6])
T[:3, 3] = container_center + np.array([0, -0.5, 0])
destination1 = create_box(T, color = [0, 0, 0.6])
# Generate random box positions
boxes = []
nbox_per_layer = 2
n_layer = 20
h = container_center[2]
for i in range(n_layer):
  nbox_current_layer = 0
  while nbox_current_layer < nbox_per_layer:
    theta = np.random.rand()*np.pi
    T[0, 0] = np.cos(theta)
    T[0, 1] = -np.sin(theta)
    T[1, 0] = np.sin(theta)
    T[1, 1] = np.cos(theta)
    T[0, 3] = container_center[0] - 0.03 + (np.random.rand()-0.5)*0.17
    T[1, 3] = container_center[1] + (np.random.rand()-0.5)*0.08
    T[2, 3] = h
    box = create_box(T)
    if env.CheckCollision(box):
      env.Remove(box)
    else:
      boxes.append(box)
      nbox_current_layer += 1
  h += 0.011

def path_planner_and_exec(grasp):
  planner = orpy.RaveCreatePlanner(env, 'birrt') # Using bidirectional RRT
  params = orpy.Planner.PlannerParameters()
  params.SetRobotActiveJoints(robot)
  params.SetGoalConfig(grasp)
  params.SetExtraParameters('<_postprocessing planner="ParabolicSmoother"><_nmaxiterations>40</_nmaxiterations></_postprocessing>')
  planner.InitPlan(robot, params)
  traj = orpy.RaveCreateTrajectory(env, '')
  planner.PlanPath(traj)
  return traj

robot = env.GetRobots()[0]
manipulator = robot.SetActiveManipulator('gripper')
robot.SetActiveDOFs(manipulator.GetArmIndices())
np.set_printoptions(precision=6, suppress=True)

effector_ori_transform = manipulator.GetEndEffectorTransform()
turn_effector_down = matrixFromAxisAngle([0,np.pi,0])
effector_transform = np.dot(turn_effector_down,effector_ori_transform)

iktype=orpy.IkParameterization.Type.Transform6D
ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=iktype)
if not ikmodel.load():
  ikmodel.autogenerate()

number_of_boxes_placed = 0
number_of_boxes_deleted = 0

number_of_boxes = 39
while number_of_boxes > 10:
  
  need_to_turn = 0
  # raw_input('Press Enter to start process')
  print ('Now is box'), number_of_boxes
  box = boxes[number_of_boxes]
  box_transform = box.GetTransform()
  print box_transform
  box_transform[:3,:3] = np.transpose(box_transform[:3,:3])

  effector_according_box = np.eye(4)
  effector_according_box[:3,:3] = np.dot(effector_transform[:3,:3], box_transform[:3,:3])
  effector_according_box[:3,3] = box_transform[:3,3]
  effector_according_box[:3,3] = np.add(effector_according_box[:3,3], np.array([0,0,0.008]))
  print effector_according_box

  ikparam = orpy.IkParameterization(effector_according_box, iktype)
  # from IPython.terminal import embed; ipshell=embed.InteractiveShellEmbed(config=embed.load_default_config())(local_ns=locals())
  solutions_with_col = manipulator.FindIKSolutions(ikparam, orpy.IkFilterOptions.CheckEnvCollisions)
  print solutions_with_col
  if len(solutions_with_col) == 0:
    solutions_without_col = manipulator.FindIKSolutions(effector_according_box, 0)
    print solutions_without_col

  if len(solutions_with_col) == 0 and len(solutions_without_col) == 0:
    env.Remove(box)
    number_of_boxes_deleted += 1
    print("bobian la...")

  if len(solutions_with_col) == 0 and len(solutions_without_col) != 0:
    need_to_turn = 1
    turn_effector_90 = matrixFromAxisAngle([0,0,np.pi/2])
    effector_according_box[:3,:3] = np.dot(effector_according_box[:3,:3], turn_effector_90[:3,:3])
    solutions_with_col = manipulator.FindIKSolutions(effector_according_box, orpy.IkFilterOptions.CheckEnvCollisions)
    print solutions_with_col

    if len(solutions_with_col) == 0:
      env.Remove(box)
      number_of_boxes_deleted += 1
      print("this one really bobian la...")

  if len(solutions_with_col) != 0:
    traj = path_planner_and_exec(solutions_with_col[0])
    controller = robot.GetController()
    # print controller
    controller.SetPath(traj)
    robot.WaitForController(0)
    
    taskprob = interfaces.TaskManipulation(robot) # create the interface for task manipulation programs
    taskprob.CloseFingers() # close fingers until collision
    robot.WaitForController(0) # wait
    robot.Grab(box)
    robot.WaitForController(0)

    move_up = 0.001 * (20-number_of_boxes/2)
    twist = np.array([0, 0, move_up, 0, 0, 0])
    link_idx = [l.GetName() for l in robot.GetLinks()].index('robotiq_85_base_link')
    link_origin = robot.GetLink('robotiq_85_base_link').GetTransform()[:3,3]
    print link_origin
    J = np.zeros((6,6))
    q = robot.GetActiveDOFValues()
    for i in range(10):
      J[:3,:] = robot.ComputeJacobianTranslation(link_idx, link_origin)[:,:6]
      J[3:,:] = robot.ComputeJacobianAxisAngle(link_idx)[:,:6]
      qdot = np.linalg.solve(J, twist)
      print qdot
      q[:6] += qdot
      robot.SetActiveDOFValues(q)
      # raw_input('Press Enter for next differential IK step')
      time.sleep(0.1)

    effector_according_dest = np.eye(4)

    if number_of_boxes > 24:
      destination_stack = destination1.GetTransform()
      effector_according_dest[:3,:3] = np.dot(effector_transform[:3,:3], destination_stack[:3,:3])
      effector_according_dest[:3,3] = destination_stack[:3,3] + np.array([0,0,0.008]) + np.array([0,0,0.012*(40-number_of_boxes+number_of_boxes_deleted)])
    
    else:
      destination_stack = destination0.GetTransform()
      effector_according_dest[:3,:3] = np.dot(effector_transform[:3,:3], destination_stack[:3,:3])
      effector_according_dest[:3,3] = destination_stack[:3,3] + np.array([0,0,0.008]) + np.array([0,0,0.012*(25-number_of_boxes)])
    
    print effector_according_dest
    if need_to_turn == 1:
      turn_effector_90 = matrixFromAxisAngle([0,0,np.pi/2])
      effector_according_dest[:3,:3] = np.dot(effector_according_dest[:3,:3], turn_effector_90[:3,:3])

    ikparam = orpy.IkParameterization(effector_according_dest, iktype)
    solutions_for_dest = manipulator.FindIKSolutions(ikparam, orpy.IkFilterOptions.CheckEnvCollisions)
    print solutions_for_dest

    solutions_for_dest_without_col = manipulator.FindIKSolutions(ikparam, 0)
    print solutions_for_dest

    if len(solutions_for_dest) == 0:
      # raw_input('wait...')
      taskprob.ReleaseFingers()
      robot.WaitForController(0)
      robot.Release(box)
      robot.WaitForController(0)
      env.Remove(box)
      number_of_boxes_deleted += 1

    if len(solutions_for_dest) != 0:
      qplace = solutions_for_dest[0]
      traj = path_planner_and_exec(qplace)
      controller = robot.GetController()
      # print controller
      controller.SetPath(traj)
      robot.WaitForController(0)
      taskprob.ReleaseFingers()
      robot.WaitForController(0)
      robot.Release(box)
      robot.WaitForController(0)
      number_of_boxes_placed += 1

    print('Number of boxes placed is'), number_of_boxes_placed
    print('Number of boxes deleted is'), number_of_boxes_deleted

  number_of_boxes -= 1
  # raw_input('Next Box')

while True:
  time.sleep(1)