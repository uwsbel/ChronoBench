import pychrono as chrono
import pychrono.robot as robot
import math


system = chrono.ChSystemNSC()


robot = robot.UR10(system)
robot.SetInitPos(chrono.ChVector3d(0, 0, 0))
robot.SetGripper(0.01)
robot.SetFixed(False)
robot.SetCollide(False)
robot.SetGripperCollide(False)
robot.SetVerbose(False)
robot.SetGripperMaxForce(10)


robot.Initialize()


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(10, 10, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


box = chrono.ChBodyEasyBox(0.25, 0.25, 0.25, 1000, True, True, ground_mat)
box.SetPos(chrono.ChVector3d(0, 0, 0.5))
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box)


time_step = 1e-3
time_end = 20


target_pos = chrono.ChVector3d(0, 0, 0.5)
target_rot = chrono.ChQuaterniond(1, 0, 0, 0)


sim_time = 0


while sim_time < time_end:
    
    if sim_time < 2:
        robot.SetJointTarget(0, 0, 0, 0)
        robot.SetJointTarget(1, 0, 0, 0)
        robot.SetJointTarget(2, 0, 0, 0)
        robot.SetJointTarget(3, 0, 0, 0)
        robot.SetJointTarget(4, 0, 0, 0)
        robot.SetJointTarget(5, 0, 0, 0)
    elif sim_time < 4:
        robot.SetJointTarget(0, 0, 0, 0)
        robot.SetJointTarget(1, 0, 0, 0)
        robot.SetJointTarget(2, 0, 0, 0)
        robot.SetJointTarget(3, 0, 0, 0)
        robot.SetJointTarget(4, 0, 0, 0)
        robot.SetJointTarget(5, -math.pi / 2, 0, 0)
    elif sim_time < 6:
        robot.SetJointTarget(0, 0, 0, 0)
        robot.SetJointTarget(1, 0, 0, 0)
        robot.SetJointTarget(2, 0, 0, 0)
        robot.SetJointTarget(3, 0, 0, 0)
        robot.SetJointTarget(4, -math.pi / 2, 0, 0)
        robot.SetJointTarget(5, -math.pi / 2, 0, 0)
    elif sim_time < 8:
        robot.SetJointTarget(0, 0, 0, 0)
        robot.SetJointTarget(1, 0, 0, 0)
        robot.SetJointTarget(2, 0, 0, 0)
        robot.SetJointTarget(3, -math.pi / 2, 0, 0)
        robot.SetJointTarget(4, -math.pi / 2, 0, 0)
        robot.SetJointTarget(5, -math.pi / 2, 0, 0)
    elif sim_time < 10:
        robot.SetJointTarget(0, 0, 0, 0)
        robot.SetJointTarget(1, 0, 0, 0)
        robot.SetJointTarget(2, -math.pi / 2, 0, 0)
        robot.SetJointTarget(3, -math.pi / 2, 0, 0)
        robot.SetJointTarget(4, -math.pi / 2, 0, 0)
        robot.SetJointTarget(5, -math.pi / 2, 0, 0)
    elif sim_time < 12:
        robot.SetJointTarget(0, 0, 0, 0)
        robot.SetJointTarget(1, -math.pi / 2, 0, 0)
        robot.SetJointTarget(2, -math.pi / 2, 0, 0)
        robot.SetJointTarget(3, -math.pi / 2, 0, 0)
        robot.SetJointTarget(4, -math.pi / 2, 0, 0)
        robot.SetJointTarget(5, -math.pi / 2, 0, 0)
    elif sim_time < 14:
        robot.SetJointTarget(0, -math.pi / 2, 0, 0)
        robot.SetJointTarget(1, -math.pi / 2, 0, 0)
        robot.SetJointTarget(2, -math.pi / 2, 0, 0)
        robot.SetJointTarget(3, -math.pi / 2, 0, 0)
        robot.SetJointTarget(4, -math.pi / 2, 0, 0)
        robot.SetJointTarget(5, -math.pi / 2, 0, 0)
    else:
        robot.SetJointTarget(0, 0, 0, 0)
        robot.SetJointTarget(1, 0, 0, 0)
        robot.SetJointTarget(2, 0, 0, 0)
        robot.SetJointTarget(3, 0, 0, 0)
        robot.SetJointTarget(4, 0, 0, 0)
        robot.SetJointTarget(5, 0, 0, 0)

    
    robot.UpdateKinematics()

    
    sim_time += time_step

    
    system.DoStepDynamics(time_step)