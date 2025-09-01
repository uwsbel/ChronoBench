import pychrono as ch
import pychrono.ros as chros

def main():
    
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))
    sys.SetSolverType(ch.ChSolver.Type_BARZILAIBORWEIN)
    sys.GetSolver().AsIterative().SetMaxIterations(150)
    sys.SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)

    
    ground_mat = ch.ChContactMaterialNSC()
    ground_mat.SetFriction(0.5)
    ground = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, ground_mat)
    ground.SetPos(ch.ChVector3d(0, 0, -1))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetTexture(ch.GetChronoDataFile("textures/concrete.jpg"))
    sys.Add(ground)

    
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, ground_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))
    box.SetMass(30)
    sys.Add(box)

    
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSDynamicReconfigureHandler(UpdateParams))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))
    ros_manager.RegisterHandler(chros.ChROSTFHandler(25, "~/transform"))
    ros_manager.RegisterHandler(MyCustomHandler())
    ros_manager.Initialize()

    
    time = 0
    time_step = 1e-3
    time_end = 30

    box_vel = box.GetLinVel()

    while time < time_end:
        time = sys.GetChTime()

        sys.DoStepDynamics(time_step)

        if not ros_manager.Update(time, time_step):
            break

        if abs(box_vel.x - box.GetLinVel().x) > 0.1:
            print("Box velocity x:", box.GetLinVel().x)
            box_vel = box.GetLinVel()

        if abs(box_vel.y - box.GetLinVel().y) > 0.1:
            print("Box velocity y:", box.GetLinVel().y)
            box_vel = box.GetLinVel()

    return 0

def UpdateParams(config, level):
    print("Updating params:", config)
    return True

class MyCustomHandler(chros.ChROSHandler):
    def __init__(self):
        super().__init__()
    def Initialize(self, manager):
        print("Initializing custom handler")
    def Update(self, time, step_size):
        print("Publishing from custom handler")
        return True

main()