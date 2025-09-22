import pychrono as chrono
import pychrono.robot as robot
from pychrono import irrlicht as chronoirr

def main():
    

    step_size = 0.005

    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    sys.GetSolver().AsIterative().SetMaxIterations(150)
    sys.SetMaxPenetrationRecoverySpeed(4.0)

    
    ground_mat = chrono.ChContactMaterialNSC()
    ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
    ground.SetPos(chrono.ChVector3d(0, 0, -1))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    sys.Add(ground)

    
    turtlebot = robot.Turtlebot()
    turtlebot.SetInitPosition(chrono.ChFramed(chrono.ChVector3d(0, -0.2, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
    turtlebot.SetMotorType(robot.MotorModelType_SHAFTS)
    turtlebot.SetWheelType(robot.WheeldriverTypeWV)
    turtlebot.SetTireType(robot.TireModelType_TMEASY)
    turtlebot.SetDriveline8WD(True)
    turtlebot.Initialize(sys)

    turtlebot.SetChassisVisualizationType(robot.VisualizationType_PRIMITIVES)
    turtlebot.SetSuspensionVisualizationType(robot.VisualizationType_PRIMITIVES)
    turtlebot.SetSteeringVisualizationType(robot.VisualizationType_PRIMITIVES)
    turtlebot.SetWheelVisualizationType(robot.VisualizationType_NONE)
    turtlebot.SetTireVisualizationType(robot.VisualizationType_PRIMITIVES)

    turtlebot.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(turtlebot.GetSystem())
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Turtlebot')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddCamera(chrono.ChVector3d(0.0, 0.0, 1.75), chrono.ChVector3d(0.0, 0.0, 0.0))
    vis.AddTypicalKeyboardBindings()

    
    time = 0
    time_end = 100

    turtlebot.SetChassisOutput(True)
    turtlebot.SetSuspensionOutput(True)
    turtlebot.SetSteeringOutput(True)
    turtlebot.SetWheelOutput(True)
    turtlebot.SetTireOutput(True)

    while vis.Run() :
        time = turtlebot.GetSystem().GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver_inputs = turtlebot.GetDriverInputs()

        
        turtlebot.Synchronize(time)
        vis.Synchronize(time, driver_inputs)

        
        turtlebot.Advance(step_size)
        vis.Advance(step_size)

        if (time < time_end):
            turtlebot.GetSystem().DoStepDynamics(step_size)

    return 0


if __name__ == "__main__":
    main()