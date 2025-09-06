import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as chronoirr
import math

# 1. Initialize the PyChrono environment
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
sys.SetTolerance(1e-6)

# Set collision system to Bullet for better performance
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# 2. Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover Simulation")
vis.SetSymbolsScale(0.002)
vis.SetShadows(True)
vis.SetLightIntensity(2.0)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chronoirr.ChVectorD(5, 3, 5))
vis.AddTypicalLights()
vis.AttachSystem(sys)
vis.Initialize()
vis.GetView().SetFov(60)
vis.GetView().SetZoom(2)
vis.GetView().SetFullView()

# 3. Terrain setup
terrain = vehicle.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.05)
terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)), 100, 100)
terrain.Initialize()

# 4. Curiosity rover setup
rover = vehicle.Curiosity(sys)  # Assuming a predefined model (if not available, create custom)
rover.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0,1,0))))
rover.SetChassisCollisionType(vehicle.ChassisCollisionType.BOX)
rover.SetWheelType(vehicle.WheelType.RIGID)
rover.SetTireType(vehicle.TireType.SIMPLE)
rover.SetSuspensionType(vehicle.SuspensionType.INDEPENDENT_ROCKER_BOGIE)
rover.Initialize()

# Add visualization assets
rover.GetChassis().AddVisualShape(chrono.ChVisualShapeBox(2, 1, 0.5, False, chrono.ChColor(0.5, 0.5, 0.5)))
for wheel in rover.GetWheelSystems():
    wheel.GetWheelBody().AddVisualShape(chrono.ChVisualShapeCylinder(0.2, 0.4, False, chrono.ChColor(0.2, 0.2, 0.2)))

# 5. Driver setup for real-time steering
driver = vehicle.RobotDriver()
driver.SetThrottle(0.5)  # Constant forward throttle
driver.SetSteering(-0.2)  # Left turn
rover.SetDriver(driver)

# 6. Simulation loop
time_step = 0.01
while vis.Run():
    sys.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()