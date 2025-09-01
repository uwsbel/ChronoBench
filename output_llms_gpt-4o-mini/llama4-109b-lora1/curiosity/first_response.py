import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.vehicle.utils as veh_utils
import math

# Initialize PyChrono environment and core components
ch = chrono.ChEngine('SCN')
ch.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
ch.SetSolverType(chrono.ChSolver.Type_PYMKL)
ch.SetSolverIter(100)

# Create the ground with collision properties
ground = veh.GetDatafile('terrain/terrain_assets/ground/ground.json')
terrain = veh.ChTerrain()
terrain.SetGroundMaterial(chrono.ChContactMaterialData().GetDefault())
terrain.Initialize(ground)
ch.Add(terrain.GetGroundBody())

# Create the rover model with motor control driver
rover = veh.ChCuriosityRover()
rover.Initialize()
rover.SetChassisVisualizationType(veh.VisualizationType_MESH)
rover.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
rover.SetSteeringVisualizationType(veh.VisualizationType_MESH)
rover.SetWheelVisualizationType(veh.VisualizationType_MESH)
rover.SetChassisCollisionType(veh.CollisionType_BOX)
rover.SetSuspensionCollisionType(veh.CollisionType_BOX)
rover.SetSteeringCollisionType(veh.CollisionType_BOX)
rover.SetWheelCollisionType(veh.CollisionType_BOX)
ch.Add(rover.GetChassisBody())

# Create the motor control driver
driver = veh.ChWheeledVehicleIrrApp(rover, 'CuriosityRover')
driver.SetSteeringDelta(0.5)
driver.SetTireVisualizationType(veh.VisualizationType_MESH)
driver.SetChassisVisualizationType(veh.VisualizationType_MESH)
driver.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
driver.SetSteeringVisualizationType(veh.VisualizationType_MESH)
driver.SetWheelVisualizationType(veh.VisualizationType_MESH)

# Initialize the Irrlicht visualization system
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(rover)
vis.SetWindowTitle('Curiosity Rover Simulation')
vis.SetWindowSize(1280, 720)
vis.SetRenderMode(irr.RenderMode_SOLID)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 0.5), chrono.ChVector3d(0, 0, 0))
vis.AddLightDirectional(chrono.ChVector3d(0.5, 0.5, 0.5), chrono.ChVector3d(-0.5, -0.5, -0.5), True, 0.8)
vis.AddLightPoint(chrono.ChVector3d(0, 0, 0.5), chrono.ChVector3d(0, 0, 0), 0.8)

# Set simulation parameters
rover.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.ChQuaterniond(1, 0, 0, 0)))
driver.SetStart(chrono.ChTime())
driver.SetTireStepSize(0.01)
driver.SetTireMaxSpeed(10)
driver.SetTireMaxSteerAngle(math.pi / 4)

# Run the simulation
while vis.Run() and not driver.GetQuitRequest():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    ch.DoStepSimulation(1e-3)
    driver.Synchronize(chrono.ChTime())
    driver.Advance(1e-3)
    vis.Synchronize(driver.GetTimeStep())
    vis.UpdateCamera()
    vis.UpdateLigths()
    vis.UpdateShadows()