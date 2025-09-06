import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# Initialize the PyChrono system
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))

# Set up the rigid terrain
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(), 100, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
patch.SetTexture(chrono.GetChronoDataFile("textures/terrain/red_clay.jpg"))
terrain.Initialize()

# Create and configure the Curiosity rover
rover = veh.Curiosity(sys)
rover.SetChassisCollisionType(veh.Curiosity.CollisionType_ALL)
rover.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
rover.Initialize()

# Set up the interactive driver for real-time control
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity Rover Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, -10))
vis.AddTypicalLights()
vis.EnableShadows()

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)
rover.SetDriver(driver)

# Simulation loop
time_step = 0.01
while vis.Run():
    # Synchronize driver inputs
    driver.Synchronize(sys.GetChTime())
    
    # Advance the simulation
    sys.DoStepDynamics(time_step)
    
    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()