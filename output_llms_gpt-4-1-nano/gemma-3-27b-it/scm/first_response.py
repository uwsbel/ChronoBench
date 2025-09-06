import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.scm as scm

# --- Simulation Setup ---
sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# --- Visualization ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV on SCM Terrain')
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(10, 5, -15))
vis.AddTypicalLights()

# --- Terrain ---
terrain = scm.SCM_Terrain()
terrain.Create(sys, 100, 100, 10, 10)  # xsize, ysize, nx, ny
terrain.SetSoilParameters(0.5, 5000, 0.8, 0.1, 0.9) # rho, Young, Poisson, damping, friction
terrain.SetDrawMesh(True)
terrain.SetMeshDensity(2)
terrain.SetDrawSinkage(True)
terrain.SetSinkageColorMap(True)

# --- Vehicle ---
hmmwv = veh.HMMWV(sys)
init_location = chrono.ChVector3d(0, 0.5, 0)
init_rotation = chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0))
hmmwv.SetInitPosition(chrono.ChCoordsysD(init_location, init_rotation))
hmmwv.Initialize()

# Set rigid tire model
for i in range(4):
    hmmwv.GetWheel(i).SetTireModel(veh.HMMWV_Tire::Model::RIGID)

# Mesh visualization for all components
for i in range(hmmwv.GetNumBody()):
    body = hmmwv.GetBody(i)
    if body:
        body.SetCollide(True)
        body.SetVisualizationType(chrono.ChVisualizationType.MESH)
        body.GetCollisionModel().SetSuggestedEnvelope(0.005)
        body.GetCollisionModel().SetSuggestedMargin(0.005)


# --- Moving Patch ---
patch = terrain.AddPatch(hmmwv.GetBody(), chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 2, 2)
patch.SetMoving(True)

# --- Driver System ---
driver = veh.HMMWV_Driver()
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)
hmmwv.SetDriver(driver)

# --- Simulation Loop ---
time_step = 0.005
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update driver inputs (example)
    if vis.GetSystem().GetChTime() < 10:
        driver.SetSteering(0.2)
        driver.SetThrottle(0.5)
        driver.SetBraking(0.0)
    else:
        driver.SetSteering(0.0)
        driver.SetThrottle(0.0)
        driver.SetBraking(0.1)

    sys.DoStepDynamics(time_step)