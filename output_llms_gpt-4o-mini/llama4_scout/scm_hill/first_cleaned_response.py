import pychrono as chrono
import pychrono.irrlicht as irr
from pychrono.vehicle import HMMWV, SCMDeformableTerrain


chrono.ChEngine.Initialize()


SIMULATION_NAME = "HMMWV on SCM Deformable Terrain"
TIME_STEP = 0.01  
TOTAL_TIME = 10.0  


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


system.SetCollisionSystem(chrono.ChCollisionSystemBruteForce())
system.GetCollisionSystem().SetBroadphaseMethod(chrono.ChCollisionSystem.BroadphaseMethod.SweepAndPrune)


hmmwv = HMMWV(system)


hmmwv.SetInitialPosition(chrono.ChVector3d(0, 0, 1.0))
hmmwv.SetInitialVelocity(chrono.ChVector3d(0, 0, 0))


hmmwv.EnableVisualization(True)


terrain = SCMDeformableTerrain(system)


soil_params = chrono.ChSoilParams()
soil_params.SetFrictionAngle(30.0)  
soil_params.SetCohesion(10.0)  
soil_params.SetDensity(1500.0)  
terrain.SetSoilParams(soil_params)


height_map = []
for i in range(100):
    row = []
    for j in range(100):
        row.append(0.5 * (1.0 + chrono.ChMath.cos(2 * chrono.ChMath.pi * i / 100.0)) * (1.0 + 0.1 * j / 100.0))
    height_map.append(row)
terrain.Initialize(height_map)


driver = chrono.ChVehicleDriver(hmmwv.GetChassisBody())


driver.SetThrottleInput(0.5)
driver.SetSteeringInput(0.0)


app = irr.ChIrrApp(system, SIMULATION_NAME, 800, 600)


app.SetCameraPosition(chrono.ChVector3d(5.0, -5.0, 2.0))
app.SetCameraTarget(chrono.ChVector3d(0.0, 0.0, 0.5))
app.EnableSunlight(True)


app.Show()


while (app.GetSimulationTime() < TOTAL_TIME) and (not app.IsStopped()):
    
    hmmwv.Synchronize(TIME_STEP)
    terrain.Synchronize(TIME_STEP)
    driver.Synchronize(TIME_STEP)
    app.Synchronize("", TIME_STEP)
    app.Advance(TIME_STEP)

    
    app.SpinEventLoop()


app.EndSimulation()
chrono.ChEngine.Finalize()