import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# ---------------------------------------------------------------------
# Create the Chrono system and set default parameters
# ---------------------------------------------------------------------
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
chrono.SetChronoDataPath('./chrono_data/')

# ---------------------------------------------------------------------
# Create the HMMWV vehicle and set parameters
# ---------------------------------------------------------------------
vehicle = veh.HMMWV_Full('./hmmwv/hmmwv_chassis.obj', system)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_PRIMITIVES)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.6), chrono.QUNIT))
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.Initialize()

# ---------------------------------------------------------------------
# Create the terrain and set parameters
# ---------------------------------------------------------------------
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChMaterialSurfaceSMC(), 
                       chrono.CSYSNORM, 
                       200, 200)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# ---------------------------------------------------------------------
# Create the interactive driver system
# ---------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vehicle)
driver.Initialize()

# ---------------------------------------------------------------------
# Create the Irrlicht application and set parameters
# ---------------------------------------------------------------------
application = chronoirr.ChIrrApp(system, 'HMMWV Simulation', chronoirr.dimension2du(1024,768))
application.AddTypicalLights()
application.AddSkyBox()
application.AddTypicalCamera(chronoirr.vector3df(20, -20, 5), chronoirr.vector3df(0, 0, 1))
application.AssetBindAll()
application.AssetUpdateAll()

# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------
application.SetTimestep(0.02) # 50 fps

while application.GetDevice().run():
    time = system.GetChTime()
    
    # Update driver inputs
    driver.Synchronize(time)
    
    # Advance simulation by one step
    system.DoStepDynamics(application.GetTimestep())
    
    # Update the visualization
    application.BeginScene()
    application.DrawAll()
    application.EndScene()