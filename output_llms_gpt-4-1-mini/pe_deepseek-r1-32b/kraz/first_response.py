import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# Initialize the simulation system
system = chrono.ChSystemSMC()
system.SetTimeStep(0.001)
system.SetGravity(chrono.ChVectorD(0, -9.81, 0))
system.GetSolver().SetMaxIterations(20)

# Create the Kraz vehicle
kraz = veh.Kraz()
kraz.SetContactMethod(chrono.ChContactMethod_NSC)
kraz.SetChassisCollisionType(veh.VehicleCollisionType_NONE)
kraz.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
kraz.Initialize()

# Create the driver system
driver = veh.RS_Driver()
driver.Initialize(kraz.GetDriverInputs())
kraz.SetDriver(driver)

# Create the rigid terrain
terrain = veh.RigidTerrain(system)
terrain.SetMaterial(veh.ChTerrainMaterial(0.5, 0.3))  # Friction, restitution
terrain_patch = terrain.AddPatch(chrono.ChVectorD(0, 0, 0), 100, 100)
terrain.Initialize()

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Kraz Vehicle Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 5, 10))
vis.AddTypicalLights()

# Simulation loop
simulation_time = 0
render_step = 0

while simulation_time < 10:
    # Advance simulation
    system.DoStepDynamics(system.GetTimeStep())
    simulation_time += system.GetTimeStep()
    
    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Check for exit condition
    if vis.Run() == False:
        break
    
    # Synchronize with real time
    render_step += 1
    if render_step >= 100:
        render_step = 0
        elapsed_time = system.GetChTime()
        print(f"Elapsed time: {elapsed_time:.2f}s")

# Cleanup
vis.Close()