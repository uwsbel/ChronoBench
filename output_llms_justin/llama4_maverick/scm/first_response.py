import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.pardisomkl as mkl

# Initialize the PyChrono environment and core components
print("Copyright (c) 2023 ProjectChrono")

# Create the HMMWV vehicle
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVector3d(-5, -5, 0.5), chrono.Q_from_AngZ(0)))
vehicle.SetInitFwdVel(0)
vehicle.SetTireType(veh.TireType_Rigid)
vehicle.SetTireStepSize(1e-3)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# Create the SCM terrain
terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e8, 0.01, 2e6, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0)
terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(2.0, 6.0, 0.04)

# Create the moving patch
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(4.0, 2.5, 0.1))

# Create the Irrlicht application
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV on SCM Deformable Terrain')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the interactive driver system
driver = veh.ChIrrGuiDriver(vis.GetDevice(), vehicle.GetVehicle())
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)

# Simulation loop
while vis.Run() :
    time = vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.Advance(time, driver_inputs)
    terrain.Advance(time)
    vis.Advance(time)
    vis.Render()

# Run the simulation at 50 frames per second
chrono.ChRealtimeStep(vehicle.GetSystem(), 1 / 50)

# The above while and ChRealtimeStep should be used together like this:
while vis.Run() :
    time = vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.Advance(time, driver_inputs)
    terrain.Advance(time)
    vis.Advance(time)
    vis.Render()
    chrono.ChRealtimeStep(vehicle.GetSystem(), 1 / 50)

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.pardisomkl as mkl

print("Copyright (c) 2023 ProjectChrono")

vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVector3d(-5, -5, 0.5), chrono.Q_from_AngZ(0)))
vehicle.SetInitFwdVel(0)
vehicle.SetTireType(veh.TireType_Rigid)
vehicle.SetTireStepSize(1e-3)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e8, 0.01, 2e6, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0)
terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(2.0, 6.0, 0.04)

terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(4.0, 2.5, 0.1))

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV on SCM Deformable Terrain')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

driver = veh.ChIrrGuiDriver(vis.GetDevice(), vehicle.GetVehicle())
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.Advance(time, driver_inputs)
    terrain.Advance(time)
    vis.Advance(time)
    vis.Render()
    chrono.ChRealtimeStep(vehicle.GetSystem(), 1 / 50)