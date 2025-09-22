import chrono
import chrono.irrlicht as chronoirr
import chrono.vehicle as veh
import numpy as np

# --- Initialize the PyChrono environment ---
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# --- Create the deformable terrain ---
terrain = chrono.ChTerrain()
terrain.Set_Size_X(100)
terrain.Set_Size_Z(100)
terrain.Set_Height_map(chrono.ChVectorD(100, 100))
terrain.Initialize()

# Soil parameters
terrain.Set_Soil_Density(1500)
terrain.Set_Soil_Cohesion(10000)
terrain.Set_Soil_Friction(0.8)
terrain.Set_Soil_Restitution(0.1)

# Initialize terrain height map (example: a simple bump)
height_map = terrain.Get_Height_map()
for i in range(terrain.Get_Size_X()):
    for j in range(terrain.Get_Size_Z()):
        height_map[i, j] = 0.0
        if (i - terrain.Get_Size_X() / 2)**2 + (j - terrain.Get_Size_Z() / 2)**2 < (terrain.Get_Size_X() / 4)**2:
            height_map[i, j] = 1.0  # Create a bump

terrain.Update()
system.Add(terrain)


# --- Create the HMMWV vehicle ---
hmmwv = veh.HMMWV()
hmmwv.Set_Vehicle_Mass(1800.0)
hmmwv.Set_Engine_Force(10000.0)
hmmwv.Set_Brake_Force(5000.0)
hmmwv.Set_Max_Steering_Angle(0.5)
hmmwv.Set_Wheel_Radius(0.4)
hmmwv.Set_Wheel_Width(0.3)

# Initial position and orientation
hmmwv.Set_Pos(chrono.ChVectorD(0, 1.0, 0))
hmmwv.Set_Transform(chrono.ChQuaternionD(0, 0, 0))

system.Add(hmmwv)

# --- Create a driver system ---
driver = veh.ChWheeledVehicleSimpleDriver()
driver.Set_Vehicle(hmmwv)
driver.Set_Input_Type(veh.ChDriverControlMode.SPEED)
driver.Set_Target_Speed(10.0)  # m/s
driver.Set_Max_Throttle(1.0)
driver.Set_Max_Brake(1.0)
driver.Set_Steering_Rate(0.5)

# --- Create the Irrlicht visualization ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV on Deformable Terrain')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))
vis.AddTypicalLights()

# --- Simulation loop ---
timestep = 0.005
time = 0.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation
    system.DoStepDynamics(timestep)

    # Update the driver
    driver.Synchronize(timestep)

    time += timestep

    # Print simulation time
    print(f"Time: {time:.2f}")