# Import necessary libraries
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# Initialize the PyChrono environment and core components
sys = chrono.ChSystemNSC()

# Set the default contact material properties
contact_material = chrono.ChMaterialSurfaceNSC()
contact_material.SetFriction(0.9)
contact_material.SetDampingF(0.2)
contact_material.SetCompliance(0.0001)
contact_material.SetComplianceT(0.0001)

# Create the ground body
ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True, contact_material)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)

# Create the pendulum body
pendulum = chrono.ChBodyEasySphere(sys, 1, 1000, True, True, contact_material)
pendulum.SetPos(chrono.ChVectorD(0, 5, 0))
pendulum.SetMass(10)
pendulum.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))

# Create a revolute joint to connect the pendulum to the ground
revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(ground, pendulum, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
sys.Add(revolute_joint)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 10, -10))
vis.AddTypicalLights()

# Add the ground and pendulum to the visualization system
vis.AssetBind(ground)
vis.AssetBind(pendulum)

# Set up the simulation loop
time_step = 0.01
end_time = 10

while sys.GetChTime() < end_time:
    # Update the simulation
    sys.DoStepDynamics(time_step)

    # Log the pendulum's position and velocity
    print(f"Time: {sys.GetChTime():.2f}, Position: {pendulum.GetPos().x:.2f}, {pendulum.GetPos().y:.2f}, {pendulum.GetPos().z:.2f}, Velocity: {pendulum.GetPos_dt().x:.2f}, {pendulum.GetPos_dt().y:.2f}, {pendulum.GetPos_dt().z:.2f}")

    # Render the simulation
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

    # Pause for a short time to control the simulation speed
    chrono.ChSleep(0.01)