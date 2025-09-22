import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math

# Create a Chrono simulation system
sys = chrono.ChSystemNSC()

# Define some constants for positioning the gears
planet_center_x = 0.33
planet_center_y = 0.25
planet_center_z = 0.05

# Set the truss center point in the 3D space
truss_center = chrono.ChVector3d(0, 0.5, 0.3)

# Create an epicyclic gear set with specified parameters
gear_epicyclic = chrono.ChGearEpicyclic()
gear_epicyclic.SetComputeNfromRatio(True)  # Automatically compute the teeth number ratio
gear_epicyclic.SetRatio(2)  # Set the gear ratio
gear_epicyclic.SetPhase(0)  # Set the phase between gears
gear_epicyclic.SetDiametralPitch(40)  # Set the diametral pitch of the gears

# Define the size of the gear wheels for visualization
gear_wheel_pit_size = chrono.ChVector3d(.1, .05, .06)

# Create the first gear (gear 1) and add it to the system
gear1 = chrono.ChSharedBody()
gear1_physical = chrono.ChBody()
gear_epicyclic.MakeGear1(gear1_physical, truss_center + chrono.ChVector3d(-0.20, 0, 0), chrono.ChVector3d(0, 1, 0), gear_wheel_pit_size)
gear1_physical.SetPos_dt(chrono.ChVector3d(0, 0, 0))  # Set initial angular velocity to zero
sys.Add(gear1_physical)  # Add gear 1 to the system
gear1.SetBody(gear1_physical)  # Associate the physical body with the gear

# Create the second gear (gear 2) and add it to the system
gear2 = chrono.ChSharedBody()
gear2_physical = chrono.ChBody()
gear_epicyclic.MakeGear2(gear2_physical, truss_center + chrono.ChVector3d(+0.12, 0, 0), chrono.ChVector3d(0, 1, 0), gear_wheel_pit_size)
gear2_physical.SetPos_dt(chrono.ChVector3d(0, 0, 0))  # Set initial angular velocity to zero
sys.Add(gear2_physical)  # Add gear 2 to the system
gear2.SetBody(gear2_physical)  # Associate the physical body with the gear

# Create the sun gear and add it to the system
gearS = chrono.ChSharedBody()
gearS_physical = chrono.ChBody()
gear_epicyclic.MakeGearS(gearS_physical, truss_center + chrono.ChVector3d(planet_center_x, planet_center_y, planet_center_z), chrono.ChVector3d(0, 0, 1))
sys.Add(gearS_physical)  # Add sun gear to the system
gearS.SetBody(gearS_physical)  # Associate the physical body with the sun gear

# Create the planet gear and add it to the system
gearP = chrono.ChSharedBody()
gearP_physical = chrono.ChBody()
gear_epicyclic.MakeGearP(gearP_physical, truss_center + chrono.ChVector3d(planet_center_x, planet_center_y, planet_center_z), chrono.ChVector3d(0, 0, 1), 0.06)
sys.Add(gearP_physical)  # Add planet gear to the system
gearP.SetBody(gearP_physical)  # Associate the physical body with the planet gear

# Create the ring gear and add it to the system
gearR = chrono.ChSharedBody()
gearR_physical = chrono.ChBody()
gear_epicyclic.MakeGearR(gearR_physical, truss_center + chrono.ChVector3d(planet_center_x, planet_center_y, planet_center_z), chrono.ChVector3d(0, 0, 1))
sys.Add(gearR_physical)  # Add ring gear to the system
gearR.SetBody(gearR_physical)  # Associate the physical body with the ring gear

# Connect the gears using epicyclic connections
conS = chrono.ChSharedBody()
conSP = chrono.ChSharedBody()
conPR = chrono.ChSharedBody()

conS_physical = chrono.ChBody()
conSP_physical = chrono.ChBody()
conPR_physical = chrono.ChBody()

gear_epicyclic.MakeConnections(conS_physical, conSP_physical, conPR_physical, gearP_physical, chrono.ChVector3d(planet_center_x, planet_center_y, planet_center_z), chrono.ChVector3d(0, 0, 1))

sys.Add(conS_physical)  # Add connection S to the system
sys.Add(conSP_physical)  # Add connection SP to the system
sys.Add(conPR_physical)  # Add connection PR to the system

conS.SetBody(conS_physical)
conSP.SetBody(conSP_physical)
conPR.SetBody(conPR_physical)

# Create and add a truss to the system for structural support
mtruss = chrono.ChBodyEasyBox(0.8, 0.1, 0.05, 1000, True, False)
mtruss.SetPos(truss_center)
mtruss.SetFixed(True)  # Fix the truss in place
sys.Add(mtruss)

# Create a bar connected to the truss and gears
bar = chrono.ChBodyEasyBox(0.5, 0.05, 0.05, 1000, True, False)
bar.SetPos(truss_center + chrono.ChVector3d(0.15, -0.20, 0))
sys.Add(bar)  # Add the bar to the system

# Create a revolute joint to connect the bar and the truss
joint_btr = chrono.ChLinkLockRevolute()
joint_btr.Initialize(bar, mtruss, chrono.ChFramed(truss_center + chrono.ChVector3d(0.15, 0, 0), chrono.QUNIT))
sys.Add(joint_btr)

# Create a gear and a shaft for rotational motion
shaft = chrono.ChBodyEasyCylinder(0.02, 0.05, 1000, True, False)
shaft.SetPos(truss_center + chrono.ChVector3d(0.15, -0.1, 0))
shaft.SetRot(chrono.QuatFromAngleX(chrono.CH_PI_2))
sys.Add(shaft)

# Connect the bar and the shaft with a revolute joint
joint_bsh = chrono.ChLinkLockRevolute()
joint_bsh.Initialize(bar, shaft, chrono.ChFramed(truss_center + chrono.ChVector3d(0.15, -0.1, 0), chrono.QuatFromAngleX(chrono.CH_PI_2)))
sys.Add(joint_bsh)

# Create a gear pair to connect the shaft and gear 1
gear2s = chrono.ChLinkLockGear()
gear2s.Initialize(gear1_physical, shaft)
gear2s.SetRevolute2(chrono.ChFramed(chrono.ChVector3d(truss_center.x + 0.055, truss_center.y - 0.1, truss_center.z), chrono.QUNIT))
gear2s.SetPhase(chrono.CH_PI / 2)
gear2s.SetRotAxis1(chrono.ChVector3d(0, 0, 1))
gear2s.SetRotAxis2(chrono.ChVector3d(1, 0, 0))
gear2s.SetFusion(chrono.ChLinkLockGear.FUSION_ATERROR)
sys.Add(gear2s)

# Create and add a gear motor to the system
gear_mot = chrono.ChLinkMotorRotationAngle()
gear_mot.Initialize(gearS_physical, mtruss, chrono.ChFramed(truss_center, chrono.QUNIT))
gear_mot.SetAngleFunction(chrono.ChFunctionAngleConst(0))  # Set initial angle function
gear_mot.SetMotorFunction(chrono.ChFunctionAngleConst(chrono.CH_PI))  # Set constant angular velocity
sys.Add(gear_mot)

# Create and set up the visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 0.5, 0.3), chrono.ChVector3d(0.3, 0.1, 0.05))
vis.AddTypicalLights()

# Run the interactive simulation loop
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-4)  # Advance the simulation by a time step of 1e-4 seconds