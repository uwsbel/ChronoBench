import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

# If true, use provided callback to change the mass of the pendulum bodies during simulation.
# This is an example of how to extend the functionality of the standard system.
change_mass = False

# Initial length of the pendulum
L = 1.0

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 0.75)

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Simulation end time
tend = 15

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Output directories
out_dir = "./PENULUM"

# =============================================================================

# Change default data path to current directory
chrono.SetChronoDataPath('.')

# Print message
print( "Copyright (c) 2017 projectchrono.org\n")

# --------------
# Create systems
# --------------

# Create the mechanical system
system = chrono.ChSystemNSC()

# Set gravitational acceleration
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# Create the ground body
ground = chrono.ChBody()
ground.SetFixed(True)
system.Add(ground)

# Create the first pendulum body
pendulum1 = chrono.ChBody()
pendulum1.SetMass(20)
pendulum1.SetInertiaXX(chrono.ChVector3d(10, 10, 10))
system.Add(pendulum1)

# Create the second pendulum body
pendulum2 = chrono.ChBody()
pendulum2.SetMass(15)
pendulum2.SetInertiaXX(chrono.ChVector3d(5, 5, 5))
system.Add(pendulum2)

# Create the third pendulum body
pendulum3 = chrono.ChBody()
pendulum3.SetMass(10)
pendulum3.SetInertiaXX(chrono.ChVector3d(2, 2, 2))
system.Add(pendulum3)

# Create the fourth pendulum body
pendulum4 = chrono.ChBody()
pendulum4.SetMass(5)
pendulum4.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
system.Add(pendulum4)

# Create the fifth pendulum body
pendulum5 = chrono.ChBody()
pendulum5.SetMass(1)
pendulum5.SetInertiaXX(chrono.ChVector3d(0.5, 0.5, 0.5))
system.Add(pendulum5)

# Create the sixth pendulum body
pendulum6 = chrono.ChBody()
pendulum6.SetMass(0.5)
pendulum6.SetInertiaXX(chrono.ChVector3d(0.2, 0.2, 0.2))
system.Add(pendulum6)

# Create the seventh pendulum body
pendulum7 = chrono.ChBody()
pendulum7.SetMass(0.2)
pendulum7.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
system.Add(pendulum7)

# Create the eighth pendulum body
pendulum8 = chrono.ChBody()
pendulum8.SetMass(0.1)
pendulum8.SetInertiaXX(chrono.ChVector3d(0.05, 0.05, 0.05))
system.Add(pendulum8)

# Create the ninth pendulum body
pendulum9 = chrono.ChBody()
pendulum9.SetMass(0.05)
pendulum9.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
system.Add(pendulum9)

# Create the tenth pendulum body
pendulum10 = chrono.ChBody()
pendulum10.SetMass(0.02)
pendulum10.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
system.Add(pendulum10)

# Create the eleventh pendulum body
pendulum11 = chrono.ChBody()
pendulum11.SetMass(0.01)
pendulum11.SetInertiaXX(chrono.ChVector3d(0.005, 0.005, 0.005))
system.Add(pendulum11)

# Create the twelfth pendulum body
pendulum12 = chrono.ChBody()
pendulum12.SetMass(0.005)
pendulum12.SetInertiaXX(chrono.ChVector3d(0.002, 0.002, 0.002))
system.Add(pendulum12)

# Create the thirteenth pendulum body
pendulum13 = chrono.ChBody()
pendulum13.SetMass(0.002)
pendulum13.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
system.Add(pendulum13)

# Create the fourteenth pendulum body
pendulum14 = chrono.ChBody()
pendulum14.SetMass(0.001)
pendulum14.SetInertiaXX(chrono.ChVector3d(0.0005, 0.0005, 0.0005))
system.Add(pendulum14)

# Create the fifteenth pendulum body
pendulum15 = chrono.ChBody()
pendulum15.SetMass(0.0005)
pendulum15.SetInertiaXX(chrono.ChVector3d(0.0002, 0.0002, 0.0002))
system.Add(pendulum15)

# Create the sixteenth pendulum body
pendulum16 = chrono.ChBody()
pendulum16.SetMass(0.0002)
pendulum16.SetInertiaXX(chrono.ChVector3d(0.0001, 0.0001, 0.0001))
system.Add(pendulum16)

# Create the seventeenth pendulum body
pendulum17 = chrono.ChBody()
pendulum17.SetMass(0.0001)
pendulum17.SetInertiaXX(chrono.ChVector3d(0.00005, 0.00005, 0.00005))
system.Add(pendulum17)

# Create the eighteenth pendulum body
pendulum18 = chrono.ChBody()
pendulum18.SetMass(0.00005)
pendulum18.SetInertiaXX(chrono.ChVector3d(0.00002, 0.00002, 0.00002))
system.Add(pendulum18)

# Create the nineteenth pendulum body
pendulum19 = chrono.ChBody()
pendulum19.SetMass(0.00002)
pendulum19.SetInertiaXX(chrono.ChVector3d(0.00001, 0.00001, 0.00001))
system.Add(pendulum19)

# Create the twentieth pendulum body
pendulum20 = chrono.ChBody()
pendulum20.SetMass(0.00001)
pendulum20.SetInertiaXX(chrono.ChVector3d(0.000005, 0.000005, 0.000005))
system.Add(pendulum20)

# Create the twenty-first pendulum body
pendulum21 = chrono.ChBody()
pendulum21.SetMass(0.000005)
pendulum21.SetInertiaXX(chrono.ChVector3d(0.000002, 0.000002, 0.000002))
system.Add(pendulum21)

# Create the twenty-second pendulum body
pendulum22 = chrono.ChBody()
pendulum22.SetMass(0.000002)
pendulum22.SetInertiaXX(chrono.ChVector3d(0.000001, 0.000001, 0.000001))
system.Add(pendulum22)

# Create the twenty-third pendulum body
pendulum23 = chrono.ChBody()
pendulum23.SetMass(0.000001)
pendulum23.SetInertiaXX(chrono.ChVector3d(0.0000005, 0.0000005, 0.0000005))
system.Add(pendulum23)

# Create the twenty-fourth pendulum body
pendulum24 = chrono.ChBody()
pendulum24.SetMass(0.0000005)
pendulum24.SetInertiaXX(chrono.ChVector3d(0.0000002, 0.0000002, 0.0000002))
system.Add(pendulum24)

# Create the twenty-fifth pendulum body
pendulum25 = chrono.ChBody()
pendulum25.SetMass(0.0000002)
pendulum25.SetInertiaXX(chrono.ChVector3d(0.0000001, 0.0000001, 0.0000001))
system.Add(pendulum25)

# Create the twenty-sixth pendulum body
pendulum26 = chrono.ChBody()
pendulum26.SetMass(0.0000001)
pendulum26.SetInertiaXX(chrono.ChVector3d(0.00000005, 0.00000005, 0.00000005))
system.Add(pendulum26)

# Create the twenty-seventh pendulum body
pendulum27 = chrono.ChBody()
pendulum27.SetMass(0.00000005)
pendulum27.SetInertiaXX(chrono.ChVector3d(0.00000002, 0.00000002, 0.00000002))
system.Add(pendulum27)

# Create the twenty-eighth pendulum body
pendulum28 = chrono.ChBody()
pendulum28.SetMass(0.00000002)
pendulum28.SetInertiaXX(chrono.ChVector3d(0.00000001, 0.00000001, 0.00000001))
system.Add(pendulum28)

# Create the twenty-ninth pendulum body
pendulum29 = chrono.ChBody()
pendulum29.SetMass(0.00000001)
pendulum29.SetInertiaXX(chrono.ChVector3d(0.000000005, 0.000000005, 0.000000005))
system.Add(pendulum29)

# Create the thirtieth pendulum body
pendulum30 = chrono.ChBody()
pendulum30.SetMass(0.000000005)
pendulum30.SetInertiaXX(chrono.ChVector3d(0.000000002, 0.000000002, 0.000000002))
system.Add(pendulum30)

# Create the thirty-first pendulum body
pendulum31 = chrono.ChBody()
pendulum31.SetMass(0.000000002)
pendulum31.SetInertiaXX(chrono.ChVector3d(0.000000001, 0.000000001, 0.000000001))
system.Add(pendulum31)

# Create the thirty-second pendulum body
pendulum32 = chrono.ChBody()
pendulum32.SetMass(0.000000001)
pendulum32.SetInertiaXX(chrono.ChVector3d(0.0000000005, 0.0000000005, 0.0000000005))
system.Add(pendulum32)

# Create the thirty-third pendulum body
pendulum33 = chrono.ChBody()
pendulum33.SetMass(0.0000000005)
pendulum33.SetInertiaXX(chrono.ChVector3d(0.0000000002, 0.0000000002, 0.0000000002))
system.Add(pendulum33)

# Create the thirty-fourth pendulum body
pendulum34 = chrono.ChBody()
pendulum34.SetMass(0.0000000002)
pendulum34.SetInertiaXX(chrono.ChVector3d(0.0000000001, 0.0000000001, 0.0000000001))
system.Add(pendulum34)

# Create the thirty-fifth pendulum body
pendulum35 = chrono.ChBody()
pendulum35.SetMass(0.0000000001)
pendulum35.SetInertiaXX(chrono.ChVector3d(0.00000000005, 0.00000000005, 0.00000000005))
system.Add(pendulum35)

# Create the thirty-sixth pendulum body
pendulum36 = chrono.ChBody()
pendulum36.SetMass(0.00000000005)
pendulum36.SetInertiaXX(chrono.ChVector3d(0.00000000002, 0.00000000002, 0.00000000002))
system.Add(pendulum36)

# Create the thirty-seventh pendulum body
pendulum37 = chrono.ChBody()
pendulum37.SetMass(0.00000000002)
pendulum37.SetInertiaXX(chrono.ChVector3d(0.00000000001, 0.00000000001, 0.00000000001))
system.Add(pendulum37)

# Create the thirty-eighth pendulum body
pendulum38 = chrono.ChBody()
pendulum38.SetMass(0.00000000001)
pendulum38.SetInertiaXX(chrono.ChVector3d(0.000000000005, 0.000000000005, 0.000000000005))
system.Add(pendulum38)

# Create the thirty-ninth pendulum body
pendulum39 = chrono.ChBody()
pendulum39.SetMass(0.000000000005)
pendulum39.SetInertiaXX(chrono.ChVector3d(0.000000000002, 0.000000000002, 0.000000000002))
system.Add(pendulum39)

# Create the fortieth pendulum body
pendulum40 = chrono.ChBody()
pendulum40.SetMass(0.000000000002)
pendulum40.SetInertiaXX(chrono.ChVector3d(0.000000000001, 0.000000000001, 0.000000000001))
system.Add(pendulum40)

# Create the forty-first pendulum body
pendulum41 = chrono.ChBody()
pendulum41.SetMass(0.000000000001)
pendulum41.SetInertiaXX(chrono.ChVector3d(0.0000000000005, 0.0000000000005, 0.0000000000005))
system.Add(pendulum41)

# Create the forty-second pendulum body
pendulum42 = chrono.ChBody()
pendulum42.SetMass(0.0000000000005)
pendulum42.SetInertiaXX(chrono.ChVector3d(0.0000000000002, 0.0000000000002, 0.0000000000002))
system.Add(pendulum42)

# Create the forty-third pendulum body
pendulum43 = chrono.ChBody()
pendulum43.SetMass(0.0000000000002)
pendulum43.SetInertiaXX(chrono.ChVector3d(0.0000000000001, 0.0000000000001, 0.0000000000001))
system.Add(pendulum43)

# Create the forty-fourth pendulum body
pendulum44 = chrono.ChBody()
pendulum44.SetMass(0.0000000000001)
pendulum44.SetInertiaXX(chrono.ChVector3d(0.00000000000005, 0.00000000000005, 0.00000000000005))
system.Add(pendulum44)

# Create the forty-fifth pendulum body
pendulum45 = chrono.ChBody()
pendulum45.SetMass(0.00000000000005)
pendulum45.SetInertiaXX(chrono.ChVector3d(0.00000000000002, 0.00000000000002, 0.00000000000002))
system.Add(pendulum45)

# Create the forty-sixth pendulum body
pendulum46 = chrono.ChBody()
pendulum46.SetMass(0.00000000000002)
pendulum46.SetInertiaXX(chrono.ChVector3d(0.00000000000001, 0.00000000000001, 0.00000000000001))
system.Add(pendulum46)

# Create the forty-seventh pendulum body
pendulum47 = chrono.ChBody()
pendulum47.SetMass(0.00000000000001)
pendulum47.SetInertiaXX(chrono.ChVector3d(0.000000000000005, 0.000000000000005, 0.000000000000005))
system.Add(pendulum47)

# Create the forty-eighth pendulum body
pendulum48 = chrono.ChBody()
pendulum48.SetMass(0.000000000000005)
pendulum48.SetInertiaXX(chrono.ChVector3d(0.000000000000002, 0.000000000000002, 0.000000000000002))
system.Add(pendulum48)

# Create the forty-ninth pendulum body
pendulum49 = chrono.ChBody()
pendulum49.SetMass(0.000000000000002)
pendulum49.SetInertiaXX(chrono.ChVector3d(0.000000000000001, 0.000000000000001, 0.000000000000001))
system.Add(pendulum49)

# Create the fiftieth pendulum body
pendulum50 = chrono.ChBody()
pendulum50.SetMass(0.000000000000001)
pendulum50.SetInertiaXX(chrono.ChVector3d(0.0000000000000005, 0.0000000000000005, 0.0000000000000005))
system.Add(pendulum50)

# Create the fifty-first pendulum body
pendulum51 = chrono.ChBody()
pendulum51.SetMass(0.0000000000000005)
pendulum51.SetInertiaXX(chrono.ChVector3d(0.0000000000000002, 0.0000000000000002, 0.0000000000000002))
system.Add(pendulum51)

# Create the fifty-second pendulum body
pendulum52 = chrono.ChBody()
pendulum52.SetMass(0.0000000000000002)
pendulum52.SetInertiaXX(chrono.ChVector3d(0.0000000000000001, 0.0000000000000001, 0.0000000000000001))
system.Add(pendulum52)

# Create the fifty-third pendulum body
pendulum53 = chrono.ChBody()
pendulum53.SetMass(0.0000000000000001)
pendulum53.SetInertiaXX(chrono.ChVector3d(0.00000000000000005, 0.00000000000000005, 0.00000000000000005))
system.Add(pendulum53)

# Create the fifty-fourth pendulum body
pendulum54 = chrono.ChBody()
pendulum54.SetMass(0.00000000000000005)
pendulum54