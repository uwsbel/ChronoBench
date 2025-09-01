import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# Create the Chrono::Engine physical system
chrono.SetChronoDataPath("path/to/chrono/data")
my_system = chrono.ChSystemNSC()

# Create the Irrlicht visualization
my_application = chronoirr.ChIrrApp(my_system, "HMMWV Simulation", chronoirr.Dimensions(800, 600))