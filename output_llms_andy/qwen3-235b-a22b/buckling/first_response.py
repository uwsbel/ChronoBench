import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr

# Initialize the PyChrono system
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, 0, 0))

# Beam parameters
length = 1.0
radius = 0.05
num_elements = 10
young_modulus = 1e7  # Pa
poisson_ratio = 0.3
density = 1000  # kg/m^3

# Calculate cross-sectional properties
area = chrono.CH_C_PI * radius**2
Iyy = chrono.CH_C_PI * radius**4 / 4
Izz = Iyy
J = chrono.CH_C_PI * radius**4 / 2

# Create material
material = fea.ChMaterialBeamANCF()
material.SetYoungModulus(young_modulus)
material.SetPoissonRatio(poisson_ratio)
material.SetDensity(density)
material.SetArea(area)
material.SetIyy(Iyy)
material.SetIzz(Izz)
material.SetJ(J)

# Create beam using builder
builder = fea.ChBuilderBeam()
builder.BuildBeam(my_system, material, num_elements, 
                  chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(length, 0, 0), 
                  radius)

# Fix the first node
builder.GetLastBeamNodes()[0].SetFixed(True)

# Custom motor function for compressive displacement
class CompressiveDisplacement(chrono.ChFunction):
    def __init__(self):
        super().__init__()
        self.speed = -0.01  # m/s (negative for compression)

    def Get_y(self, x):
        return self.speed * x

# Apply motor to the last node
end_node = builder.GetLastBeamNodes()[-1]
motor = chrono.ChLinkMotorLinearPosition()
motor.Initialize(end_node, chrono.ChFrameD(), chrono.ChFrameD())
func = CompressiveDisplacement()
motor.SetFunction(func)
my_system.AddLink(motor)

# Set solver and timestepper
solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(100)
solver.SetTolerance(1e-8)
my_system.SetSolver(solver)

timestepper = chrono.ChTimestepperHHT(my_system)
timestepper.SetStepControl(True)
timestepper.SetTolerance(1e-5)
timestepper.SetMaxIterations(20)
my_system.SetTimestepper(timestepper)

# Visualization setup
application = irr.ChIrrApp(my_system, 'Beam Buckling Simulation', irr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVectorD(0, 0, -2))

application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop
my_system.SetStep(0.01)
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    my_system.DoStepDynamics(my_system.GetStep())