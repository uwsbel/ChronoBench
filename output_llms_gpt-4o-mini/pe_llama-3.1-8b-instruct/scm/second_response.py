import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Define a new class to manage SCM terrain parameters
class SCMParameters:
    def __init__(self):
        self.Bekker_Kphi = 2e6  # Bekker Kphi
        self.Bekker_Kc = 0  # Bekker Kc
        self.Bekker_n_exponent = 1.1  # Bekker n exponent
        self.Mohr_cohesive_limit = 0  # Mohr cohesive limit (Pa)
        self.Mohr_friction_limit = 30  # Mohr friction limit (degrees)
        self.Janosi_shear_coefficient = 0.01  # Janosi shear coefficient (m)
        self.Elastic_stiffness = 2e8  # Elastic stiffness (Pa/m), before plastic yield
        self.Damping = 3e4  # Damping (Pa s/m), proportional to negative vertical speed (optional)

    def set_parameters(self, Bekker_Kphi, Bekker_Kc, Bekker_n_exponent, Mohr_cohesive_limit, Mohr_friction_limit, Janosi_shear_coefficient, Elastic_stiffness, Damping):
        self.Bekker_Kphi = Bekker_Kphi
        self.Bekker_Kc = Bekker_Kc
        self.Bekker_n_exponent = Bekker_n_exponent
        self.Mohr_cohesive_limit = Mohr_cohesive_limit
        self.Mohr_friction_limit = Mohr_friction_limit
        self.Janosi_shear_coefficient = Janosi_shear_coefficient
        self.Elastic_stiffness = Elastic_stiffness
        self.Damping = Damping

# Create an instance of the SCMParameters class
scm_parameters = SCMParameters()

# Define terrain configurations
terrain_configurations = {
    "soft": {
        "Bekker_Kphi": 2e6,
        "Bekker_Kc": 0,
        "Bekker_n_exponent": 1.1,
        "Mohr_cohesive_limit": 0,
        "Mohr_friction_limit": 30,
        "Janosi_shear_coefficient": 0.01,
        "Elastic_stiffness": 2e8,
        "Damping": 3e4
    },
    "mid": {
        "Bekker_Kphi": 1e6,
        "Bekker_Kc": 0,
        "Bekker_n_exponent": 1.0,
        "Mohr_cohesive_limit": 0,
        "Mohr_friction_limit": 30,
        "Janosi_shear_coefficient": 0.01,
        "Elastic_stiffness": 1e8,
        "Damping": 3e4
    },
    "hard": {
        "Bekker_Kphi": 1e5,
        "Bekker_Kc": 0,
        "Bekker_n_exponent": 1.0,
        "Mohr_cohesive_limit": 0,
        "Mohr_friction_limit": 30,
        "Janosi_shear_coefficient": 0.01,
        "Elastic_stiffness": 1e7,
        "Damping": 3e4
    }
}

# Initialize SCM terrain parameters using a predefined configuration
scm_parameters.set_parameters(terrain_configurations["soft"]["Bekker_Kphi"], terrain_configurations["soft"]["Bekker_Kc"], terrain_configurations["soft"]["Bekker_n_exponent"], terrain_configurations["soft"]["Mohr_cohesive_limit"], terrain_configurations["soft"]["Mohr_friction_limit"], terrain_configurations["soft"]["Janosi_shear_coefficient"], terrain_configurations["soft"]["Elastic_stiffness"], terrain_configurations["soft"]["Damping"])

# Rest of the code remains the same