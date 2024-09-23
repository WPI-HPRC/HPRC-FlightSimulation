from rocketpy import Environment, SolidMotor, Rocket, Flight
import datetime

# Create the Environment
## Launch on the Quad
launchLat = 42.273836 # [deg]
launchLon = -71.809810 # [deg]
launchElev = 152.4 # [m]

env = Environment(
    latitude=launchLat, 
    longitude=launchLon, 
    elevation=launchElev)

tomorrow = datetime.date.today() + datetime.timedelta(days=1) # Tomorrow

env.set_date((tomorrow.year, tomorrow.month, tomorrow.day, 12)) # Midnight today

env.set_atmospheric_model(type='Forecast', file='GFS')

# env.info()

# Motor Configuration
M3400 = SolidMotor(
    thrustSource='Cesaroni_9994M3400-P.eng',  # Assuming the thrust curve data is in this file
    dry_mass = 7.3,  # kg, approximate dry mass of the motor case without fuel
    dry_inertia= [0.05, 0.05, 0.01],  # kg*m^2, estimated moments of inertia (Ixx, Iyy, Izz), adjust if needed
    nozzle_radius = 0.026,  # meters, estimated nozzle exit radius (26mm)
    grain_number = 4,  # Number of propellant grains
    grain_density = 1815,  # kg/m^3, typical propellant density (Cesaroni motors use a HTPB-based propellant)
    grain_outer_radius = 0.046,  # meters, approximate outer radius of each grain (46mm)
    grain_initial_inner_radius = 0.020,  # meters, approximate initial inner radius of each grain (20mm)
    grain_initial_height = 0.18,  # meters, approximate grain height (180mm)
    grain_separation = 0.005,  # meters, gap between each grain (5mm)
    grains_center_of_mass_position = -0.25,  # meters, estimated position of grains' center of mass relative to motor reference (adjust as needed)
    nozzle_position = -0.4,  # meters, approximate position of the nozzle exit relative to motor's reference
    burn_time = 2.3,  # seconds, estimated burn time based on the motor's thrust curve
    throat_radius = 0.010,  # meters, estimated throat radius (10mm)
    coordinate_system_orientation = "nozzle_to_combustion_chamber",  # Default orientation
)
