# Procedure.py

# Python Imports
import numpy as np
import pylab as plt
from subprocess import call

# SUAVE Imports
import SUAVE
from SUAVE.Core import Data, Units
from SUAVE.Components.Energy.Networks.Battery_Propeller import Battery_Propeller
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Input_Output.Results import  print_parasite_drag,  \
     print_compress_drag, \
     print_engine_data,   \
     print_mission_breakdown, \
     print_weight_breakdown
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_energy_and_power, initialize_from_mass
from SUAVE.Input_Output.OpenVSP import vsp_write
#from SUAVE.Input_Output.OpenVSP import vspaero

# Files Imports

import Analyses
import Missions
import Vehicles

# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():

    # vehicle data
    vehicle  = Vehicles.vehicle_setup()
    configs  = Vehicles.configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = Analyses.analyses_setup(configs)

    # mission analyses
    mission  = Missions.mission_setup(configs_analyses,configs)
    missions_analyses = Missions.missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses

    return configs, analyses