# Optimize.py

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
import Plot_Mission
import Vehicles
import Procedure

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():

    # build the vehicle, configs, and analyses
    configs, analyses = Procedure.full_setup()
    
    configs.finalize()
    analyses.finalize()    
    
    # weight analysis
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate()          
    
    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate()
    
    # plot results    
    Plot_Mission.plot_mission(results)
    

    return



if __name__ == '__main__':
    main()