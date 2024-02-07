# GSHP Modelling and Co-simulation Platform for Civil Engineering Building at Cambridge

## Overview
This repository is dedicated to the modelling and co-simulation of the Ground Source Heat Pump (GSHP) systems for the Civil Engineering Building at the University of Cambridge. The modelling is primarily based on the LBNL's Buildings library, available at [LBNL's Modelica Buildings library](https://simulationresearch.lbl.gov/modelica/).

## Repository Structure
The repository is organized into two main components:

1. **Above Ground Component**: This part is modelled using the Modelica language and is currently developed within the *Cambridge_CE.mo* package.

2. **Under Ground Heat Exchanger Component**: This section is intended to be modelled using Python. The connection between Modelica and Python is established through the Buildings' `Buildings.Utilities.IO.Python_3_8` module.

## Integration and Example
An example of the underground heat exchanger model, written in Python, is placed under `Buildings 9.1.0\Resources\Python-Sources\GroundHX.py`. In this *GroundHX.py* example, it's assumed that the ground source heat exchanger will increase the water temperature by a nominal 0.5 degrees Celsius.

The integration of Modelica and Python allows for a robust simulation environment, harnessing the strengths of both languages to model complex GSHP systems effectively.

For further details and updates, please refer to the respective documentation within each component's directory.
