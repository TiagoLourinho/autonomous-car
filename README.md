# Autonomous Car 

## Goal
This project aims at developing components for an autonomous car operating inside IST Alameda campus. 

## Authors
- Francisco Rodrigues
- João Marafuz Gaspar
- João Fonseca
- Manuel Graça
- Tiago Ferreira
- Tiago Lourinho

## Usage
1. Install the required libraries: `pip install -r requirements.txt`
2. Adjust the parameters in `source/constants.py` (for example frequency operation and whether it is just a simulation or is it working with real sensors and motors, the default is 100 Hz and simulation mode) 
3. Run the program: `python3 source/main.py`

## Folders

- `images/` - IST map
- `imu_gps/` - Handles communication with real sensors
- `source/` - Main system code
- `tests/` - Tests done to modules of the system


# Example
A simulation example in the IST campus:

![A simulation example in the IST campus](/images/example.png "IST")
