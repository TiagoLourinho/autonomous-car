import time
from typing import Optional

import numpy as np


class Sensors:
    """Sensor interface/model"""
    
    GPS_MEASUREMENT_STD = np.array((0.7, 0.7))
    ROTATION_MEASUREMENT_STD = np.array((0.01745, 0.01745, 0.01745))
    ACCELERATION_MEASUREMENT_STD = np.array((0.1, 0.1, 0.1))
        
    def __init__(self, port: Optional[str]=None, simulated: bool=True):
        """Initialize a new sensor interface class

        Args:
            port (Optional[str], optional): The serial port to be used when 
            communicating. Defaults to None.
            simulated (bool, optional): Whether to use simulated data (instead
            of real data from sensors). Defaults to True.

        Raises:
            ValueError: Port was not specified in non-simulation mode
        """
        self._simulated = simulated
        self._port = port
        self._world_view = None
        self._acquiring = False
        if self._port is None and not self._simulated:
            raise ValueError("Port must not be None when using real sensor data")
        
    def update_world_view(self, rot: float, pos: np.ndarray, vel: np.ndarray, acc: np.ndarray):
        """Updated the world view of the sensor model.

        Args:
            rot (float): The real rotation of the car
            pos (np.ndarray): The real position of the car
            vel (np.ndarray): The real velocity of the car
            acc (np.ndarray): The real acceleration of the car

        Raises:
            RuntimeError: Method call when using real sensor data
        """
        if not self._simulated:
            raise RuntimeError(f"{__name__}.update_world_view should only be called when using simulated sensor data")
        self._world_view = (rot, pos, vel, acc)
        
    def acquire(self):
        """Communicates with the Arduino.
        """
        if not self._simulated:
            raise NotImplementedError("Real sensor data acquisition is not yet implemented")
                
    def get_GPS_position(self) -> Optional[np.ndarray]:
        """Get a GPS position

        Raises:
            RuntimeError: Function was ran before acquisition was started

        Returns:
            Optional[np.ndarray]: A position array or `None` if no data is available.
        """        
        if self._simulated:
            mean = self._world_view[1][:2]
            return np.random.normal(mean, self.GPS_MEASUREMENT_STD)
        else:
            raise NotImplementedError("Real GPS data acquisition is not yet implemented")
        
    def get_IMU_data(self) -> Optional[np.ndarray]:
        """Get IMU data

        Raises:
            RuntimeError: Function was ran before acquisition was started

        Returns:
            Optional[np.ndarray]: An array containing the measured velocities
        """        
        if self._simulated:
            z_rotation = self._world_view[0]
            measured_z_rotation = np.random.normal(z_rotation, self.ROTATION_MEASUREMENT_STD[2])
            rot_mat = np.array((
                (np.cos(measured_z_rotation), -np.sin(measured_z_rotation)),
                (np.sin(measured_z_rotation), np.cos(measured_z_rotation)),
            ))
            # FIXME: should we still integrate the acceleration in simulation mode?
            velocity = rot_mat @ self._world_view[1][:2]
            measured_velocity = np.random.normal(velocity, self.ACCELERATION_MEASUREMENT_STD[:2])
            return np.array((*measured_velocity, measured_z_rotation))
        else:
            raise NotImplementedError("Real IMU data acquisition is not yet implemented")