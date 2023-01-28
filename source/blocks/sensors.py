import re
from typing import Optional

import numpy as np
import serial

from . import map as Map
MyTransformer = Map.MyTransformer


class Sensors:
    """Sensor interface/model"""

    GPS_MEASUREMENT_STD = np.array((0.7, 0.7))
    ROTATION_MEASUREMENT_STD = np.array((0.01745, 0.01745, 0.01745))
    ACCELERATION_MEASUREMENT_STD = np.array((0.1, 0.1, 0.1))
    IMU_MEASUREMENT_STD = np.array((0.1, 0.1, 0.01745))

    def __init__(self, port: Optional[str] = None, simulated: bool = True):
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
        self._buffer = b""

        self._imu_timestamp = 0
        self._last_imu_measurement = np.array([0.0, 0.0, 0.0])
        self._last_gps_measurement = np.array([0.0, 0.0])

        if self._port is None and not self._simulated:
            raise ValueError("Port must not be None when using real sensor data")
        self._serial = serial.Serial(
            self._port, 115200, bytesize=8, timeout=2, parity="N", xonxoff=0, stopbits=1
        )

    def _process_buffer(self):
        lines = self._buffer.split(b"\n")
        if len(lines) == 1:
            return
        self._buffer = lines[-1]
        for line in lines:
            rotation_line = re.findall(
                r"r (-?\d+\.\d+) (-?\d+\.\d+) (-?\d+\.\d+) (\d+)", line
            )
            if len(rotation_line):
                current_time = int(rotation_line[0][-1])
                if self._imu_timestamp == 0:
                    self._imu_timestamp = current_time
                self._world_view[0] += (
                    (current_time - self._imu_timestamp)
                    * 1e-3
                    * float(rotation_line[0][2])
                )
                self._imu_timestamp = current_time
                continue

            acceleration_line = re.findall(
                r"a (-?\d+\.\d+) (-?\d+\.\d+) (-?\d+\.\d+) (\d+)", line
            )
            if len(acceleration_line):
                current_time = int(acceleration_line[0][-1])
                if self._imu_timestamp == 0:
                    self._imu_timestamp = current_time
                self._world_view[1] += (
                    (current_time - self._imu_timestamp)
                    * 1e-3
                    * np.array([
                        np.cos(self._world_view[0]) * acceleration_line[0][:-1],
                        np.sin(self._world_view[0]) * acceleration_line[1][:-1],
                    ])
                )
                self._imu_timestamp = current_time
                continue

            gps_line = re.findall(r"g (-?\d+\.\d+) (-?\d+\.\d+)", line)
            if len(gps_line):
                parsed_gps = MyTransformer.transform(float(gps_line[0][0]), float(gps_line[0][1]))
                self._world_view[1] = np.array([*parsed_gps])

        self._last_gps_measurement = self._world_view[1]
        self._last_imu_measurement = self._world_view[2]

    def update_world_view(
        self, rot: float, pos: np.ndarray, vel: np.ndarray, acc: np.ndarray
    ):
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
            raise RuntimeError(
                f"{__name__}.update_world_view should only be called when using simulated sensor data"
            )
        self._world_view = (rot, pos, vel, acc)

    def acquire(self):
        """Communicates with the Arduino."""
        if not self._simulated:
            self._buffer += self._serial.read_all()
            self._process_buffer()

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
            return self._last_gps_measurement

    def get_IMU_data(self) -> Optional[np.ndarray]:
        """Get IMU data

        Raises:
            RuntimeError: Function was ran before acquisition was started

        Returns:
            Optional[np.ndarray]: An array containing the measured velocities
        """
        if self._simulated:
            z_rotation = self._world_view[0]
            measured_z_rotation = np.random.normal(
                z_rotation, self.ROTATION_MEASUREMENT_STD[2]
            )
            measured_z_velocity = np.random.normal(
                self._world_view[2][-1], self.ROTATION_MEASUREMENT_STD[2]
            )
            measured_velocity = np.random.normal(
                self._world_view[2][:2], self.ACCELERATION_MEASUREMENT_STD[:2]
            )
            return np.array((*measured_velocity, measured_z_velocity))
        else:
            return self._last_imu_measurement
