import googlemaps
import numpy as np


from constants import GMAPS_KEY


class Map:
    def __init__(self) -> None:
        self.__gmaps = googlemaps.Client(key=GMAPS_KEY)

    def get_directions(self, start: np.array, end: np.array):
        """Gets the directions coordinates"""

        return np.array(
            [
                [step["end_location"]["lat"], step["end_location"]["lng"]]
                for step in self.__gmaps.directions(
                    start, end, mode="driving", alternatives=False, units="metric"
                )[0]["legs"][0]["steps"]
            ]
        )

    def transform_coordinates(self, points: np.array):
        """Transforms latitudes/longitudes (WGS 84) to cartesian coordinates"""

        pass
