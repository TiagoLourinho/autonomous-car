import googlemaps
import numpy as np



from constants import GMAPS_KEY
from pyproj import Transformer


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
        # Please try 3857 epsg code used by Google Maps
        transform = Transformer.from_crs("epsg:4326", "+proj=utm +zone=10 +ellps=WGS84", always_xy=True,)
        xx, yy = transform.transform(points[:, 0], points[:, 1])
        return xx, yy
