import googlemaps
import numpy as np
from pyproj import Transformer


from constants import GMAPS_KEY


class Map:
    """Path planning (using google maps API for now)"""

    def __init__(self) -> None:
        self.gmaps = googlemaps.Client(key=GMAPS_KEY)
        self.transformer = Transformer.from_crs(
            "epsg:4326",
            "+proj=utm +zone=10 +ellps=WGS84",
            always_xy=True,
        )

    def get_path(self, start: np.array, end: np.array):
        """Gets the path from `start` to `end` point

        Parameters
        ----------

        start: np.array of shape (2, )
            Latitude and longitude of starting point

        end: np.array of shape (2, )
            Latitude and longitude of destination point

        Returns
        -------

        np.array of shape (N, 2) where N is the number of points
            Obtained path points in cartesian coordinates
        """

        path = np.array(
            [
                [step["end_location"]["lat"], step["end_location"]["lng"]]
                for step in self.gmaps.directions(
                    start, end, mode="driving", alternatives=False, units="metric"
                )[0]["legs"][0]["steps"]
            ]
        )

        # Transforms latitudes/longitudes (WGS 84) to cartesian coordinates
        xx, yy = self.transformer.transform(path[:, 0], path[:, 1])

        return np.array([[xx[i], yy[i]] for i in len(xx)])
