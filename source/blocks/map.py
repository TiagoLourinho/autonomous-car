import googlemaps
import numpy as np

from pyproj import Transformer
from haversine import haversine
from constants import GMAPS_KEY


class Map:
    """Path planning (using google maps API for now)"""

    def __init__(self) -> None:
        self.__gmaps = googlemaps.Client(key=GMAPS_KEY)
        self.__transformer = Transformer.from_crs(
            "epsg:4326",
            "+proj=utm +zone=10 +ellps=WGS84",
            always_xy=True,
        )

    def rotate_points(self, points: np.array, angle: float):
        """Rotates the points by `angle` radians

        Parameters
        ----------

        points: np.array of shape (N, 2)
            Points to be rotated

        angle: float
            Angle in radians

        Returns
        -------

        np.array of shape (N, 2)
            Rotated points
        """

        rotation_matrix = np.array(
            [
                [np.cos(angle), -np.sin(angle)],
                [np.sin(angle), np.cos(angle)],
            ]
        )

        return np.array([rotation_matrix @ point for point in points])

    def get_coordinates(self, latitude: float, longitude: float, origin: np.array):
        """Transforms latitude and longitude to cartesian coordinates

        Parameters
        ----------

        latitude: float
            Latitude in degrees

        longitude: float
            Longitude in degrees

        Returns
        -------

        np.array of shape (2, )
            Cartesian coordinates
        """

        return np.array(
            [
                np.array([*self.__transformer.transform(latitude, longitude)])
                - np.array([*self.__transformer.transform(origin[0], origin[1])])
            ]
        )

    def get_points(self, start: np.array, end: np.array):
        """Gets the points between `start` and `end` point

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

        # Latitude and longitude coordinates of the two points
        point1 = (start[0], start[1])
        point2 = (end[0], end[1])

        # Calculate the distance between the two points
        distance = haversine(point1, point2)

        # Set the number of points you want to generate along the line
        num_points = int(distance * 200)
        print(f"Number of points: {num_points}")

        # Calculate the latitude and longitude increments for each point
        lat_inc = (end[0] - start[0]) / (num_points - 1)
        lon_inc = (end[1] - start[1]) / (num_points - 1)

        # Generate the points
        points = []
        for i in range(num_points):
            points.append(
                np.array(
                    [
                        start[0] + (i * lat_inc),
                        start[1] + (i * lon_inc),
                    ]
                )
            )

        return np.array(points)

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
                for step in self.__gmaps.directions(
                    start,
                    end,
                    mode="driving",
                    alternatives=False,
                    units="metric",
                )[0]["legs"][0]["steps"]
            ]
        )
        print("path")
        path = np.insert(path, 0, start, axis=0)
        print(path)
        new_path = []
        for i in range(len(path) - 1):
            new_path.extend(self.get_points(path[i], path[i + 1]))
            
        print("new_path")
        new_path = np.array(new_path)


        temp = np.array(
            [
                [step["location"]["latitude"], step["location"]["longitude"]]
                for step in self.__gmaps.nearest_roads(new_path)
            ]
        )
        print("temp")
        print(temp)
        

        origin_coord = self.__transformer.transform(start[0], start[1])

        # Transforms latitudes/longitudes (WGS 84) to cartesian coordinates
        xx, yy = np.array([*self.__transformer.transform(temp[:, 0], temp[:, 1])])
        for i in range(len(xx)):
            xx[i] -= origin_coord[0]
        for i in range(len(yy)):
            yy[i] -= origin_coord[1]

        return self.rotate_points(np.array([[xx[i], yy[i]] for i in range(len(xx))]), -np.pi / 80)
