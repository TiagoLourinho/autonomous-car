import googlemaps
import numpy as np
import math

from utils import R_x, R_z
from pyproj import CRS, Transformer, Proj
from haversine import haversine
from constants import (
    GMAPS_KEY,
    TOP_LEFT_CORNER,
    BOTTOM_RIGHT_CORNER,
    TOP_RIGHT_CORNER,
    BOTTOM_LEFT_CORNER,
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    ORIGIN,
)


class MyTransformer:
    """Transformer that projects (lat, lon) ECEF into (x, y) ENU"""

    def transform(self, lat, lon):
        wgs84_geod = CRS.from_epsg(4326)
        wgs84_geoc = CRS.from_epsg(4978)
        transformer = Transformer.from_crs(wgs84_geod, wgs84_geoc)

        xyzORIGIN = np.array([*transformer.transform(ORIGIN[0], ORIGIN[1], 89)])
        xyzPoint = np.array([*transformer.transform(lat, lon, 89)])

        return tuple(
            np.array(
                [
                    *(
                        (xyzPoint - xyzORIGIN)
                        @ R_z(math.radians(ORIGIN[1]) + np.pi / 2)
                        @ R_x(np.pi / 2 - math.radians(ORIGIN[0]))
                    )
                ]
            )[:2]
        )


class Map:
    """Path planning (using google maps API for now)"""

    def __init__(self) -> None:
        self.gmaps = googlemaps.Client(key=GMAPS_KEY)

        self.transformer = MyTransformer()

        self.top_left_coord = np.array(
            [*self.transformer.transform(TOP_LEFT_CORNER[0], TOP_LEFT_CORNER[1])]
        )
        self.bottom_right_coord = np.array(
            [
                *self.transformer.transform(
                    BOTTOM_RIGHT_CORNER[0], BOTTOM_RIGHT_CORNER[1]
                )
            ]
        )
        self.top_right_coord = np.array(
            [*self.transformer.transform(TOP_RIGHT_CORNER[0], TOP_RIGHT_CORNER[1])]
        )
        self.bottom_left_coord = np.array(
            [*self.transformer.transform(BOTTOM_LEFT_CORNER[0], BOTTOM_LEFT_CORNER[1])]
        )
        self.scale_x = IMAGE_WIDTH / (
            np.linalg.norm(self.bottom_left_coord - self.bottom_right_coord)
        )
        self.scale_y = IMAGE_HEIGHT / (
            np.linalg.norm(self.top_left_coord - self.bottom_left_coord)
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

    def get_coordinates(self, latitude: float, longitude: float):
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
        arr = np.array(
            [
                np.array([*self.transformer.transform(latitude, longitude)])
                # - self.bottom_left_coord
            ]
        )

        # rotate the points by pi/2 radians and reflect them
        # arr = self.rotate_points(arr, np.pi / 2)
        # arr = self.reflect_points(arr)

        # apperrently the pyproj library is rotating the points by around pi/60 radians
        # arr = self.rotate_points(arr, np.pi / 60)

        return arr * np.array([self.scale_x, self.scale_y])

    def reflect_points(self, points: np.array):
        """Reflects the points

        Parameters
        ----------

        points: np.array of shape (N, 2)
            Points to be reflected

        Returns
        -------

        np.array of shape (N, 2)
            Reflected points
        """

        reflection_matrix = np.array(
            [
                [1, 0],
                [0, -1],
            ]
        )

        return np.array([reflection_matrix @ point for point in points])

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
        num_points = int(distance * 400)

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
                for step in self.gmaps.directions(
                    start,
                    end,
                    mode="driving",
                    alternatives=False,
                    units="metric",
                )[0]["legs"][0]["steps"]
            ]
        )
        path = np.insert(path, 0, start, axis=0)
        new_path = []
        for i in range(len(path) - 1):
            new_path.extend(self.get_points(path[i], path[i + 1]))

        new_path = np.array(new_path)

        # # Gets points on the road
        # temp = np.array(
        #     [
        #         [step["location"]["latitude"], step["location"]["longitude"]]
        #         for step in self.gmaps.nearest_roads(new_path)
        #     ]
        # )
        
        # Get rid of 2-way roads
        temp = []
        originalIndex = -1
        for step in self.gmaps.nearest_roads(new_path):
            if originalIndex != step["originalIndex"]:
                temp.append([step["location"]["latitude"], step["location"]["longitude"]])
                originalIndex = step["originalIndex"]

        origin_coord = self.transformer.transform(start[0], start[1])

        coords = np.zeros((len(temp), 2))
        for i in range(len(temp)):
            coords[i] = self.get_coordinates(temp[i][0], temp[i][1])

        return coords

    def orient_path(self, path: np.array) -> np.array:

        oriented_path = np.zeros(shape=(len(path), 3))

        for i in range(0, len(path)):
            oriented_path[i][:2] = path[i]

            if i < len(path) - 1:
                # Add theta
                oriented_path[i][2] = np.arctan2(
                    (path[i + 1][1] - path[i][1]), (path[i + 1][0] - path[i][0])
                )

            else:
                oriented_path[i][2] = oriented_path[i - 1][2]

        return oriented_path
