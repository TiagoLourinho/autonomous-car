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
    """Coordinate and Reference Frame Transformer"""

    def transform(self, lat, lon):
        """Transforms (lat, lon) ECEF into (e, n) ENU"""
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

    def invtransform(self, e, n):
        """Transforms (e, n) ENU into (lat, lon) ECEF"""
        wgs84_geod = CRS.from_epsg(4326)
        wgs84_geoc = CRS.from_epsg(4978)
        transformer = Transformer.from_crs(wgs84_geod, wgs84_geoc)
        invtransformer = Transformer.from_crs(wgs84_geoc, wgs84_geod)

        xyzORIGIN = np.array([*transformer.transform(ORIGIN[0], ORIGIN[1], 89)])
        enuPoint = np.array([e, n, 0])

        xyzPoint = tuple(
            np.array(
                [
                    *(
                        enuPoint
                        @ R_x(math.radians(ORIGIN[0]) - np.pi / 2)
                        @ R_z(-math.radians(ORIGIN[1]) - np.pi / 2)
                        + xyzORIGIN
                    )
                ]
            )
        )

        return tuple(
            np.array(
                [*invtransformer.transform(xyzPoint[0], xyzPoint[1], xyzPoint[2])]
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
    
    def verify_point(self, point):
        """Checks if the point is valid

        Parameters
        ----------

        point: np.array of shape (2, )
            Point to be checked

        Returns
        -------

        bool
            True if the point is within the map, False otherwise
        """
        new_point = self.gmaps.nearest_roads(point)
        new_point = np.array([new_point[0]["location"]["latitude"], new_point[0]["location"]["longitude"]])

        # disance between the point and the nearest road
        distance = haversine(point, new_point)

        # maybe pass to x, y so that we can better understand the distance limit

        if distance < 0.01:
            return True
        return False

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

    def get_latlon(self, east: float, north: float):
        """Transforms cartesian coordinates into latitude and longitude

        Parameters
        ----------

        east: float
            East (x) coordinate in meters

        north: float
            North (y) coordinate in meters

        Returns
        -------

        np.array of shape (2, )
            Geodetic coordinates
        """
        arr = np.array([east, north]) / np.array([map.scale_x, map.scale_y])
        arr = np.array([np.array([*self.transformer.invtransform(east, north)])])

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
        segment_size = 100
        path_segments = [new_path[i:i + segment_size] for i in range(0, len(new_path), segment_size)]
        for segment in path_segments:
            segment_result = self.gmaps.nearest_roads(segment)
            for step in segment_result:
                if originalIndex != step["originalIndex"]:
                    temp.append(
                        [step["location"]["latitude"], step["location"]["longitude"]]
                    )
                    originalIndex = step["originalIndex"]

        origin_coord = self.transformer.transform(start[0], start[1])

        coords = np.zeros((len(temp), 2))
        for i in range(len(temp)):
            coords[i] = self.get_coordinates(temp[i][0], temp[i][1])

        return coords
    def round_path(self, path: np.array) -> np.array:
        points_to_remove = list()
        for i,point in enumerate(path):
            if i > 3 and i < len(path) -3:
                upfront = path[i+1] - point
                back = point - path[i-1]
                if(abs(np.tensordot(upfront,back, axes=1)) <0.01):
                    new_point = path[i-3] + path[i+3] - point
                    vector =  (point-new_point)
                    angle = np.arctan2(vector[1], vector[0])
                    vector = np.linalg.norm(vector)*0.8* np.array([np.cos(angle),np.sin(angle)])
                    desired_point = vector + new_point
                    path[i][0] ,path[i][1] =  desired_point[0],desired_point[1]
                    points_to_remove.append(i-1)
                    points_to_remove.append(i+1)
                    points_to_remove.append(i+2)
                #path = path.reshape((-1,2))
        path = np.delete(path,points_to_remove,axis=0)
        return path

    def orient_path(self, path: np.array) -> np.array:
        oriented_path = np.zeros(shape=(len(path), 4))

        for i in range(0, len(path)):
            oriented_path[i][:2] = path[i]

            if i < len(path) - 1:
                # Add phi
                oriented_path[i][3] = np.arctan2(
                    (path[i + 1][1] - path[i][1]), (path[i + 1][0] - path[i][0])
                )
            else:
                oriented_path[i][3] = oriented_path[i - 1][3]
            oriented_path[i][2] = oriented_path[i][3]

        return oriented_path
