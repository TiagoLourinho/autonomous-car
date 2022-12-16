import numpy as np
import matplotlib.pyplot as plt

from blocks import Map
from constants import ORIGIN
from pyproj import Transformer


def main():
    map = Map()

    civil = np.array([38.73726, -9.13984])

    points = map.get_directions(ORIGIN, civil)

    # plt.plot(
    #     points[:, 1],
    #     points[:, 0],
    # )
    # plt.show()
    
    # Please try 3857 epsg code used by Google Maps
    transform = Transformer.from_crs("epsg:4326", "+proj=utm +zone=10 +ellps=WGS84", always_xy=True,)
    xx, yy = transform.transform(points[:, 0], points[:, 1])
    
    fig = plt.figure()
    plt.grid()
    plt.gca().set_aspect('equal', 'box')
    plt.xlabel('Easting [m]', fontsize = 14)   
    plt.ylabel('Northing [m]', fontsize = 14) 
    plt.plot(xx, yy)
    plt.gca().set_aspect('equal', 'box')
    plt.show()
        
    # a Pythagore's theorem is sufficient to compute an approximate distance
    for i in range(len(xx) - 1):
        distance = np.sqrt((xx[i + 1] - xx[i]) ** 2 + (yy[i + 1] - yy[i]) ** 2)
        print(distance)
    
    
if __name__ == "__main__":
    main()
