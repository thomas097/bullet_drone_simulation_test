from math import atan2, sqrt, atan, cos, sin, asin
import numpy as np
from components.constants import WGS84
from components.datatypes import GPS


def ecef2lla(ecef: np.ndarray, tolerance=1e-9):
    """Convert Earth-centered, Earth-fixed coordinates to lat, lon, alt (GPS).
    """
    x, y, z = ecef

    # Calculate lon
    lon = atan2(y, x)

    # Initialize the variables to calculate lat and alt
    alt = 0
    N = WGS84.a
    p = sqrt(x**2 + y**2)
    lat = 0
    previousLat = 90

    # Special case for poles (note: any longitude value is valid here)
    if x == 0 and y == 0:   
        if z > 0:
            return (90.00, 69.00, round(z - WGS84.b, 2))

        elif z < 0:
            return (-90.00, 69.00, round(-z - WGS84.b, 2))

        else:
            raise Exception("The Exact Center of the Earth has no lat/lon")

    # Iterate until tolerance is reached
    while abs(lat - previousLat) >= tolerance:
        previousLat = lat
        sinLat = z / (N * (1 - WGS84.e**2) + alt)
        lat = atan((z + WGS84.e**2 * N * sinLat) / p)
        N = WGS84.a / sqrt(1 - (WGS84.e * sinLat)**2)
        alt = p / cos(lat) - N

    # Return the lla coordinates
    return GPS(latitude=np.rad2deg(lat), longitude=np.rad2deg(lon), altitude=alt)


def lla2ecef(gps: GPS) -> np.ndarray:
    """
    Convert GPS location to Earth-centered, Earth-fixed (ECEF) coordinates.
    """
    # Decompose the input
    lat = np.deg2rad(gps.latitude)
    lon = np.deg2rad(gps.longitude)

    # Calculate length of the normal to the ellipsoid
    N = WGS84.a / sqrt(1 - (WGS84.e * sin(lat))**2)

    # Calculate ecef coordinates
    x = (N + gps.altitude) * cos(lat) * cos(lon)
    y = (N + gps.altitude) * cos(lat) * sin(lon)
    z = (N * (1 - WGS84.e**2) + gps.altitude) * sin(lat)

    # Return the ecef coordinates
    return np.array([x, y, z], dtype=np.float32)


def haversine_distance(loc1: GPS, loc2: GPS):
    """
    Calculate the great circle distance in meters between two points 
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(np.deg2rad, [loc1.longitude, loc1.latitude, loc2.longitude, loc2.latitude])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371000 # Radius of earth in meters.
    return c * r