import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from shapely import Polygon, Point
from typing import Iterable
from components.constants import WGS84
from components.datatypes import EntityState, Track, GPS, StateMessage
from components.conversions import ecef2lla, lla2ecef, haversine_distance


class Geofence:
    def __init__(self, coordinates: Iterable[GPS], max_altitude: float) -> None:
        """
        Initializes a geofence polygon from a list of GPS coordinates and a maximum altitude (in meters).
        """
        vertices = [(c.latitude, c.longitude, c.altitude) for c in coordinates]
        self._polygon = Polygon(shell=vertices)
        self._max_altitude = max_altitude

    @property
    def bounds(self) -> tuple:
        """
        Returns bounding box around geofence
        """
        x_min, y_min, x_max, y_max = self._polygon.bounds
        return (x_min, y_min, 0.0, x_max, y_max, self._max_altitude)
    
    def contains(self, location: GPS) -> bool:
        """
        Tests whether location lies within geofence.
        """
        point = Point(location.latitude, location.longitude)
        return self._polygon.contains(point)
    
    def sample(self, n: int) -> list[GPS]:
        """
        Samples `n` GPS locations within geofence using rejection sampling.
        """
        x_min, y_min, x_max, y_max = self._polygon.bounds
        
        points = []
        while len(points) < n:
            lat = np.random.uniform(x_min, x_max)
            lon = np.random.uniform(y_min, y_max)

            point = GPS(latitude=lat, longitude=lon, altitude=0.0)
            if self.contains(point):
                points.append(point)

        return points


class Simulation:
    def __init__(self, n_drones: int, n_tracks: int, gcs: GPS, geofence: Geofence, max_velocity: float = 0.5, track_detection_range: float = 10.0, drone_communication_range: float = 10.0) -> None:
        """
        Initializes a Simulation object representing a 3D scene with `n_drones` and `n_tracks`, representing objects of interest.
        """
        self._ax = plt.figure().add_subplot(projection='3d')

        # Set environment limits
        x_min, y_min, z_min, x_max, y_max, z_max = geofence.bounds
        self._ax.set_xlim(x_min, x_max)
        self._ax.set_ylim(y_min, y_max)
        self._ax.set_zlim(z_min, z_max)

        # Set axis labels
        self._ax.set_xlabel('Latitude')
        self._ax.set_ylabel('Longitude')
        self._ax.set_zlabel('Altitude')

        # Init drones and potential tracks around GCS' location 
        self._drones = self._init_drones(n=n_drones, gcs=gcs)
        self._tracks = self._init_tracks(n=n_tracks, geofence=geofence)

        # Create drawing primitives for drones and tracks
        self._drones_plot = self._ax.scatter(
            xs=[d.location.latitude for d in self._drones],
            ys=[d.location.longitude for d in self._drones],
            zs=[d.location.altitude for d in self._drones],
            marker="o",
            color="C0"
        )

        self._tracks_plot = self._ax.scatter(
            xs=[t.location.latitude for t in self._tracks],
            ys=[t.location.longitude for t in self._tracks],
            zs=[t.location.altitude for t in self._tracks],
            marker="^",
            color="C2"
        )

        # Properties
        self._n_drones = n_drones
        self._n_tracks = n_tracks
        self._gcs = gcs
        self._geofence = geofence
        self._max_velocity = max_velocity
        self._track_detection_range = track_detection_range
        self._drone_communication_range = drone_communication_range

    def reset(self) -> list[EntityState]:
        self._drones = self._init_drones(n=self._n_drones, gcs=self._gcs)
        self._tracks = self._init_tracks(n=self._n_tracks, geofence=self._geofence)
        return self._drones

    def _init_drones(self, n: int, gcs: GPS, max_dist: float = 1.0) -> list[EntityState]:
        """
        Initializes `n` drones at most `max_dist` meters away from GCS.
        """
        # Sample random points around GCS with a distance of at most `max_dist` meters
        dx, dy = np.random.uniform(-max_dist, max_dist, size=(2, n))
        latitudes  = gcs.latitude + (dy / WGS84.r_earth) * (180 / np.pi)
        longitudes = gcs.longitude + (dx / WGS84.r_earth) * (180 / np.pi) / np.cos(gcs.latitude * np.pi / 180)

        # Convert coordinates to GPS objects
        locations = [GPS(latitude=latitudes[i], longitude=longitudes[i], altitude=0.0) for i in range(n)]

        # Init Drone object at each GPS location
        return [EntityState(id=i, location=locations[i], waypoint=None, recv_messages=[], tracks={}) for i in range(n)]
    
    def _init_tracks(self, n: int, geofence: Geofence) -> list[Track]:
        """
        Initializes `n` tracks at random locations within geofenced area.
        """
        locations = geofence.sample(n=n)
        return [Track(id=i, location=loc) for i, loc in enumerate(locations)]

    def update_waypoint(self, drone_id: int, waypoint: GPS) -> None:
        """
        Updates the waypoint of the controller.
        """
        assert drone_id >= 0, "drone_id must be positive"
        self._drones[drone_id].waypoint = waypoint

    def step(self, delay: float = 0.1) -> list[EntityState]:
        """
        Function to call to advance the state of the simulation.
        """
        # Update tracks detected by drones
        for drone in self._drones:
            for track in self._tracks:
                if haversine_distance(track.location, drone.location) < self._track_detection_range:
                    drone.tracks.add(track)

        # Update messages received by drones from other drones nearby
        for drone in self._drones:
            drone.recv_messages = [] # clear
            for other in self._drones:
                if drone.id != other.id and haversine_distance(drone.location, other.location)< self._drone_communication_range:
                    drone.recv_messages.append(StateMessage(id=other.id, location=other.location, waypoint=other.waypoint))
                    drone.tracks.update(other.tracks)

        # Update drone positions towards waypoint
        for drone in self._drones:
            if drone.waypoint is not None:
                # Calculate direction vector for drone in ECEF coordinates
                # capped to maximum velocity of drone
                ecef_drone_location = lla2ecef(gps=drone.location)
                ecef_direction = lla2ecef(gps=drone.waypoint) - ecef_drone_location
                if np.linalg.norm(ecef_direction) > self._max_velocity:
                    ecef_direction *= self._max_velocity / np.linalg.norm(ecef_direction)

                # Update GPS location of drone
                drone.location = ecef2lla(ecef=ecef_drone_location + ecef_direction)

        # Update visualization
        drones_x, drones_y, drones_z = zip(*[(d.location.latitude, d.location.longitude, d.location.altitude) for d in self._drones])
        self._drones_plot._offsets3d = (drones_x, drones_y, drones_z)

        tracks_x, tracks_y, tracks_z = zip(*[(t.location.latitude, t.location.longitude, t.location.altitude) for t in self._tracks])
        self._tracks_plot._offsets3d = (tracks_x, tracks_y, tracks_z)

        plt.draw()
        plt.pause(delay)

        return self._drones


