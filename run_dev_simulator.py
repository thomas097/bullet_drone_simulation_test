from copy import deepcopy
from components.datatypes import GPS
from components import Simulation, Geofence


if __name__ == '__main__':
    # Scene
    gcs = GPS(latitude=52.680572, longitude=5.922059, altitude=0.0)

    geofence = Geofence(
        coordinates=[
            GPS(latitude=52.678427, longitude=5.917254, altitude=0.0),
            GPS(latitude=52.684450, longitude=5.923230, altitude=0.0),
            GPS(latitude=52.681897, longitude=5.930645, altitude=0.0),
            GPS(latitude=52.675873, longitude=5.924886, altitude=0.0),
        ],
        max_altitude=100.0
    )

    # Init environment
    sim = Simulation(
        n_drones=10,
        n_tracks=5,
        gcs=gcs,
        geofence=geofence
    )

    # Simulate
    states = sim.reset()

    for _ in range(999):
        
        # DO SOMETHING WITH CONTROLLERS CONDITIONED ON STATES
        # UPDATE WITH SIM.UPDATE_WAYPOINT()

        states = sim.step(delay=1/30)
