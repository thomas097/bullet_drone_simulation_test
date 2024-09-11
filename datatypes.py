from dataclasses import dataclass

@dataclass
class GPS:
    latitude: float
    longitude: float
    altitude: float

@dataclass
class Track:
    id: int
    location: GPS
    priority: float = 1.0

@dataclass
class StateMessage:
    id: int
    location: GPS
    waypoint: GPS

@dataclass
class EntityState:
    id: int
    location: GPS
    waypoint: GPS
    recv_messages: list[StateMessage]
    tracks: set[Track]