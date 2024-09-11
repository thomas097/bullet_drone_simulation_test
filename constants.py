
from math import sqrt, sin

class WGS84:
    """General parameters defined by the WGS84 system"""

    r_earth = 6378000.0

    # Semimajor axis length (m)
    a = 6378137.0

    # Semiminor axis length (m)
    b = 6356752.3142

    # Ellipsoid flatness (unitless)
    f = (a - b) / a

    # Eccentricity (unitless)
    e = sqrt(f * (2 - f))

    # Speed of light (m/s)
    c = 299792458.

    # Relativistic constant
    F = -4.442807633e-10

    # Earth's universal gravitational constant
    mu = 3.986005e14

    # Earth rotation rate (rad/s)
    omega_ie = 7.2921151467e-5

    def g0(self, L):
        """
        acceleration due to gravity at the ellipsoid surface at latitude L
        
        Note: this is not the WGS84 models for gravity at latitude,
              it is an approximation from another source.     
        """
        return 9.7803267715 * (1 + 0.001931851353 * sin(L)**2) / sqrt(1 - 0.0066943800229 * sin(L)**2)