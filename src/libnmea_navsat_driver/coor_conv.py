import pyproj
import math

DEG_TO_RAD = 0.01745329252
EARTH_MAJOR = 6378137.0
EARTH_MINOR = 6356752.31424518

ECEF = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
LLA = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')

def lla2ecef(lon, lat, alt):
  x, y, z = pyproj.transform(LLA, ECEF, lon, lat, alt)
  return x, y, z

def lla2ecef_simple(lon, lat, alt):
  lat = math.radians(lat)
  lon = math.radians(lon)
  earth_r = math.pow(EARTH_MAJOR, 2) / math.sqrt(pow(EARTH_MAJOR * math.cos(lat), 2) + math.pow(EARTH_MINOR * math.sin(lat), 2))
  x = (earth_r + alt) * math.cos(lat) * math.cos(lon)
  y = (earth_r + alt) * math.cos(lat) * math.sin(lon)
  z = (math.pow(EARTH_MINOR / EARTH_MAJOR, 2) * earth_r + alt) * math.sin(lat)
  return x, y, z

if __name__ == '__main__':

  import time
  start = time.time()
  for i in range(1000):
    x, y, z = lla2ecef(106.2, 26.0, 1234.0)
  end = time.time()
  print(1.0/((end-start)/1000))
  print(lla2ecef(106.2, 26.4, 1231.5))
  start = time.time()
  for i in range(1000):
    x, y, z = lla2ecef_simple(106.2, 26.4, 1231.5)
  end = time.time()
  print(1.0/((end-start)/1000))
  print(lla2ecef_simple(106.2, 26.4, 1231.5))
  