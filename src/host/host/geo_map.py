import math

_RADIUS_OF_EARTH=6371000.0
MATH_PI=3.141592653589793238462643383280

def MATH_RAD(a):
    return a*MATH_PI/180.0

def MATH_DEG(a):
    return a*180.0/MATH_PI

def limit(val,min,max):
    if val < min:
        return min

    if val > max:
        return max
    
    return val

def geo_project(lat_0,lon_0,lat,lon):
    ref_lat = MATH_RAD(lat_0)
    ref_lon = MATH_RAD(lon_0)
    ref_sin_lat = math.sin(ref_lat)
    ref_cos_lat = math.cos(ref_lat)

    lat_rad = MATH_RAD(lat)
    lon_rad = MATH_RAD(lon)

    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)

    cos_d_lon = math.cos(lon_rad - ref_lon)

    arg = limit(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon, -1.0, 1.0)
    c = math.acos(arg)

    k = 1.0

    if math.fabs(c) > 0.0:
        k = (c / math.sin(c))

    x = float(k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * _RADIUS_OF_EARTH)
    y = float(k * cos_lat * math.sin(lon_rad - ref_lon) * _RADIUS_OF_EARTH)

    return [x,y]

def geo_reproject(lat_0,lon_0,x,y):
    ref_lat = MATH_RAD(lat_0)
    ref_lon = MATH_RAD(lon_0)
    ref_sin_lat = math.sin(ref_lat)
    ref_cos_lat = math.cos(ref_lat)

    x_rad = x / _RADIUS_OF_EARTH
    y_rad = y / _RADIUS_OF_EARTH
    c = math.sqrt(x_rad * x_rad + y_rad * y_rad)

    if math.fabs(c) > 0.0:
        sin_c = math.sin(c)
        cos_c = math.cos(c)
        lat_rad = math.asin(cos_c * ref_sin_lat + (x_rad * sin_c * ref_cos_lat) / c)
        lon_rad = (ref_lon + math.atan2(y_rad * sin_c, c * ref_cos_lat * cos_c - x_rad * ref_sin_lat * sin_c))
        lat = MATH_DEG(lat_rad)
        lon = MATH_DEG(lon_rad)
    else:
        lat = MATH_DEG(ref_lat)
        lon = MATH_DEG(ref_lon)
    
    return [lat,lon]