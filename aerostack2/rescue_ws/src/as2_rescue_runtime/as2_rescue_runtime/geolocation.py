"""Calibrated optical ray / surveyed horizontal surface intersection, ENU metres.
Not a height-of-person estimator. Flood water/roofs require a measured surface.
"""
import math

def locate(pixel, intrinsics, rotation, origin, surface_z, position_sigma, angle_sigma_rad,
           surface_sigma, surface_verified=False, max_range=150.0):
    numbers=list(pixel)+list(intrinsics)+[x for row in rotation for x in row]+list(origin)+[surface_z,position_sigma,angle_sigma_rad,surface_sigma,max_range]
    if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in numbers):raise ValueError('Nonfinite geometry')
    if not surface_verified:raise ValueError('Surface elevation is unverified')
    fx,fy,cx,cy=intrinsics
    if min(fx,fy,max_range)<=0 or min(position_sigma,angle_sigma_rad,surface_sigma)<0:raise ValueError('Invalid calibration/uncertainty')
    for i in range(3):
        for j in range(3):
            if abs(sum(rotation[k][i]*rotation[k][j] for k in range(3))-(1 if i==j else 0))>1e-5:raise ValueError('Rotation is not orthonormal')
    det=sum(rotation[0][i]*(rotation[1][(i+1)%3]*rotation[2][(i+2)%3]-rotation[1][(i+2)%3]*rotation[2][(i+1)%3]) for i in range(3))
    if det<.999:raise ValueError('Rotation must be right handed')
    ray=[(pixel[0]-cx)/fx,(pixel[1]-cy)/fy,1.0]
    ray=[sum(rotation[i][j]*ray[j] for j in range(3)) for i in range(3)]
    norm=math.sqrt(sum(v*v for v in ray));ray=[v/norm for v in ray]
    if ray[2]>=-.2:raise ValueError('Ray is near horizon or not directed down')
    distance=(surface_z-origin[2])/ray[2]
    if not 0<distance<=max_range:raise ValueError('Intersection is behind camera or beyond range')
    point=[origin[i]+distance*ray[i] for i in range(3)]
    sigma=math.sqrt(position_sigma**2+(distance*angle_sigma_rad/abs(ray[2]))**2+(surface_sigma/abs(ray[2]))**2)
    return dict(position=point,sigma_m=sigma,slant_range_m=distance)
