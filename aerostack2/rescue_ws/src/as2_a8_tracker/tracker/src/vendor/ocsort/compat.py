"""Small FilterPy-compatible helpers used by the vendored OC-SORT Kalman filter."""
import numpy as np

def reshape_z(z, dim_z, ndim):
    z=np.asarray(z,dtype=float)
    if z.ndim==0: z=z.reshape(1)
    if z.size!=dim_z: raise ValueError(f'expected {dim_z} observation values, got {z.size}')
    return z.reshape((dim_z,1) if ndim==2 else (dim_z,))

def pretty_str(name, value):
    return f'{name} = {value}'

def logpdf(x, mean=None, cov=1., allow_singular=True):
    x=np.asarray(x,dtype=float).reshape(-1)
    m=np.zeros_like(x) if mean is None else np.asarray(mean,dtype=float).reshape(-1)
    c=np.asarray(cov,dtype=float)
    if c.ndim==0: c=np.eye(x.size)*float(c)
    sign, logdet=np.linalg.slogdet(c)
    if sign<=0:
        if not allow_singular: raise ValueError('covariance must be positive definite')
        return float('-inf')
    delta=x-m
    return float(-.5*(x.size*np.log(2*np.pi)+logdet+delta@np.linalg.solve(c,delta)))
