import numpy as np

def all_pts_close(pts, tol=0.0000001):
    if not pts:
        return True
    pts = np.array(pts)
    in_tol = [np.all(np.abs(pts[:, i] - np.mean(pts[:, i])) < tol) for i in range(2)]
    return np.all(in_tol)
