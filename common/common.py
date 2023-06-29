import numpy as np

def rot(angle):
    return np.array([[ np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])

def drot(angle):
    return np.array([[ -np.sin(angle), -np.cos(angle)], [np.cos(angle), -np.sin(angle)]])

class Rect():
    '''
    Simple rectangle.
    '''
    def __init__(self, sides, center_offset):
        self.length = sides[0]
        self.width = sides[1]
        self.center_offset = center_offset

    def get_center(self, pose):
        x, y, angle = pose[0], pose[1], pose[2]
        return ( x - self.center_offset*np.cos(angle), y - self.center_offset*np.sin(angle) )

    def get_corners(self, pose, *args):
        x, y, angle = pose[0], pose[1], pose[2]

        topleft = ( rot(angle) @ np.array([-self.length/2, self.width/2]) + np.array( self.get_center(pose) ) ).tolist()
        topright = ( rot(angle) @ np.array([ self.length/2, self.width/2]) + np.array( self.get_center(pose) ) ).tolist()
        bottomleft = ( rot(angle) @ np.array([-self.length/2, -self.width/2]) + np.array( self.get_center(pose) ) ).tolist()
        bottomright = ( rot(angle) @ np.array([ self.length/2, -self.width/2]) + np.array( self.get_center(pose) ) ).tolist()

        if len(args):
            if 'topleft' in args[0].lower():
                return topleft
            elif 'topright' in args[0].lower():  
                return topright
            elif 'bottomleft' in args[0].lower():
                return bottomleft
            elif 'bottomright' in args[0].lower():
                return bottomright
            else:
                return [ topleft, topright, bottomleft, bottomright ]
        else:
            return [ topleft, topright, bottomleft, bottomright ]
        
class EllipticalBarrier():
    '''
    Elliptical barrier
    '''
    def __init__(self, shape=[1,1]):

        self.center = np.zeros(2)
        self.orientation = 0.0

        self.shape = shape
        self.H = rot(self.orientation).T @ np.diag(self.shape) @ rot(self.orientation)
        self.dH = drot(self.orientation).T @ np.diag(self.shape) @ rot(self.orientation) + rot(self.orientation).T @ np.diag(self.shape) @ drot(self.orientation)

    def update_pose(self, new_pose):
        self.center = new_pose[0:2]
        self.orientation = new_pose[2]
        self.H = rot(self.orientation).T @ np.diag(self.shape) @ rot(self.orientation)
        self.dH = drot(self.orientation).T @ np.diag(self.shape) @ rot(self.orientation) + rot(self.orientation).T @ np.diag(self.shape) @ drot(self.orientation)

    def function(self, value):
        return 0.5 * ( value - self.center ) @ self.H @ ( value - self.center ) - 0.5
    
    def gradient(self, value):
        return np.hstack( [ (-(value - self.center) @ self.H).reshape(2), ( value - self.center ).T @ self.dH @ ( value - self.center ) ])