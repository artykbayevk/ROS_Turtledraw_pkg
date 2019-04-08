class Pose():
    def __init__(self, x, y, theta=0):
        self.theta = theta
        self.y = y
        self.x = x
    def __repr__(self):
        return str(self)