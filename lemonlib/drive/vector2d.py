import math


class Vector2d:
    def __init__(self, x=0.0, y=0.0):
        self.x = x  # forward component
        self.y = y  # right component

    def rotate(self, angle_deg):
        """Rotate this vector counter-clockwise by angle_deg degrees."""
        angle_rad = math.radians(angle_deg)
        cosA = math.cos(angle_rad)
        sinA = math.sin(angle_rad)
        x_new = self.x * cosA - self.y * sinA
        y_new = self.x * sinA + self.y * cosA
        self.x = x_new
        self.y = y_new

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def magnitude(self):
        return math.hypot(self.x, self.y)

    def scalarProject(self, other):
        """Returns the scalar projection of this vector onto 'other'."""
        mag = other.magnitude()
        if mag == 0:
            return 0.0
        return self.dot(other) / mag
