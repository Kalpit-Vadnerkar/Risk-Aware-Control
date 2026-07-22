from math import sqrt


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"Point({self.x}, {self.y})"

    @staticmethod
    def convert_coordinate_frame(x_A, y_A, reference_points):
        if len(reference_points) < 2:
            raise ValueError("At least two reference points are required for conversion.")

        (x_A1, y_A1), (x_B1, y_B1) = reference_points[0]
        (x_A2, y_A2), (x_B2, y_B2) = reference_points[1]

        a = (x_B2 - x_B1) / (x_A2 - x_A1)
        c = (y_B2 - y_B1) / (y_A2 - y_A1)
        b = x_B1 - a * x_A1
        d = y_B1 - c * y_A1

        x_B = a * x_A + b
        y_B = c * y_A + d

        return Point(x_B, y_B)

    @staticmethod
    def get_mid_point(point1, point2):
        mx = (point1.x + point2.x) / 2
        my = (point1.y + point2.y) / 2
        return Point(mx, my)

    @staticmethod
    def distance(point1, point2):
        return sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))

    def scale(self, x_min, x_max, y_min, y_max):
        scaled_x = self.clamp((self.x - x_min) / (x_max - x_min), 0, 1)
        scaled_y = self.clamp((self.y - y_min) / (y_max - y_min), 0, 1)
        return Point(scaled_x, scaled_y)
