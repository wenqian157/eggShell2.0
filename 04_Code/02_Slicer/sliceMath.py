
"""Slice Math
"""

import math


def angle_two_vectors(vector01, vector02):
    """calculates angle between two vectors
    """
    dot_pro = dot_product(vector01, vector02)
    len_01 = length_of_vector(vector01)
    len_02 = length_of_vector(vector02)
    try:
        angle = math.acos(dot_pro/(len_01*len_02))
    except:
        angle = 0

    return angle


def area_of_polygon(points):
    """Calculates the area of an arbitrary polygon given its verticies
    """
    x = [p[0] for p in points]
    y = [p[1] for p in points]
    area = 0.0
    try:
        for i in range(-1, len(x)-1):
            area += x[i] * (y[i+1] - y[i-1])
        return abs(area) / 2.0
    except:
        return None


def length_of_vector(vector):
    """calculates length of vector
    """
    return math.sqrt(vector[0]**2 + vector[1]**2 + vector[2]**2)


def dot_product(vector01, vector02):
    """calculates dot product
    """
    return vector01[0]*vector02[0] + vector01[1]*vector02[1] + vector01[2]*vector02[2]


def substract_two_vectors(vector_a, vector_b):
    """substracts two vectors
    """
    return [vector_a[0]-vector_b[0], vector_a[1]-vector_b[1], vector_a[2]-vector_b[2]]


def two_d_distance(point_a, point_b):
    """gets 2D distance between two points
    """
    dist = math.sqrt((point_a[0] - point_b[0])**2 + (point_a[1] - point_b[1])**2)
    return dist


def three_d_distance(point_a, point_b):
    """gets 2D distance between two points
    """
    return math.sqrt((point_a[0] - point_b[0])**2 + (point_a[1] - point_b[1])**2 + (point_a[2] - point_b[2])**2)


def is_almost_equal(x ,y ,epsilon=1*10**(-8)):
    """Return True if two values are close in numeric value
        By default close is withing 1*10^-8 of each other
        i.e. 0.00000001
    """
    return abs(x-y) <= epsilon


def get_two_d_center_of_points(points):
    """gets the center point of the curve
    """
    sum_x, sum_y = 0, 0

    for p in points:
        sum_x += p[0]
        sum_y += p[1]

    avrg_x, avrg_y = sum_x/len(points), sum_y/len(points)
    return [avrg_x, avrg_y, points[0][2]]


def get_three_d_center_of_points(points):
    """gets the center point of the curve
    """
    sum_x, sum_y, sum_z = 0, 0, 0

    for p in points:
        sum_x += p.X
        sum_y += p.Y
        sum_z += p.Z

    avrg_x, avrg_y, avrg_z  = sum_x/len(points), sum_y/len(points), sum_z/len(points)
    return [avrg_x, avrg_y, avrg_z]


def cross_product(vector_a, vector_b):
    """Return the perpendicular vector
    from two given vectors
    """
    a_x, a_y, a_z = vector_a[0], vector_a[1], vector_a[2]
    b_x, b_y, b_z = vector_b[0], vector_b[1], vector_b[2]
    c_x, c_y, c_z = (a_y*b_z - a_z*b_y),(a_z*b_x - a_x*b_z),(a_x*b_y - a_y*b_x)

    return [c_x, c_y, c_z]


def closest_points_from_points(point_list_a, point_list_b):
    """finds the two closest points from two lists of points
    """
    min_dist = 99999

    for p_a in point_list_a:
        for p_b in point_list_b:
            dist = two_d_distance(p_a,p_b)
            if dist<min_dist: min_dist,pair=dist,(p_a,p_b)

    return pair, min_dist


def closest_point_from_point(point, point_list):
    """finds the closest point to target point
    """
    min_dist = 99999

    for i,p in enumerate(point_list):
        dist = two_d_distance(p,point)
        if dist<min_dist: min_dist,min_i,min_p=dist,i,p

    return min_dist,min_i,min_p


def farthest_point_from_point(point, point_list):
    """finds the closest point to target point
    """
    max_dist = 0

    for i,p in enumerate(point_list):
        # print i
        dist = two_d_distance(p,point)
        # print dist
        if dist>max_dist:
            max_dist,max_i,max_p=dist,i,p
            # print max_dist
            # print i

        # print max_i
    return max_dist,max_i,max_p
