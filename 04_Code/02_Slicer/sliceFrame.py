
import rhinoscriptsyntax as rs
import Rhino.Geometry as rg
import scriptcontext as sc
import math
import collections

class SliceFrame(object):
    """This class creates a slice point
    """

    def __init__(self, point):
        """initiates the class
        """
        self.point = point
        self.cantiliver = None


    def calc_frame(self, point, tcp_vector, normal):
        """calculate frame
        """
        vector_y = rg.Vector3d(self.cross_product(tcp_vector, normal))
        vector_x = rg.Vector3d(self.cross_product(normal, vector_y))
        tcp_frame = rg.Plane(point, vector_y, vector_x)

        self.frame = tcp_frame
        return tcp_frame


    def cross_product(self, vector_a, vector_b):
        """Return the perpendicular vector
        from two given vectors
        """
        a_x, a_y, a_z = vector_a[0], vector_a[1], vector_a[2]
        b_x, b_y, b_z = vector_b[0], vector_b[1], vector_b[2]
        c_x, c_y, c_z = (a_y*b_z - a_z*b_y),(a_z*b_x - a_x*b_z), (a_x*b_y - a_y*b_x)

        vector_c = rg.Vector3d(c_x, c_y, c_z)

        return vector_c


    def calc_cantiliver_normal(self, point, curve):
        """calculates cantiliver normal
        """
        _, c_double = curve.ClosestPoint(point)
        c_point = curve.PointAt(c_double)

        vector01 = rg.Vector3d(c_point)
        vector02 = rg.Vector3d(point)
        p_normal = vector02 - vector01
        p_normal.Unitize()

        return p_normal


    # @property
    # def SCurve(self):
    #     """gets SCurve
    #     """
    #     return self.curve
    #
    # @SCurve.setter
    # def SCurve(self, curve):
    #     """sets SCure
    #     """
    #     self.seam = curve.PointAtStart
    #     self.points = self.resample_points_by_count(curve,
    #         self.line_definition, self.append_bool)
    #     self.center = self.get_center_of_points(self.points)
    #     self.area = self.get_area(curve)
    #
    # @SCurve.deleter
    # def SCurve(self):
    #     """deletes SCurve
    #     """
    #     self.seam = None
    #     self.center = None
    #     self.area = None
    #     self.points = None
