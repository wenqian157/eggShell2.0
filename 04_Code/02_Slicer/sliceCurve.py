
import rhinoscriptsyntax as rs
import Rhino.Geometry as rg
import scriptcontext as sc
import math
import collections

class SliceCurve(object):
    """This class creates a slice curve
    """

    def __init__(self, curve, line_definition):

        self.curve = curve
        self.line_definition = line_definition
        self.seam = None
        self.center = self.get_center_of_curve()
        self.area = self.get_area()
        # self.center = None

    # @property
    # def getCurve(self):
    #     return self.curve
    #
    # @getCurve.setter
    # def setCurve(self, curve):
    #     self.curve = curve
    #     self.seam = curve.PointAtStart
    #     # self.center = self.get_center_of_curve()
    #     # self.area = self.get_area()
    #
    # @property
    # def getSeam(self):
    #     return self.seam
    #
    # @getSeam.setter
    # def setSeam(self, value):
    #     self.seam = self.curve.

    def get_area(self):
        """calculates area of planar curve
        """

        area = rg.AreaMassProperties.Compute(self.curve)
        return area.Area


    def is_almost_equal(self, x ,y ,epsilon=1*10**(-8)):

    	"""Return True if two values are close in numeric value
    		By default close is withing 1*10^-8 of each other
            i.e. 0.00000001
    	"""
    	return abs(x-y) <= epsilon


    def calculate_normals(self, closest_curve):
        """calculates the cantiliver normal for each point
        on the curve
        """

        self.normals = []

        for i,p in enumerate(self.points):

            _, c_double = closest_curve.ClosestPoint(p)
            c_point = closest_curve.PointAt(c_double)

            vector01 = rg.Vector3d(c_point)
            vector02 = rg.Vector3d(p)

            p_normal = vector02 - vector01
            p_normal.Unitize()
            angle = self.angle_two_vectors(rg.Vector3d(0,0,1), p_normal)

            if angle < 0.45:
                p_normal = z_vector

            # perp_vector = rg.Vector3d(0,0,1)
            # p_normal = p_normal + perp_vector
            # p_normal = p_normal + perp_vector

            self.normals.append(p_normal)


    def resample_points_by_count(self, append_point_0=True):
        """resamples the curve by count of points
        """

        length = self.curve.GetLength()
        points_per_curve = int(round(length/self.line_definition))
        divisions = self.curve.DivideByCount(points_per_curve, True)
        points = [self.curve.PointAt(d) for d in divisions]

        if append_point_0:
            points.append(points[0])

        return points


    def get_center_of_curve(self):
        """gets the center point of the curve
        """

        sum_x = 0
        sum_y = 0

        points = self.resample_points_by_count(self.curve)

        for p in points:

            sum_x += p.X
            sum_y += p.Y

        avrg_x = sum_x/len(points)
        avrg_y = sum_y/len(points)

        avrg_p = rg.Point3d(avrg_x, avrg_y, points[0].Z)

        return avrg_p
