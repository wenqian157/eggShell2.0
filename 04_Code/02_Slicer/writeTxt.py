from __future__ import division
import Rhino.Geometry as rg
from datetime import datetime as dt
import scriptcontext as sc
import math


class WriteTxt():

    def __init__(self, data, dir_path, file_name):

        self.data = data
        self.dir_path = dir_path
        self.file_name = file_name
        # self.pallet_value_x = pallet_value_x
        # self.pallet_value_y = pallet_value_y
        # self.start_z = start_z

        # self.speed = str(265)
        # self.zone_value = str(50)

    def move_to_pallet(self):

        avgx = 0
        avgy = 0
        min_z = 9999

        self.circle_center = rg.Point3d(
            self.pallet_value_x, self.pallet_value_y, self.start_z)

        for p in self.points:

            avgx += p.X
            avgy += p.Y

            if p.Z < min_z:

                min_z = p.Z

        x = avgx / len(self.points)
        y = avgy / len(self.points)

        center = rg.Point3d(x, y, min_z)

        vector = -rg.Vector3d(center.X - self.circle_center.X, center.Y -
                              self.circle_center.Y, center.Z - self.circle_center.Z)

        """move to the pallet and check for errors
        """
        for p, p01 in zip(self.points, self.points):

            distance = p.DistanceTo(p01)

            if distance > 100:

                self.error = "ERROR"
                break

            else:

                self.error = "NO ERROR"
                translation = rg.Transform.Translation(vector.X, vector.Y, vector.Z)
                p.Transform(translation)


    def rotate_points(self):

        rotation_angle = math.pi

        for p in self.points:

            translation = rg.Transform.Rotation(-rotation_angle,
                                                rg.Plane.WorldXY.Normal, self.circle_center)
            p.Transform(translation)


    def cull_end(self):

        for i in range(len(self.points) - 1, -1, -1):

            p = self.points[i]
            vec1 = rg.Vector3d(p.X - self.circle_center.X,
                               p.Y - self.circle_center.Y, 0)

            if self.flip_bucket_position:

                vec2 = rg.Vector3d(0, 1, 0)

            else:

                vec2 = rg.Vector3d(0, -1, 0)

            vec1.Unitize()
            vec2.Unitize()

            angle1 = math.atan2(vec1.Y, vec1.X)
            angle2 = math.atan2(vec2.Y, vec2.X)

            angle_diff = math.degrees(abs(angle1 - angle2))

            if angle_diff > 10:

                self.points.pop(i)

            else:

                break


    def visualize_points(self):

        self.print_points = []

        self.print_points.append(self.bucket_point)  # bucket point

        self.print_points = self.print_points + self.points  # print path

        safe_exit_point = rg.Point3d(
            self.bucket_point.X, self.bucket_point.Y, self.points[-1].Z)  # safety exit point
        self.print_points.append(safe_exit_point)


    def write_txt_file(self):

        """create full file_path
        """
        if ".txt" not in self.file_name:
            self.file_name += ".txt"

        now = dt.now().strftime("%Y%m%d_%H%M%S_")
        self.file_path = self.dir_path + now + self.file_name

        print self.file_path

        with open(self.file_path, "w+") as f:

            for d in self.data:

                data = str(d)
                print data
                data = "".join(data.split())  # remove any whitespace in the line
                f.write(data + "\n")

        print "file written in : %s" % (self.file_path)
