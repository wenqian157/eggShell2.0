from __future__ import division
import Rhino.Geometry as rg
from datetime import datetime as dt
import scriptcontext as sc
import math


class WriteTxt():

    def __init__(self, points, dir_path, file_name, scale_factor, start_z,
                 pallet_value_x, pallet_value_y, flip_bucket_position):

        self.points = points
        self.dir_path = dir_path
        self.file_name = file_name
        self.pallet_value_x = pallet_value_x
        self.pallet_value_y = pallet_value_y
        self.scale_factor = scale_factor
        self.start_z = start_z
        self.flip_bucket_position = flip_bucket_position

        self.speed = str(265)
        self.zone_value = str(50)

        if self.flip_bucket_position:

            self.base_gantry_values = [28000, -5600, -2600]
            self.bucket_point = rg.Point3d(self.pallet_value_x, 1500, 400)

        else:

            self.base_gantry_values = [28000, -4000, -2600]
            self.bucket_point = rg.Point3d(self.pallet_value_x, -250, 400)

        # self.safe_entry_point = rg.Point3d(self.pallet_value_x, self.pallet_value_y, 50)
        # self.safe_exit_point = rg.Point3d(self.pallet_value_x, self.pallet_value_y, self.points[-1].Z + 50)

    def scale_points(self):

        if self.scale_factor != 1:

            for p in self.points:

                scale_transform = rg.Transform.Scale(rg.Plane.WorldXY, self.scale_factor,
                                                     self.scale_factor, self.scale_factor)

                p.Transform(scale_transform)

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

        # self.print_points.append(self.bucket_point) #bucket point

    def write_txt_file(self):
        """create full file_path
        """
        if ".txt" not in self.file_name:
            self.file_name += ".txt"

        now = dt.now().strftime("%Y%m%d_%H%M%S_")
        self.file_path = self.dir_path + now + self.file_name

        print self.file_path

        base_gantry_x = self.base_gantry_values[0]
        base_gantry_y = self.base_gantry_values[1]
        base_gantry_z = self.base_gantry_values[2]

        with open(self.file_path, "w+") as f:

            """go to bucket point
            """
            bucket_point_values = "[{0:.2f},{1:.2f},{2:.2f}]".format(round(self.bucket_point.X, 2),
                                                                     round(self.bucket_point.Y, 2), round(self.bucket_point.Z, 2))
            gantry_values = str([base_gantry_x, base_gantry_y, base_gantry_z])
            data = bucket_point_values + "," + \
                "[0.000000,0.000000,1.000000,0.000000]" + "," + \
                gantry_values + "," + "200" + "," + "Zone.fine"
            data = "".join(data.split())  # remove any whitespace in the line
            f.write(data + "\n")

            # """go fast to the center of the base (same height as bucket)
            # """
            # safe_entry_point1_values = "[{0:.2f},{1:.2f},{2:.2f}]".format(round(self.safe_entry_point.X, 2),
            #     round(self.safe_entry_point.Y, 2), round(self.bucket_point.Z, 2))
            # gantry_values = str([base_gantry_x, base_gantry_y, base_gantry_z])
            # data = safe_entry_point1_values + "," + "[0.000000,0.000000,1.000000,0.000000]" + "," + gantry_values + "," + "500" + "," + "Zone.z20"
            # data = "".join(data.split()) #remove any whitespace in the line
            # f.write(data + "\n")
            #
            # """go slowly to the correct start height
            # """
            # safe_entry_point2_values = "[{0:.2f},{1:.2f},{2:.2f}]".format(round(self.safe_entry_point.X, 2),
            #     round(self.safe_entry_point.Y, 2), round(self.safe_entry_point.Z, 2))
            # gantry_values = str([base_gantry_x, base_gantry_y, base_gantry_z])
            # data = safe_entry_point2_values + "," + "[0.000000,0.000000,1.000000,0.000000]" + "," + gantry_values + "," + "100" + "," + "Zone.z20"
            # data = "".join(data.split()) #remove any whitespace in the line
            # f.write(data + "\n")

            """go to print path
            """
            for point in self.points:

                point_values = "[{0:.2f},{1:.2f},{2:.2f}]".format(
                    round(point.X, 2), round(point.Y, 2), round(point.Z, 2))

                """define gantry values (move up until 2.3m then stop)
                """
                if point.Z < 2300:
                    gantry_values = str(
                        [base_gantry_x, base_gantry_y, int(base_gantry_z - round(point.Z, 1))])
                else:
                    gantry_values = str(
                        [base_gantry_x, base_gantry_y, base_gantry_z - 2300])

                zone = "Zone.z" + self.zone_value

                data = point_values + "," + \
                    "[0.000000,0.000000,1.000000,0.000000]" + "," + \
                    gantry_values + "," + self.speed + "," + zone
                data = "".join(data.split())  # remove any whitespace in the line
                f.write(data + "\n")

            """go to safety bucket point
            """
            safe_exit_point_values = "[{0:.2f},{1:.2f},{2:.2f}]".format(round(self.bucket_point.X, 2),
                                                                        round(self.bucket_point.Y, 2), round(self.points[-1].Z, 2))

            if self.points[-1].Z < 2300:
                gantry_values = str([base_gantry_x, base_gantry_y, int(
                    base_gantry_z - round(self.points[-1].Z, 1))])
            else:
                gantry_values = str(
                    [base_gantry_x, base_gantry_y, base_gantry_z - 2300])

            data = safe_exit_point_values + "," + \
                "[0.000000,0.000000,1.000000,0.000000]" + "," + \
                gantry_values + "," + "200" + "," + "Zone.fine"
            data = "".join(data.split())  # remove any whitespace in the line
            f.write(data + "\n")

            # """go to bucket point
            # """
            # gantry_values = str([base_gantry_x, base_gantry_y, base_gantry_z])
            # data = bucket_point_values + "," + "[0.000000,0.000000,1.000000,0.000000]" + "," + gantry_values + "," + "200" + "," + "Zone.fine"
            # data = "".join(data.split()) #remove any whitespace in the line
            # f.write(data + "\n")

            # """go to safety exit point (center of the top)
            # """
            # safe_exit_point1_values = "[{0:.2f},{1:.2f},{2:.2f}]".format(round(self.safe_exit_point.X, 2), round(self.safe_exit_point.Y, 2), round(self.safe_exit_point.Z, 2))
            # data = safe_exit_point1_values + "," + "[0.000000,0.000000,1.000000,0.000000]" + "," + gantry_values + "," + "500" + "," + "Zone.fine" #go fast to center, a bit above top of the column
            # data = "".join(data.split()) #remove any whitespace in the line
            # f.write(data + "\n")
            #
            # """wait 30seconds to stop manualy the pumps and have less concrete flowing
            # """
            # safe_exit_point2_values = "[{0:.2f},{1:.2f},{2:.2f}]".format(round(self.safe_exit_point.X + 30, 2), round(self.safe_exit_point.Y, 2), round(self.safe_exit_point.Z, 2)) # x + 30mm = 30seconds
            # data = safe_exit_point2_values + "," + "[0.000000,0.000000,1.000000,0.000000]" + "," + gantry_values + "," + "1" + "," + "Zone.fine" #stay at center position (move very slowly)
            # data = "".join(data.split()) #remove any whitespace in the line
            # f.write(data + "\n")

        print "file written in : %s" % (self.file_path)
