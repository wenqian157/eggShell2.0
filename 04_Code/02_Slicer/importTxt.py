
import Rhino.Geometry as rg

class ImportTxt():

    def __init__(self, file_path):

        self.file_path = file_path

    def get_points(self):

        WCS_list = []

        with open(self.file_path, "r") as f:

            for line in f.readlines():

                data = line[line.find("[")+1 : line.find("]")].rstrip().split(',')

                point = rg.Point3d(float(data[0]), float(data[1]), float(data[2]))
                vector_x = rg.Vector3d(float(data[3]), float(data[4]), float(data[5]))
                vector_y = rg.Vector3d(float(data[6]), float(data[7]), float(data[8]))
                speed = int(data[9])
                radius = float(data[10])
                extrude = int(data[11])

                WCS = [point, vector_x, vector_y, speed, radius, extrude]

                WCS_list.append(WCS)

        return WCS_list
