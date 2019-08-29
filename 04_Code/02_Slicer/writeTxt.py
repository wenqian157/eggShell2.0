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


    def write_txt_file(self):
        """create full file_path
        """
        if ".txt" not in self.file_name: self.file_name += ".txt"
        now = dt.now().strftime("%Y%m%d_%H%M%S_")
        self.file_path = self.dir_path + now + self.file_name

        with open(self.file_path, "w+") as f:

            for d in self.data:
                data = str(d)
                data = "".join(data.split())  # remove any whitespace in the line
                f.write(data + "\n")

        print "file written in : %s" % (self.file_path)
