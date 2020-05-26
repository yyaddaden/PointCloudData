# -*- coding: UTF-8 -*-

import os
import csv
from datetime import datetime
from geopy.distance import geodesic
import numpy as np
import open3d


class PointCloudData():

    def __init__(self, in_path, path_start, zone_perimeter, path_end, out_path):

        self.in_path = in_path
        self.out_path = out_path
        self.path_start = path_start
        self.zone_perimeter = zone_perimeter
        self.path_end = path_end

        self.time_gps_data = []
        self.time_obj_data = []
        self.gps_obj_data = []
        self.files_per_zone = {}

    def compute_distance(self, src, dst):

        gps_src = (src["latitude"], src["longitude"])
        gps_dst = (dst["latitude"], dst["longitude"])

        return geodesic(gps_src, gps_dst).meters

    def import_gps_data(self):
        print("Importing localization (gps) data ... ", end="", flush=True)

        gps_data_all = []

        for gps_file in os.listdir(self.in_path):
            if gps_file.endswith("gps.csv"):
                with open(self.in_path + gps_file) as csv_file:
                    reader = csv.DictReader(
                        csv_file, skipinitialspace=True, delimiter=",")

                    for row in reader:
                        row_datetime_str = row["Date"] + " " + row["Time"]
                        row_datetime = datetime.strptime(
                            row_datetime_str, "%Y-%m-%d %H:%M:%S.%f")

                        row_latitude = row["Latitude (°)"]
                        row_longitude = row["Longitude (°)"]

                        gps_data_all.append(
                            {"datetime": row_datetime, "latitude": row_latitude, "longitude": row_longitude})

        end_distances = []
        for elt in gps_data_all:
            end_distances.append(self.compute_distance(self.path_end, elt))

        end_idx = end_distances.index(min(end_distances))

        gps_data_all = gps_data_all[0:end_idx+1]

        start_distances = []
        for elt in gps_data_all:
            start_distances.append(self.compute_distance(self.path_start, elt))

        start_idx = start_distances.index(min(start_distances))

        self.time_gps_data = gps_data_all[start_idx:]

        print("[\u2713]")

    def import_obj_data(self):
        print("Importing point cloud (LiDAR) data ... ", end="", flush=True)

        for obj_file in os.listdir(self.in_path):
            if obj_file.endswith("Centre.obj"):

                obj_datetime_str = obj_file[:23]
                obj_datetime = datetime.strptime(
                    obj_datetime_str, "%Y-%m-%d %H-%M-%S-%f")

                if(obj_datetime <= self.time_gps_data[-1]["datetime"]):
                    self.time_obj_data.append(
                        {"datetime": obj_datetime, "file": obj_file})

        print("[\u2713]")

    def compute_gps_obj_data(self):

        print("Generating point cloud (LiDAR) with localization (gps) data ... ",
              end="", flush=True)

        for time_obj_elt in self.time_obj_data:

            datetime_diff = []

            for time_gps_elt in self.time_gps_data:
                datetime_diff.append(
                    abs(time_obj_elt["datetime"] - time_gps_elt["datetime"]))

            elt_idx = datetime_diff.index(min(datetime_diff))
            self.gps_obj_data.append({"file": time_obj_elt["file"], "latitude": self.time_gps_data[elt_idx]
                                      ["latitude"], "longitude": self.time_gps_data[elt_idx]["longitude"]})

        print("[\u2713]")

    def compute_files_per_zone(self):
        print("Dividing the route path into zones ... ", end="", flush=True)

        gps_point = self.gps_obj_data[0]
        distance = 0

        for elt in self.gps_obj_data:
            distance += self.compute_distance(gps_point, elt)
            zone = int(distance / self.zone_perimeter)

            if(str(zone + 1).zfill(3) in self.files_per_zone.keys()):
                self.files_per_zone[str(zone + 1).zfill(3)].append(elt["file"])
            else:
                self.files_per_zone.update(
                    {str(zone + 1).zfill(3): [elt["file"]]})

            gps_point = elt

        print("[\u2713]")

    def export_pcd_data(self):
        print("Converting & exporting point cloud (LiDAR) data to .pcd ... ",
              end="", flush=True)

        if(not os.path.exists(self.out_path)):
            os.mkdir(self.out_path)

        for zone, files in self.files_per_zone.items():
            os.mkdir(self.out_path + zone)

            for idx in range(len(files)):
                obj_file = open(self.in_path + files[idx])

                obj_data = []
                for line in obj_file:
                    if(line[0] == "v"):
                        obj_data.append([float(elt)
                                         for elt in line[2:].split()])

                pcd = open3d.geometry.PointCloud()
                pcd.points = open3d.utility.Vector3dVector(np.array(obj_data))

                open3d.io.write_point_cloud(
                    "{}/{}/{}.pcd".format(self.out_path, zone, str(idx + 1).zfill(4)), pcd)

        print("[\u2713]")