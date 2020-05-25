# -*- coding: UTF-8 -*-

from PointCloudData import *

if __name__ == "__main__":

    print("-- Start --\n")

    # input arguments
    in_path = "data/Target 03/Original/2020-02-27/"
    out_path = "data/Target 03/Processed/2020-02-27_a/"

    path_start = {"latitude": "46.77654", "longitude": "-71.27184"}
    path_end = {"latitude": "46.813508", "longitude": "-71.2053822"}
    zone_perimeter = 100 # in meters

    # processing
    pointCloudData = PointCloudData(in_path, path_start, zone_perimeter, path_end, out_path)
    # importing the localization (gps) data associated with datetime 
    pointCloudData.import_gps_data()
    # importing the point cloud (LiDAR) data associated with datetime 
    pointCloudData.import_obj_data()
    # generate the point cloud (LiDAR) data with localization (gps) data
    pointCloudData.compute_gps_obj_data()
    # divide the route path into zones 
    pointCloudData.compute_files_per_zone()
    # convert and save the point cloud (LiDAR) data in .pcd
    pointCloudData.export_pcd_data()

    print("\n-- End --")