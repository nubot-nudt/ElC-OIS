import os
import argparse
import numpy as np
import time
from tqdm import tqdm
from pc_cluster.ellipsoidal_clustering.build import ellipsoidalClustering
import open3d as o3d

parser = argparse.ArgumentParser()
parser.add_argument('--dataset_dir', dest="dataset_dir", required=True,
                    help="The path of kitti sequences. Like /dataset/kitti/dataset/sequences")
parser.add_argument('--minimum_points', dest="minimum_points", default=1,
                    help="minimum_points to be considered as an instance")
parser.add_argument('--target_set', dest="target_set", default='val', help="process val set or test set;")
parser.add_argument('--panoptic_segmentation_result', dest="panoptic_segmentation_result", default='DS-net',
                    help="use the panoptic segmentation from 'DS-net' or 'Panoptic-PolarNet'")
parser.add_argument('--max_diffusing_times', dest="max_diffusing_times", default=50,
                    help="maximum times for diffuse searching")
parser.add_argument('--use_refinement', action='store_true', help="whether implement refinement")

args = parser.parse_args()

# The numbers corresponding to the semantic classes are referred in the file:
# https://github.com/PRBonn/semantic-kitti-api/blob/master/config/semantic-kitti.yaml

# Known instances: car, truck, bicycle, motorcycle, other-vehicle, person, bicyclist, motorcyclist
# Background: road, parking, sidewalk, other-ground, lane-marking, terrain, vegetation

dsnet_inv_label_dict_exclude = {0: 0, 10: 1, 11: 2, 15: 3, 18: 4, 20: 5, 30: 6, 31: 7, 32: 8, 50: 13, 51: 14, 71: 16,
                                80: 18, 81: 19, 40: 100, 44: 101, 48: 102, 49: 103, 60: 107, 70: 108, 72: 110}

polarnet_inv_label_dict_exclude = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8, 13: 13, 14: 14, 16: 16,
                                   18: 18, 19: 19, 9: 100, 10: 101, 11: 102, 12: 103, 17: 107, 15: 108, 72: 110}

# Input parameters for ellipsoidal neighbors (rho, theta, phi)
ElC = ellipsoidalClustering.ellipsoidalClustering(2.0, 2.0, 7.5)

if args.target_set == 'val':
    sequences_list = ['08']
elif args.target_set == 'test':
    sequences_list = np.arange(11, 22)
else:
    raise RuntimeError("The target_set for processing is not correct, please input 'val' or 'test'.")

for sequence_No in sequences_list:
    print(f"Processing sequence {sequence_No}.")
    velodyne_dir_path = os.path.join(args.dataset_dir, str(sequence_No), "velodyne/")
    velodyne_lists = os.listdir(velodyne_dir_path)
    velodyne_lists.sort()

    # Path of close-set panoptic segmentation results
    label_path = os.path.join("closeset_PS_labels", str(args.panoptic_segmentation_result), str(sequence_No), "predictions/")

    # Path to save OIS results
    save_path = os.path.join("output/sequences/", str(sequence_No), "predictions/")
    if not os.path.isdir(save_path):
        os.makedirs(save_path)

    for velodyne_frame in tqdm(velodyne_lists, leave=True):
        velodyne_file_path = velodyne_dir_path + velodyne_frame
        raw_scan = np.fromfile(velodyne_file_path, dtype=np.float32).reshape((-1, 4))
        velodyne_points = raw_scan[:, 0:3]

        label_file_path = label_path + os.path.splitext(velodyne_frame)[0] + ".label"

        # Get the official close-set panoptic segmentation result
        panoptic_labels = np.fromfile(label_file_path, dtype=np.uint32).reshape((-1))
        semantic_labels = panoptic_labels & 0xFFFF
        instance_ori_IDs = panoptic_labels >> 16

        if args.panoptic_segmentation_result == "DS-net":
            semantic_labels_inv = [dsnet_inv_label_dict_exclude[mm] for mm in semantic_labels]
        elif args.panoptic_segmentation_result == "Panoptic-PolarNet":
            semantic_labels_inv = [polarnet_inv_label_dict_exclude[mm] for mm in semantic_labels]
        else:
            raise ValueError(f"No panoptic segmentation results from model {args.panoptic_segmentation_result}.")
        semantic_labels_inv_arr = np.array(semantic_labels_inv)

        # Exclude points belonging to the background and the known instances
        sem_mask = np.logical_or(semantic_labels_inv_arr < 100, semantic_labels_inv_arr > 110)
        ins_mask = np.where(instance_ori_IDs == 0, True, False)
        final_mask = sem_mask * ins_mask

        velodyne_points_filtered = velodyne_points[final_mask]

        points_x = velodyne_points_filtered[:, 0]
        points_y = velodyne_points_filtered[:, 1]
        points_z = velodyne_points_filtered[:, 2]

        ''' Ellipsoidal clustering '''
        ElC_starting_time = time.time()
        ElC_result = ElC.ellipsoidalClustering_main(points_x, points_y, points_z,
                                          velodyne_points_filtered.shape[0])
        ElC_ending_time = time.time()
        ElC_time = ElC_ending_time - ElC_starting_time
        print(f"\n{ElC_time} seconds for frame {velodyne_frame} by ElC.")

        # Instance IDs of each unknown point are stored in 64*2048-shape ndarray
        ElC_result_arr = np.array(ElC_result)

        unknown_instance_labels = np.zeros((velodyne_points_filtered.shape[0]))
        known_ID_offset = np.amax(instance_ori_IDs)
        for ElC_instance_ID in np.unique(ElC_result_arr):
            if ElC_instance_ID == 0:
                continue
            ElC_instance_indices = np.array(np.where(ElC_result_arr == ElC_instance_ID))
            unknown_instance_labels[ElC_instance_indices[0, :] * 2048 + ElC_instance_indices[1, :]] = ElC_instance_ID

        # Merge the IDs of known and unknown instances
        instance_ori_IDs[final_mask] = unknown_instance_labels + known_ID_offset

        ''' Refinement for raw known instances '''
        if args.use_refinement:
            refinement_starting_time = time.time()
            print("1")

            # Get point clouds and instance IDs of raw known instances
            known_ins_mask = np.logical_and(semantic_labels_inv_arr > 0, semantic_labels_inv_arr < 9)
            known_velodyne_points_filtered = velodyne_points[known_ins_mask]
            known_ins_IDs = instance_ori_IDs[known_ins_mask]
            known_ins_IDs_copy = known_ins_IDs.copy()
            print("2")

            # Construct the KD-tree for point clouds of raw known instances
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(known_velodyne_points_filtered)
            pcd_tree = o3d.geometry.KDTreeFlann(pcd)

            known_ins_ID_numbers = np.unique(known_ins_IDs)

            # Refine all raw known instances
            while known_ins_ID_numbers.size != 0:
                known_ins_ID_number = known_ins_ID_numbers[0]
                known_ins_ID_numbers = np.delete(known_ins_ID_numbers, 0)

                if known_ins_ID_number == 0:
                    continue

                diffusing_times = 0
                points_indices_for_refinement = []
                ins_IDs_for_refinement = []

                ins_IDs_for_refinement = np.append(ins_IDs_for_refinement, known_ins_ID_number).astype(int)

                indices_of_ins_ID_arr = np.array(np.where(known_ins_IDs == known_ins_ID_number))
                if indices_of_ins_ID_arr.shape[1] == 1:
                    indices_of_ins_ID = indices_of_ins_ID_arr
                else:
                    indices_of_ins_ID = np.squeeze(indices_of_ins_ID_arr)
                points_indices_for_refinement = np.append(points_indices_for_refinement, indices_of_ins_ID).astype(int)

                while indices_of_ins_ID.size != 0:
                    indice_of_ins_ID = indices_of_ins_ID[0]
                    indices_of_ins_ID = np.delete(indices_of_ins_ID, 0)

                    # Search for the surrounding instances
                    [k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[indice_of_ins_ID], 0.5)
                    surrounding_ins_IDs = np.unique(known_ins_IDs[idx])

                    is_new_ins_flag = ~np.isin(surrounding_ins_IDs, ins_IDs_for_refinement) * \
                                      np.isin(surrounding_ins_IDs, known_ins_ID_numbers)

                    if True not in is_new_ins_flag:
                        continue

                    ins_IDs_for_refinement = np.append(ins_IDs_for_refinement, surrounding_ins_IDs[is_new_ins_flag])

                    # Add the surrounding instances for refinement and further searching
                    for surrounding_new_ins_ID in surrounding_ins_IDs[is_new_ins_flag]:
                        indices_to_append = np.squeeze(np.array(np.where(known_ins_IDs == surrounding_new_ins_ID)))
                        indices_of_ins_ID = np.append(indices_of_ins_ID, indices_to_append).astype(int)
                        points_indices_for_refinement = np.append(points_indices_for_refinement, indices_to_append).astype(int)
                        known_ins_ID_numbers = np.delete(known_ins_ID_numbers,
                                                         np.where(known_ins_ID_numbers == surrounding_new_ins_ID))
                        diffusing_times += 1
                    if diffusing_times >= args.max_diffusing_times:
                        break

                points_for_refinement = known_velodyne_points_filtered[points_indices_for_refinement]
                if points_for_refinement.shape[0] < 10 or diffusing_times < 2:
                    continue

                # refine the surrounding instances
                points_x = points_for_refinement[:, 0]
                points_y = points_for_refinement[:, 1]
                points_z = points_for_refinement[:, 2]
                refinement_result = ElC.ellipsoidalClustering_main(points_x, points_y, points_z, points_for_refinement.shape[0])
                refinement_result_arr = np.array(refinement_result)

                refined_labels = np.zeros((points_for_refinement.shape[0]))
                refined_known_ID_offset = np.amax(known_ins_IDs_copy)
                for refined_ID in np.unique(refinement_result_arr):
                    if refined_ID == 0:
                        continue
                    indices_of_refined_ins_ID = np.where(refinement_result_arr == refined_ID)
                    refined_ins_indices_arr = np.array(indices_of_refined_ins_ID)
                    refined_labels[refined_ins_indices_arr[0, :]*2048 + refined_ins_indices_arr[1, :]] = refined_ID

                refined_labels[np.where(refined_labels == 0)] = -refined_known_ID_offset
                known_ins_IDs_copy[points_indices_for_refinement] = refined_labels + refined_known_ID_offset

            # Combine the refinement known instances and unknown instances
            complete_ins_offset = np.amax(instance_ori_IDs)
            known_ins_IDs_copy = np.where(known_ins_IDs_copy == 0, 0, known_ins_IDs_copy + complete_ins_offset)
            instance_ori_IDs[known_ins_mask] = known_ins_IDs_copy
            refinement_ending_time = time.time()
            refinement_time = refinement_ending_time - refinement_starting_time
            print(f"\n{refinement_time} seconds for frame {velodyne_frame} by refinement.")

        # Ignore instances with LiDAR hits less than the threshold
        for final_label in np.unique(instance_ori_IDs):
            if final_label == 0:
                continue
            label_indexes = np.array(np.where(instance_ori_IDs == final_label))
            if label_indexes.shape[1] >= args.minimum_points:
                continue
            instance_ori_IDs[label_indexes] = 0

        labels = semantic_labels.reshape(-1, 1) + ((instance_ori_IDs.astype(np.uint32) << 16) & 0xFFFF0000).reshape(-1, 1)
        save_file_path = save_path + os.path.splitext(velodyne_frame)[0] + ".label"
        labels.tofile(save_file_path)
