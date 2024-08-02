#!/usr/bin/env python3
import json
import numpy as np

def get_map_data(map_data_path, move_origin=True, origin_offset=5):
    """
    Load the map data from the json file and package it into a dictionary

    Args:
        map_data_path (str): The path to the map data json file
        move_origin (bool, optional): If True, move the origin to the min x and y values. Defaults to True.
        origin_offset (int, optional): The offset to apply to the origin. Defaults to 5.

    Returns:
        dict: A dictionary containing the map data
    """

    # Load the tree data dictionary
    with open(map_data_path, 'rb') as f:
        map_data = json.load(f)

    all_class_estimates = []
    all_position_estimates = []
    all_width_estimates = []
    object_numbers = []
    test_tree_numbers = []
    test_tree_indexes = []

    # Extract the classes and positions from the tree data dictionary
    for object_data in map_data:

        # Save test trees as a different class 
        if object_data['test_tree']:
            test_tree_numbers.append(int(object_data['test_tree_number']))
            test_tree_indexes.append(1)
        else:
            test_tree_indexes.append(0)

        all_class_estimates.append(object_data['class_estimate'])
        # Move the tree positions by the gps adjustment
        #TODO: Fix GPS adjustment in map
        original_position_estimate = np.array(object_data['position_estimate'])
        gps_adjustment = np.array(object_data['gps_adjustment'])
        position_estimate = original_position_estimate
        all_position_estimates.append(position_estimate.tolist())
        all_width_estimates.append(object_data['width_estimate'])
        object_numbers.append(object_data['object_number'])

    object_numbers = np.array(object_numbers, dtype=int)
    all_class_estimates = np.array(all_class_estimates, dtype=int)
    all_position_estimates = np.array(all_position_estimates)
    all_width_estimates = np.array(all_width_estimates)
    test_tree_numbers = test_tree_numbers
    test_tree_indexes = np.array(test_tree_indexes, dtype=bool)

    if move_origin:
        # Find the min x and y values, subtract 5 and set that as the origin
        x_min = np.min(all_position_estimates[:, 0]) - origin_offset
        y_min = np.min(all_position_estimates[:, 1]) - origin_offset

        # Subtract origin from positions
        all_position_estimates[:, 0] -= x_min
        all_position_estimates[:, 1] -= y_min

    map_data = {'all_class_estimates': all_class_estimates,
                'all_position_estimates': all_position_estimates,
                'all_width_estimates': all_width_estimates,
                'object_numbers': object_numbers,
                'test_tree_numbers': test_tree_numbers,
                'test_tree_indexes': test_tree_indexes,}

    return map_data
