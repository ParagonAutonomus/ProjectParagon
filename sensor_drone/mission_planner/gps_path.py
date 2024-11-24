import math

'''
Index 0: bottom-left, index 1: bottom-right, index 2: top-right, index 3: top-left
Something we will want to change in the future is a waypoint is added at the very edge of the map even if the closest 
waypoint is a centimeter off. This is to account for the area being unevenly divided into segments to ensure large swaths of land wont be
unaccounted for. 

Points list: Row 0: bottom-left, row 1: bottom-right, row 2: top-right, row 3: top-left
Column one: latitude, column 2: longitude
'''
def makepath(points, split_size_lat_miles, split_size_lng_miles):
    # Convert miles to degrees for latitude and longitude
    split_size_lat = split_size_lat_miles * (1 / 69)  # Approx. 1 degree latitude ≈ 69 miles
    mid_lat = (points[2][0] + points[0][0]) / 2  # Midpoint latitude for longitude calculation
    degrees_per_mile_lng = 1 / (69 * math.cos(math.radians(mid_lat)))
    split_size_lng = split_size_lng_miles * degrees_per_mile_lng

    # Calculate dimensions in degrees
    lat_range = points[2][0] - points[0][0]
    lng_range = points[1][1] - points[0][1]

    # Number of sections per dimension
    num_sections_lat = int(lat_range / split_size_lat)
    num_sections_lng = int(lng_range / split_size_lng)

    # Generate grid points
    p_queue = []
    for i in range(num_sections_lat + 1):
        for j in range(num_sections_lng + 1):
            # Calculate the latitude and longitude
            lat = i * split_size_lat + points[0][0]
            lng = j * split_size_lng + points[0][1]

            # Ensure the points are within the rectangle bounds
            if lat > points[2][0] or lng > points[2][1]:
                continue

            p_queue.append((lat, lng))

            # if the waypoints don't reach the highest latitude on the map add waypoints that do
            if p_queue[-1][0] != points[2][0]:
                for k in range(num_sections_lng + 1): #adding biggest latitude coordinate
                    lat = points[2][0]
                    lng = k * split_size_lat + points[0][1]
                    p_queue.append((lat, lng))
            
            # if the waypoints don't reach the highest longitude on the map add waypoints that do
            if p_queue[-1][1] != points[2][1]:
                for k in range(num_sections_lat + 1): 
                    lat = k * split_size_lat + points[0][0]
                    lng = points[2][1]
                    p_queue.append((lat, lng))

    p_queue = sort_queue(p_queue)

    return p_queue

#sort the list in incrementing latitude values and alternating increasing and decreasing longitude values
def sort_queue(p_queue):
     # Sort points primarily by lat, secondarily by lng
    p_queue.sort(key=lambda point: (point[0], point[1]))
    
    # Group points by lat-value
    from itertools import groupby
    grouped = [list(group) for _, group in groupby(p_queue, key=lambda point: point[0])]
    
    # Apply zig-zag sorting
    sorted_zigzag = []
    for index, group in enumerate(grouped):
        if index % 2 == 0:
            # Even group: Keep ascending y
            sorted_zigzag.extend(group)
        else:
            # Odd group: Reverse y
            sorted_zigzag.extend(group[::-1])
    
    return sorted_zigzag


