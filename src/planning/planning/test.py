import numpy as np
import matplotlib.pyplot as plt

# Function to compute cubic Bézier curve for a given t
def cubic_bezier(t, P0, P1, P2, P3):
    return (1 - t)**3 * P0 + 3 * (1 - t)**2 * t * P1 + 3 * (1 - t) * t**2 * P2 + t**3 * P3

def smooth(points):
    points = np.array(points)
    number = len(points)
    smoothed_path = []

    P0 = points[0]
    P1 = points[int(number / 4)]
    P2 = points[int(3 * number / 4) - 1]
    P3 = points[-1]

    t_values = np.linspace(0, 1, 20)

    for t in t_values:
        smoothed_point = cubic_bezier(t, P0, P1, P2, P3)
        smoothed_path.append(tuple(smoothed_point))  # Convert the NumPy array to a tuple

    return smoothed_path

def extract_segments(path, resolution):
    horizontal_segments = []
    vertical_segments = []
    diagonal_segments = []
    
    i = 0
    while i < len(path) - 2:
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        x3, y3 = path[i + 2]

        if y1 == y2 == y3 and np.isclose(abs(x2 - x1), resolution) and np.isclose(abs(x3 - x2), resolution):
            start = i
            while i + 2 < len(path) and np.isclose(path[i + 1][1], path[i][1]) and \
                (np.isclose(path[i + 1][0], path[i][0] + resolution) or np.isclose(path[i + 1][0], path[i][0] - resolution)) and \
                (np.isclose(path[i + 2][0], path[i + 1][0] + resolution) or np.isclose(path[i + 2][0], path[i + 1][0] - resolution)):
                i += 1
            horizontal_segments.append((path[start:i + 2], i))

        elif x1 == x2 == x3 and np.isclose(abs(y2 - y1), resolution) and np.isclose(abs(y3 - y2), resolution):
            start = i
            while i + 2 < len(path) and np.isclose(path[i + 1][0], path[i][0]) and \
                (np.isclose(abs(path[i + 1][1] - path[i][1]), resolution) or np.isclose(abs(path[i + 1][1] - path[i][1]), -resolution)) and \
                (np.isclose(abs(path[i + 2][1] - path[i + 1][1]), resolution) or np.isclose(abs(path[i + 2][1] - path[i + 1][1]), -resolution)):
                i += 1
            vertical_segments.append((path[start:i + 2], i))

        elif (np.isclose(abs(x2 - x1), resolution) and np.isclose(abs(y2 - y1), 0) and np.isclose(abs(x3 - x2), 0) and np.isclose(abs(y3 - y2), resolution)) or \
             (np.isclose(abs(x2 - x1), 0) and np.isclose(abs(y2 - y1), resolution) and np.isclose(abs(x3 - x2), resolution) and np.isclose(abs(y3 - y2), 0)):        
            start = i
            while i + 2 < len(path) and (
                (np.isclose(abs(path[i + 1][0] - path[i][0]), resolution) and np.isclose(abs(path[i + 1][1] - path[i][1]), 0) and np.isclose(abs(path[i + 2][0] - path[i + 1][0]), 0) and np.isclose(abs(path[i + 2][1] - path[i + 1][1]), resolution)) or
                (np.isclose(abs(path[i + 1][0] - path[i][0]), 0) and np.isclose(abs(path[i + 1][1] - path[i][1]), resolution) and np.isclose(abs(path[i + 2][0] - path[i + 1][0]), resolution) and np.isclose(abs(path[i + 2][1] - path[i + 1][1]), 0))
            ):
                i += 1
            diagonal_segments.append((path[start:i + 2], i))

        i += 1

    return horizontal_segments, vertical_segments, diagonal_segments


# Function to combine adjacent segments
def combine_segments(sorted_segments):
    # Combine so every segment is longer than 8
    i = 0
    while i < len(sorted_segments) - 1:
        # Check if the current segment has less than 8 elements
        if len(sorted_segments[i]) < 8:
            # How many elements we need to reach 8
            needed = 8 - len(sorted_segments[i])
            # Extend the current segment with elements from the next one
            sorted_segments[i].extend(sorted_segments[i + 1][:needed])
            # Remove the elements we just took from the next segment
            sorted_segments[i + 1] = sorted_segments[i + 1][needed:]
            
            # If the next segment becomes empty, remove it
            if len(sorted_segments[i + 1]) == 0:
                sorted_segments.pop(i + 1)
        else:
            # Move to the next segment
            i += 1

    new_segments = []
    # Merging the joints of the segments
    for i in range(1, len(sorted_segments)):
        # Get the last two points of the previous segment
        prev_segment = sorted_segments[i - 1]
        next_segment = sorted_segments[i]
        
        # Take the last two points from the previous segment
        prev_end_points = prev_segment[-3:]

        # Take the first two points from the next segment
        next_start_points = next_segment[:3]

        # Create a new segment by combining these 4 points
        merge_segment = prev_end_points + next_start_points

        # Add the non-modified segments to the list, if the segment is long enough
        new_segments.append(prev_segment[2:-2])

        # Add the merge segment
        new_segments.append(merge_segment)

    # Add the last segment and update the first to include the start
    new_segments[0] = sorted_segments[0][:-2]
    if len(sorted_segments[-1]) > 3:
        new_segments.append(sorted_segments[-1][2:])
    return new_segments


def create_god_path(path, resolution):
    # Extract segments
    horizontal, vertical, diagonal = extract_segments(path, resolution)
    # print(f'Horizontal {horizontal}')
    # print(f'Vertical {vertical}')
    # print(f'Diagonal {diagonal}')

    # Combine segments
    combined_segments = diagonal + horizontal + vertical

    # Sort the segments by the first point of each segment
    combined_segments_sorted = sorted(combined_segments, key=lambda x: x[1])

    # Remove the sorting index (only keep the segments)
    sorted_segments = [segment[0] for segment in combined_segments_sorted]

    # # Output the new combined segments
    # print("New Combined Segments:")
    # for segment in sorted_segments:
    #     print(segment)

    if len(sorted_segments) > 2:
        # Combine adjacent segments by taking out the last two and first two points
        combined_segments = combine_segments(sorted_segments)

        # Output the new combined segments
        print("New Combined Segments after merge:")
        for segment in combined_segments:
            print(segment)
    else:
        combined_segments = sorted_segments

    # Now, smooth the segments and create the new path (as a list of tuples)
    smoothed_path = []
    for segment in combined_segments:
        smoothed_segment = smooth(segment)
        smoothed_path.extend(smoothed_segment)  # Add the smoothed points to the final list

    return smoothed_path



# Example path
path = [(0, 0), (0, 1), (1, 1), (1, 2), (2, 2), (2, 3), (3, 3), (3, 4),
        (4, 4), (4, 5), (5, 5), (5, 6), (6, 6), (6, 7), (7, 7), (7, 8), 
        (8, 8), (8, 9), (8, 10),(8, 11),(8, 12),(8, 13),(8, 14),(8, 15),
        (9, 15),(10,15),(11,15),(12,15),(13,15),(14,15),(15,15),(16,15),
        (16,14),(16,13),(16,12),(16,11),(16,10),(16, 9),(16, 8),(16, 7),
        (15,7), (14, 7),(13, 7),(12, 7),(11, 7),(10, 7),(9, 7), (8, 7),
        (8, 6), (8, 5), (8, 4), (8, 3), (7, 3), (7, 2), (6, 2), (6, 1)]

# path = [
#     (0, 0), (0, 1), (1, 1), (1, 2), (2, 2), (2, 3), (3, 3), (3, 4),
#     (4, 4), (5, 4), (5, 5), (6, 5), (6, 6), (7, 6), (7, 7), (8, 7),
#     (8, 8), (9, 8), (9, 9), (10, 9), (10, 10), (11, 10), (11, 9), (12, 9),
#     (12, 8), (13, 8), (13, 7), (14, 7), (14, 6), (15, 6), (15, 5), (16, 5),
#     (16, 4), (15, 4), (15, 3), (14, 3), (14, 2), (13, 2), (13, 1), (12, 1),
#     (12, 0), (11, 0), (10, 0), (9, 0), (9, 1), (8, 1), (8, 2), (7, 2),
#     (7, 3), (6, 3), (6, 4), (5, 4), (5, 3), (4, 3), (4, 2), (3, 2),
#     (3, 1), (2, 1), (2, 0), (1, 0), (0, 0)
# ]

# path = [(0, 0), (0, 1), (1, 1), (1, 2), (2, 2), (2, 3), (3, 3), (3, 4),
#         (4, 4), (4, 5), (5, 5), (5, 6), (6, 6), (6, 7), (7, 7), (7, 8)]



smoothed_path = create_god_path(path, resolution=1)
# print(smoothed_path)

# Convert path to NumPy array for plotting
path = np.array(path)


# Plot the original staircase path (connecting the points)
plt.plot(path[:, 0], path[:, 1], 'o-', label='Original Path (Staircase)', color='r')

# Plot the smoothed path
smoothed_path = np.array(smoothed_path)
plt.plot(smoothed_path[:, 0], smoothed_path[:, 1], label='Smoothed Path', color='b')

# Add title and labels
plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Original vs Smoothed Sorted Path')

# Show the plot
plt.show()





        # # Check each direction explicitly
        # if np.isclose(x2 - x1, resolution) and np.isclose(y2 - y1, 0) and np.isclose(x3 - x2, 0) and np.isclose(y3 - y2, resolution):  # Right and up
        #     start = i
        #     while i + 2 < len(path) and (
        #         np.isclose(path[i + 1][0] - path[i][0], resolution) and np.isclose(path[i + 1][1] - path[i][1], 0) and
        #         np.isclose(path[i + 2][0] - path[i + 1][0], 0) and np.isclose(path[i + 2][1] - path[i + 1][1], resolution)
        #     ):
        #         i += 1
        #     diagonal_segments.append((path[start:i + 2], i))

        # elif np.isclose(x2 - x1, 0) and np.isclose(y2 - y1, resolution) and np.isclose(x3 - x2, resolution) and np.isclose(y3 - y2, 0):  # Up and right
        #     start = i
        #     while i + 2 < len(path) and (
        #         np.isclose(path[i + 1][0] - path[i][0], 0) and np.isclose(path[i + 1][1] - path[i][1], resolution) and
        #         np.isclose(path[i + 2][0] - path[i + 1][0], resolution) and np.isclose(path[i + 2][1] - path[i + 1][1], 0)
        #     ):
        #         i += 1
        #     diagonal_segments.append((path[start:i + 2], i))

        # elif np.isclose(x2 - x1, -resolution) and np.isclose(y2 - y1, 0) and np.isclose(x3 - x2, 0) and np.isclose(y3 - y2, -resolution):  # Left and down
        #     start = i
        #     while i + 2 < len(path) and (
        #         np.isclose(path[i + 1][0] - path[i][0], -resolution) and np.isclose(path[i + 1][1] - path[i][1], 0) and
        #         np.isclose(path[i + 2][0] - path[i + 1][0], 0) and np.isclose(path[i + 2][1] - path[i + 1][1], -resolution)
        #     ):
        #         i += 1
        #     diagonal_segments.append((path[start:i + 2], i))

        # elif np.isclose(x2 - x1, 0) and np.isclose(y2 - y1, -resolution) and np.isclose(x3 - x2, -resolution) and np.isclose(y3 - y2, 0):  # Down and left
        #     start = i
        #     while i + 2 < len(path) and (
        #         np.isclose(path[i + 1][0] - path[i][0], 0) and np.isclose(path[i + 1][1] - path[i][1], -resolution) and
        #         np.isclose(path[i + 2][0] - path[i + 1][0], -resolution) and np.isclose(path[i + 2][1] - path[i + 1][1], 0)
        #     ):
        #         i += 1
        #     diagonal_segments.append((path[start:i + 2], i))

        # elif np.isclose(x2 - x1, resolution) and np.isclose(y2 - y1, 0) and np.isclose(x3 - x2, -resolution) and np.isclose(y3 - y2, 0):  # Right and left (move right and then left)
        #     start = i
        #     while i + 2 < len(path) and (
        #         np.isclose(path[i + 1][0] - path[i][0], resolution) and np.isclose(path[i + 1][1] - path[i][1], 0) and
        #         np.isclose(path[i + 2][0] - path[i + 1][0], -resolution) and np.isclose(path[i + 2][1] - path[i + 1][1], 0)
        #     ):
        #         i += 1
        #     diagonal_segments.append((path[start:i + 2], i))

        # elif np.isclose(x2 - x1, 0) and np.isclose(y2 - y1, resolution) and np.isclose(x3 - x2, 0) and np.isclose(y3 - y2, -resolution):  # Up and down (move up then down)
        #     start = i
        #     while i + 2 < len(path) and (
        #         np.isclose(path[i + 1][0] - path[i][0], 0) and np.isclose(path[i + 1][1] - path[i][1], resolution) and
        #         np.isclose(path[i + 2][0] - path[i + 1][0], 0) and np.isclose(path[i + 2][1] - path[i + 1][1], -resolution)
        #     ):
        #         i += 1
        #     diagonal_segments.append((path[start:i + 2], i))

        # elif np.isclose(x2 - x1, resolution) and np.isclose(y2 - y1, -resolution) and np.isclose(x3 - x2, 0) and np.isclose(y3 - y2, 0):  # Right and down (move right and then down)
        #     start = i
        #     while i + 2 < len(path) and (
        #         np.isclose(path[i + 1][0] - path[i][0], resolution) and np.isclose(path[i + 1][1] - path[i][1], -resolution) and
        #         np.isclose(path[i + 2][0] - path[i + 1][0], 0) and np.isclose(path[i + 2][1] - path[i + 1][1], 0)
        #     ):
        #         i += 1
        #     diagonal_segments.append((path[start:i + 2], i))