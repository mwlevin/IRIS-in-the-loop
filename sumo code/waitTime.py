import numpy as np
from scipy.integrate import trapezoid


def waitTimes(demand_list,passage_list):
    # Convert to arrays
    demand_list = np.array(demand_list)
    passage_list = np.array(passage_list)
    
    # Get avg TT
    diff_list = demand_list-passage_list
    integral = trapezoid(diff_list[0:len(demand_list)], dx=30)  # Assuming the time step is 1, change 'dx' if it's different
    t_avg = integral/passage_list[-1]
    
    # Get Max TT
    
    def find_horizontal_intersection(y, points):
        """ Return the x coordinate at which the straight line segments linking a series
            of points first intersect the horizontal line y=y

            Returns None if there is no intersection.
        """
        for (x1, y1), (x2, y2) in zip(points, points[1:]):
            if y1 == y2:   # if the segment is itself horizontal
                if y == y1:
                    return x1
            elif y1 <= y <= y2 or y2 <= y <= y1: # Check the horizontal intersects this segment at all
                return (x2 - x1) * (y - y1) / (y2 - y1) + x1
            
    b = list(enumerate(passage_list))  # b is a list of coords [(0, 3), (1, 0), (2, 1)...]
    max_horizontal_distance = 0

    for x, y in enumerate(demand_list):
        bx = find_horizontal_intersection(y, b[x:])
        if bx is not None:
            distance = bx - x
            if distance > max_horizontal_distance:
                max_horizontal_distance = distance
                
    maxWait = max_horizontal_distance * 30
    
    return(t_avg,maxWait)


