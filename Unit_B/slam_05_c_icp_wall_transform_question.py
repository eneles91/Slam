# Subsample the scan. For each point, find a closest point on the
# wall of the arena.
# From those point pairs, estimate a transform and apply this to the pose.
# Repeat the closest point - estimate transform loop.
# This is an ICP algorithm.
# 05_c_icp_wall_transform
# Claus Brenner, 17 NOV 2012
from lego_robot import *
from slam_b_library import filter_step, concatenate_transform,\
    compute_cartesian_coordinates, write_cylinders
from math import sqrt, atan2

# Given a point list, return the center of mass.
def compute_center(point_list):
    # Safeguard against empty list.
    if not point_list:
        return (0.0, 0.0)
    # If not empty, sum up and divide.
    sx = sum([p[0] for p in point_list])
    sy = sum([p[1] for p in point_list])
    return (float(sx) / len(point_list), float(sy) / len(point_list))


# Given a left_list of points and a right_list of points, compute
# the parameters of a similarity transform: scale, rotation, translation.
# If fix_scale is True, use the fixed scale of 1.0.
# The returned value is a tuple of:
# (scale, cos(angle), sin(angle), x_translation, y_translation)
# i.e., the rotation angle is not given in radians, but rather in terms
# of the cosine and sine.
def estimate_transform(left_list, right_list, fix_scale = False):
    # Compute left and right center.
    lc = compute_center(left_list)
    rc = compute_center(right_list)
    cs = 0.0
    ss = 0.0
    rr = 0.0
    ll = 0.0
    l_strich = []
    r_strich = []

    for i in range(len(left_list)):

        l_strich.append((left_list[i][0] - lc[0], left_list[i][1] - lc[1]))
        r_strich.append((right_list[i][0] - rc[0], right_list[i][1] - rc[1]))
        cs += r_strich[i][0] * l_strich[i][0] + r_strich[i][1] * l_strich[i][1]
        ss += -r_strich[i][0] * l_strich[i][1] + r_strich[i][1] * l_strich[i][0]
        rr += r_strich[i][0]**2 + r_strich[i][1]**2
        ll += l_strich[i][0]**2 + l_strich[i][1]**2

    if ll==0 or rr ==0:
        return None
    else:
        if fix_scale:
            la = 1.0
        else:
            la = sqrt(rr / ll)

    c = cs / (sqrt(cs**2 + ss**2))
    s = ss / (sqrt(cs**2 + ss**2))

    tx = rc[0] - la * (c * lc[0] - s * lc[1])
    ty = rc[1] - la * (s * lc[0] + c * lc[1])

    # --->>> Insert here your code to compute lambda, c, s and tx, ty.

    return la, c, s, tx, ty


# Given a similarity transformation:
# trafo = (scale, cos(angle), sin(angle), x_translation, y_translation)
# and a point p = (x, y), return the transformed point.
def apply_transform(trafo, p):
    la, c, s, tx, ty = trafo
    lac = la * c
    las = la * s
    x = lac * p[0] - las * p[1] + tx
    y = las * p[0] + lac * p[1] + ty
    return (x, y)


# Correct the pose = (x, y, heading) of the robot using the given
# similarity transform. Note this changes the position as well as
# the heading.
def correct_pose(pose, trafo):

    temp = apply_transform(trafo, (pose[0], pose[1]))
    #pose[0] = temp[0]
    #pose[1] = temp[1]
    #pose[2] = atan2(trafo[2]/trafo[1])


    pose = (temp[0], temp[1], pose[2] + atan2(trafo[2],trafo[1]))
    # --->>> This is what you'll have to implement.

    return (pose[0], pose[1], pose[2])  # Replace this by the corrected pose.


# Takes one scan and subsamples the measurements, so that every sampling'th
# point is taken. Returns a list of (x, y) points in the scanner's
# coordinate system.
def get_subsampled_points(scan, sampling = 10):
    # Subsample from scan
    index_range_tuples = []
    for i in xrange(0, len(scan), sampling):
        index_range_tuples.append( (i, scan[i]) )
    return compute_cartesian_coordinates(index_range_tuples, 0.0)


# Given a set of points, checks for every point p if it is closer than
# eps to the left, right, upper or lower wall of the arena. If so,
# adds the point to left_list, and the closest point on the wall to
# right_list.
def get_corresponding_points_on_wall(points,
                                     arena_left = 0.0, arena_right = 2000.0,
                                     arena_bottom = 0.0, arena_top = 2000.0,
                                     eps = 150.0):
    left_list = []
    right_list = []

    for p in points:
        x,y = p
        if x < (arena_left+eps) and x > (arena_left - eps):
            left_list.append(p)
            right_list.append( (arena_left,y) )
        elif x < (arena_right + eps) and x > (arena_right -eps):
            left_list.append(p)
            right_list.append( (arena_right,y) )
        elif y < arena_bottom + eps and y > arena_bottom - eps:
            left_list.append(p)
            right_list.append( (x,arena_bottom) )
        elif y < arena_top + eps and y > arena_top - eps:
            left_list.append(p)
            right_list.append( (x,arena_top) )
    # ---> Implement your code here.

    return left_list, right_list


# ICP: Iterate the steps of transforming the points, selecting point pairs, and
# estimating the transform. Returns the final transformation.
def get_icp_transform(world_points, iterations):

    # Iterate assignment and estimation of trafo a few times.

    # --->>> Implement your code here.
    overall_trafo = (1.0, 1.0, 0.0, 0.0, 0.0)

    for j in range(iterations):
        trafo_pts = [apply_transform(overall_trafo, p) for p in world_points]
        left, right = get_corresponding_points_on_wall(trafo_pts)
        trafo = estimate_transform(left, right, fix_scale=True)
        if trafo:
            overall_trafo = concatenate_transform(trafo, overall_trafo)
    # You may use the following strategy:
    # Start with the identity transform:
    #   overall_trafo = (1.0, 1.0, 0.0, 0.0, 0.0)
    # Then loop for j in xrange(iterations):
    #   Transform the world_points using the curent overall_trafo
    #     (see 05_b on how to do this)
    #   Call get_correspoinding_points_on_wall(...)
    #   Determine transformation which is needed "on top of" the current
    #     overall_trafo: trafo = estimate_transform(...)
    #   Concatenate the found transformation with the current overall_trafo
    #     to obtain a new, 'combined' transformation. You may use the function
    #     overall_trafo = concatenate_transform(trafo, overall_trafo)
    #     to concatenate two similarities.
    #   Note also that estimate_transform may return None.
    # 

    # Return the final transformation.
    return overall_trafo


if __name__ == '__main__':
    # The constants we used for the filter_step.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 150.0

    # The start pose we obtained miraculously.
    pose = (1850.0, 1897.0, 3.717551306747922)

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")

    # Iterate over all positions.
    out_file = file("icp_wall_transform.txt", "w")
    for i in xrange(len(logfile.scan_data)):
        # Compute the new pose.
        pose = filter_step(pose, logfile.motor_ticks[i],
                           ticks_to_mm, robot_width,
                           scanner_displacement)

        # Subsample points.
        subsampled_points = get_subsampled_points(logfile.scan_data[i])
        world_points = [LegoLogfile.scanner_to_world(pose, c)
                        for c in subsampled_points]

        # Get the transformation.
        # You may play withe the number of iterations here to see
        # the effect on the trajectory!
        trafo = get_icp_transform(world_points, iterations = 40)

        # Correct the initial position using trafo.
        pose = correct_pose(pose, trafo)

        # Write to file.
        # The pose.
        print >> out_file, "F %f %f %f" % pose
        # Write the scanner points and corresponding points.
        write_cylinders(out_file, "W C",
            [apply_transform(trafo, p) for p in world_points])

    out_file.close()
