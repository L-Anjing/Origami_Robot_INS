"""Keep estimator reset/correction discontinuities out of motion polylines."""


def stamp_ns(stamp):
    return stamp.sec*10**9 + stamp.nanosec


def split_trajectory(poses, break_stamps_ns):
    breaks = iter(break_stamps_ns)
    boundary = next(breaks, None)
    segments, current = [], []
    for pose in poses:
        timestamp = stamp_ns(pose.header.stamp)
        split = False
        while boundary is not None and boundary <= timestamp:
            split = True
            boundary = next(breaks, None)
        if split and current:
            segments.append(current)
            current = []
        current.append(pose)
    if current:
        segments.append(current)
    return segments
