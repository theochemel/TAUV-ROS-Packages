from geometry_msgs.msg import Pose, Vector3, Quaternion, Point, Twist


# to list
def tl(o):
    if isinstance(o, Vector3):
        return [o.x, o.y, o.z]
    if isinstance(o, Point):
        return [o.x, o.y, o.z]
    if isinstance(o, Quaternion):
        return [o.x, o.y, o.z, o.w]
    if isinstance(o, list):
        return o
    raise ValueError("Unsupported type for tl! Add it in tauv_util/types.py")


# to msg
def tm(l, t):
    if t == Point:
        return Point(l[0], l[1], l[2])
    if t == Vector3:
        return Vector3(l[0], l[1], l[2])
    if t == Quaternion:
        return Quaternion(l[0], l[1], l[2], l[3])
    raise ValueError("Unsupported type for tm! Add it in tauv_util/types.py")
