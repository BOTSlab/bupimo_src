from math import pi
from math import fmod

def constrain_angle_neg_pos_pi(angle):
    """ 
    Constrains the given angle to the range (-pi, pi].
    """

    if angle > 0:
        result = fmod(angle + pi, 2*pi) - pi
        if result == -pi:
            return pi
        return result
    elif angle < 0:
        return fmod(angle - pi, 2*pi) + pi
    else:
        return 0

def constrain_angle_pos_twopi(angle):
    """ 
    Constrains the given angle to the range (0, 2*pi].
    """

    neg_pos_angle = constrain_angle_neg_pos_pi(angle)

    if neg_pos_angle < 0:
        return neg_pos_angle + 2*pi
    else:
        return neg_pos_angle

def verify_neg_pos_pi(angle, ideal):
    result = constrain_angle_neg_pos_pi(angle)
    difference = abs(result - ideal)
    if difference < 1e-10:
        pass
        print "Passed: constrain_angle_neg_pos_pi({}) = {}".format(angle, ideal)
    else:
        pass
        print "Failed: constrain_angle_neg_pos_pi({}) != {}.  Got {} instead".format(\
              angle, ideal, result)
        pass
    pass

def verify(angle, ideal):
    verify_neg_pos_pi(angle, ideal)

# Testing
if __name__ == '__main__':
    # Corner cases with positive angles
    verify(0, 0)
    verify(pi, pi)
    verify(2*pi, 0)
    verify(3*pi, pi)
    verify(4*pi, 0)
    verify(5*pi, pi)

    # Corner cases with negative angles
    verify(-pi, pi)
    verify(-2*pi, 0)
    verify(-3*pi, pi)
    verify(-4*pi, 0)
    verify(-5*pi, pi)

    # Various positive angles
    verify(pi/4, pi/4)
    verify(pi/2, pi/2)
    verify(3*pi/4, 3*pi/4)
    verify(5*pi/4, -3*pi/4)
    verify(3*pi/2, -pi/2)
    verify(9*pi/4, pi/4)

    # Various negative angles
    verify(-pi/4, -pi/4)
    verify(-pi/2, -pi/2)
    verify(-3*pi/4, -3*pi/4)
    verify(-5*pi/4, 3*pi/4)
    verify(-3*pi/2, pi/2)
    verify(-9*pi/4, -pi/4)

    print "Done."
