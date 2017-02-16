import geometry

def find_intersection(p0, p1, p2, p3 ) :
    s10_x = p1[0] - p0[0]
    s10_y = p1[1] - p0[1]
    s32_x = p3[0] - p2[0]
    s32_y = p3[1] - p2[1]
    denom = s10_x * s32_y - s32_x * s10_y
    if denom == 0 : return None # collinear
    denom_is_positive = denom > 0
    s02_x = p0[0] - p2[0]
    s02_y = p0[1] - p2[1]
    s_numer = s10_x * s02_y - s10_y * s02_x
    if (s_numer < 0) == denom_is_positive : return None # no collision
    t_numer = s32_x * s02_y - s32_y * s02_x
    if (t_numer < 0) == denom_is_positive : return None # no collision
    if (s_numer > denom) == denom_is_positive or (t_numer > denom) == denom_is_positive : return None # no collision
    # collision detected
    t = float(t_numer) / denom
    #print(p0[0], p0[1], s10_x, s10_y, denom, s_numer, t_numer, t)
    #intersection_point = ( int(p0[0] + (t * s10_x)), int(p0[1] + (t * s10_y)) )
    intersection_point = ( p0[0] + (t * s10_x), p0[1] + (t * s10_y) )
    #note: sin(angle) = denom/(|s10|*|s32|)
    #where angle is a falling angle, 90 is ortho
    #or we can use (sin(angle))^2 = denom^2/(|s10|^2*|s32|^2) to avoid sqrt
    #we basically need to compare denom^2 and (|s10|^2*|s32|^2)
    # this can be used for reflection modelling, after field tests
    return intersection_point

print geometry.c_find_intersection((10,20), (30,20), (20, 10), (20, 30) )
print find_intersection((10,20), (30,20), (20, 10), (20, 30) )

print geometry.c_find_intersection((10,10), (30,30), (30, 10), (10, 30))
print find_intersection((10,10), (30,30), (30, 10), (10, 30))
