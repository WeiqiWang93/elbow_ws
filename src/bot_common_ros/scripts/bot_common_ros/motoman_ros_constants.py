''' '''
test_test_1 = 0x03 # front, up, no flip, S<180, R<180, T<180
test_test_2 = 0.5 # each pose coordinate must be within this delta to satisfy motion_complete

MOVEJ_DEFAULT_VEL = 0.087   # 5 deg/s
MOVEJ_MAX_VEL_DEG = 30.0  # 30.0 deg/s
MIN_DURATION = 2.0  # 5.0 sec