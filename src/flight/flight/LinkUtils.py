def bit_test(flags, bit_num):
    mask = 1 << bit_num
    return (0 != flags & mask)

