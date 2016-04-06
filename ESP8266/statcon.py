import pstats
p = pstats.Stats('console/profile.txt')
p.strip_dirs().sort_stats('cumulative').print_stats('unitmap.py')