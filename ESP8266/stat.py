import pstats
p = pstats.Stats('profile.txt')
p.strip_dirs().sort_stats('cumulative').print_stats(10)