from sys import *

f = open('cluster_bounding_box_finder_3d.py', 'r')
f_out = open('output.py', 'w+')
for line in f:
	f_out.write(line.strip(' ')[6:])
f.close()
f_out.close()
print 'Finished.'