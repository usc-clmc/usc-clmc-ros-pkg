#!/usr/bin/env python

frame='/base_footprint'
id=0
#num_shelves=4
num_shelves=3
num_dividers=2
start_x = 0.60
start_y = -0.70
start_z = 0.0
#height = 1.6
height = 1.286
depth = 0.4
width = 1.2
plank_thickness = 0.03
pose_x_offset = 0.0

def make_box(x, y, z, dx, dy, dz, yaw):
    global frame
    global id
    id = id + 1
    cx = x+dx/2.0
    cy = y+dy/2.0
    cz = z+dz/2.0
    print "  -"
    print "    name: box%d"%(id)
    print "    frame: %s"%(frame)
    print "    position: [ %f, %f, %f ]"%(cx, cy, cz)
    print "    orientation: [ 0.0, 0.0, 0.0 ]"
    print "    dimensions: [ %f, %f, %f ]"%(dx, dy, dz)

print "boxes:"

# create horizontal planks (num_shelves + 2)
shelf_height = (height - plank_thickness) / (num_shelves + 1)

for i in range(num_shelves+2):
    z = start_z + i*shelf_height
    make_box(start_x, start_y, z, depth, width, plank_thickness, 0.0)

# create vertical dividers
divider_width = (width - plank_thickness) / (num_dividers + 1)
for i in range(num_dividers+2):
    y = start_y + i*divider_width
    make_box(start_x, y, start_z, depth, plank_thickness, height, 0.0)

# create poses
print "poses_frame: %s" % (frame)
print "poses:"
for i in range(num_shelves+1):
    for j in range(num_dividers+1):
        x = start_x + pose_x_offset
        y = start_y + (divider_width * j) + (plank_thickness + divider_width)/2.0
        z = start_z + (shelf_height * i) + (plank_thickness + shelf_height)/2.0
        print "  - [ %f, %f, %f, %f, %f, %f ]" % (x, y, z, 0.0, 0.0, 0.0)
