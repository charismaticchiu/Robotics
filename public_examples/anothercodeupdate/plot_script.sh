#!/bin/bash

cd /home/joel/Documents/CSCI5551_project/examples/scan_data

gnuplot <<EOF
 
set term png size 1600,800
set polar
set angle degrees

set grid polar 20
set grid ls 10

set xrange[-2000:2000]
set yrange[0:2000]

set xtics axis
set ytics axis

set output 'range.png'
set style line 2 lt 1 lw 3 pt 3 linecolor rgb "blue"
plot "most_recent_scan.txt" using 1:2 title 'range' with lines, \
     "detected_objs.txt" using 1:2 title 'detected objects' w l ls 2, \
     "obj_vels.txt" using 1:2:3:4 with vectors filled head lw 3

set xrange[-10:10]
set yrange[0:10]

set output 'log_range.png'

plot "most_recent_scan.txt" using 1:4 title 'log range'  with lines, \
     "detected_objs.txt" using 1:3 title 'detected objects' w l ls 2, \


unset polar
unset grid

set xrange[0:180]
set yrange[-1000:1000]



set output 'd_range.png'
plot "most_recent_scan.txt" using 1:3 title 'd_range' with lines


set xrange[0:180]
set yrange[-5:5]


set output 'd_log_range.png'
plot "most_recent_scan.txt" using 1:5 title 'd log range' with lines

quit

EOF

eog range.png &