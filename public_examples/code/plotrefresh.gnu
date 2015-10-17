set terminal x11

set polar
set angle degrees


set grid polar 20
set grid ls 10

set xrange[-2000:2000]
set yrange[0:2000]

set xtics axis
set ytics axis

set style line 2 lt 1 lw 3 pt 3 linecolor rgb "blue"
set style line 3 lt 1 lw 3 pt 3 linecolor rgb "red"
set style line 4 lt 1 lw 3 pt 3 linecolor rgb "yellow"

plot "objects.txt" using 1:2 title 'detected objects' w l ls 2, \
     "target.txt" using 1:2 title 'target objects' w l ls 3, \
     "ltarget.txt" using 1:2 title 'lost target prediction' w l ls 4


pause 0.1
reread