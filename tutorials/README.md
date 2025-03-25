bccw
-------
LD_LIBRARY_PATH=/usr/local/lib ./8_complete_flow



9_CGAL_Intersects
-------
# 正确编译命令（无需 -lCGAL）
g++ -std=c++17 9_CGAL_Intersects.cpp -o myprogram \
   -I/usr/include -L/usr/lib/aarch64-linux-gnu \
   -lboost_system -lboost_thread -lgmp -lmpfr


10_CGAL_Intersects
-------
g++ -o 10_CGAL_Intersects myprogram.cpp -lClipper2
