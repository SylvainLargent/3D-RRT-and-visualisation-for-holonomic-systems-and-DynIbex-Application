# RRT-and-visualisation-for-drone-DynIbex-Application

Basic RRT and RRT* alogrithms for 3D space and holonomic systems

Visualisation with Matplot3D, plot_trajectory.py (Lots of options with the booleans) (NEEDS PYTHON3.8)

A scientific article accompanying the code is available in the Paper repository

The files "main.cpp" are the "created" files !

And there is another file for the post treatment by Dynibex.

WARNING ! If you want to use the files using DynIbex, it is required to have DynIbex installed 

When installing DynIbex, move all the library in a binary form inside the repo /BIN
	./waf configure --without-lp --prefix=../BIN

Import the library DynIbex for your compiler, PLEASE CHANGE THE "your_path" part accordingly
	export PKG_CONFIG_PATH=your_path/BIN/share/pkgconfig/
