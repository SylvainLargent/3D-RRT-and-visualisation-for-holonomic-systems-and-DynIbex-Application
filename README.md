# RRT-and-visualisation-for-drone-DynIbex-Application

Basic RRT and RRT* alogrithms for 3D space and holonomic systems

Visualisation with Matplot3D, plot_trajectory.py (Lots of options with the booleans) (NEEDS PYTHON3.8)

A scientific article accompanying the code is available in the Paper repository

The files "main.cpp" are the "created" files !

And there is another file for the post treatment by Dynibex.

WARNING ! PLEASE CHANGE ALL THE PATHS FOR THE PLOTS AND CSV FILES WRITING SO IT FITS YOUR COMPUTER PATHS

WARNING ! If you want to use the files using DynIbex, it is required to have DynIbex installed 
Check if the function export3d_yn exists in ibex_simulation.h of the library, if else please add the following code in ibex_simulation.h:
    /**export in a file for ploting*/  
    void export3d_yn(const char* filename, int x, int y, int z)  
    {  
      assert(nb_var > 2);  
      std::cout << "export in progress..." << std::endl;  
      if(!list_solution_g.empty())  
	{  
	  std::ofstream file(filename, std::ios::out | std::ios::trunc);  
	  std::list<solution_g>::iterator iterator_list;  
	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)  
	    {  
	      file << iterator_list->box_jnh->operator[](x) <<  
		" ; " << iterator_list->box_jnh->operator[](y) <<  
		" ; " << iterator_list->box_jnh->operator[](z) << std::endl;  
	    }  


	  file.close();  
	}  

When installing DynIbex, move all the library in a binary form inside the repo /BIN
	./waf configure --without-lp --prefix=../BIN

Import the library DynIbex for your compiler, PLEASE CHANGE THE "your_path" part accordingly
	export PKG_CONFIG_PATH=your_path/BIN/share/pkgconfig/
