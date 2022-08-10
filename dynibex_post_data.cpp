#include <iostream>
#include <cmath>
#include "Point3.hpp"
#include "Segment.hpp"
#include "Env.hpp"
#include "RRT_STAR.hpp"
#include "Node.hpp"
#include <fstream>
#include "ibex.h"
#include <cstring>  
#include <string>
#include <stdlib.h>

using namespace ibex;
using namespace std;

int main(int argc, char ** argv){
    //Voliere de l'U2IS
    Env environment = Env(-1.5, 1.1, -1.4, 1.8, 0.0, 2, 0.25, 0.25, 0.25);

//Defining the obstacles of the scene
    vector<vector<double>> list_rectangles =
        {   
            {-1.0, -1.25, -0.1, 0.25, 0.7, 0.6}, // Boite proche du mur
            {-1.1, 0.71, -0.1, 0.4, 0.6, 0.5},  //Boite opérateur
            // {-0.6, 0.7, -0.1, 1.25, 1, 3},    //Opérateur
            // {-1.65, -0.2, -0.1, 1.2, 0.3, 3}      //Obstacle virtuel
            {-1.15, -0.4, -0.1, 1.15, 0.8, 3}      //Obstacle virtuel
        };

    //Adding rectangles to the scene
    for(int i = 0; i < list_rectangles.size(); ++i){
        environment.add_rectangle(list_rectangles[i]);
    }

//Plotting rectangle obstacles thanks to their vertices
    vector<vector<Node*>> all_boundaries = environment.get_obstacles_vertex();  
    ofstream all_boundaries_stream;
    all_boundaries_stream.open("csv_files/obstacles.txt");
    if(!all_boundaries.empty()){
        for(int i = 0; i < all_boundaries.size(); ++i){
            for(int j = 0; j < all_boundaries[i].size();++j){
                all_boundaries_stream << (*(all_boundaries[i][j])) << std::endl;
                delete(all_boundaries[i][j]);
            }
        }
    }
    all_boundaries_stream.close();

    //Obstacles for dynibex
    vector<vector<double>>rectangles_intervals = environment.get_rectangles_boundaries(); //Inflated obstacles boundaries
    
    vector<IntervalVector> obstacles_itv;
    for(int k = 0; k < rectangles_intervals.size(); ++k){
        double boundaries_itv[3][2] = {{rectangles_intervals[k][0],rectangles_intervals[k][1]},
                                        {rectangles_intervals[k][2],rectangles_intervals[k][3]},
                                            {rectangles_intervals[k][4],rectangles_intervals[k][5]}};
        IntervalVector temp = IntervalVector(3, boundaries_itv);
        obstacles_itv.push_back(temp);
    }

    //Exporting dynibex simulation
    ofstream export_3d;
    export_3d.open("export_3d.txt");

    //Exporting trajectory realised
    ofstream trajectory_boxes;
    trajectory_boxes.open("trajectory_boxes.txt");

    //Opening the command with all the necessary information
    ifstream command;
    command.open("csv_files/command.txt");

    string line;

    vector<vector<double>> list_of_information;

    if (command.is_open())
    {
        while ( getline (command,line) )
        {
            vector<double> temp;
            string s ="";
            for (auto x : line)
            {
                if (x == ','){
                    temp.push_back(std::stod(s));
                    s = "";
                }
                else {
                    s = s+x;
                }
            }
            temp.push_back(std::stod(s));
            list_of_information.push_back(temp);
        }
        //cout << list_of_information << endl;
        command.close();
    }

    int rate = atoi(argv[1]); //100 : horizon time of around a second
    const int n= 3; 
    Variable y(n);
    Affine2Vector state(n);

    IntervalVector yinit(n);

    int cpt_sub=0;
    int cpt_inter=0;
    int index = 0;
    int number_of_simulations = 0;
    bool collision = false;
    while(index < list_of_information.size()-1){
            std::cerr << index << "/" << list_of_information.size() - 1 << endl <<  std::flush;
            std::cerr << std::flush;

            //defaut sur la mesure Optitrack
            double deltax = 0.1;
            double deltay = 0.1;
            double deltaz = 0.1;

            yinit[0] = Interval(list_of_information[index][3]) + Interval(-deltax, deltax);
            yinit[1] = Interval(list_of_information[index][4]) + Interval(-deltay, deltay);
            yinit[2] = Interval(list_of_information[index][5]) + Interval(-deltaz, deltaz);
        

            //Paramètres 
            Interval Kp(1.8);
            Interval Ki(3.2);

            double x_goal = list_of_information[index+1][3];
            double y_goal = list_of_information[index+1][4];
            double z_goal = list_of_information[index+1][5];

            Interval xGoal(x_goal);
            Interval yGoal(y_goal);
            Interval zGoal(z_goal);
            IntervalVector destination_itvs = IntervalVector(6);
            destination_itvs[0] = xGoal + Interval(-0.03, 0.03);
            destination_itvs[1] = yGoal + Interval(-0.03, 0.03);
            destination_itvs[2] = zGoal + Interval(-0.03, 0.03);
            

            if(index%rate == 0 || index+rate > list_of_information.size()-1){
                int index_time_horizon = (index+rate <= list_of_information.size()-1) ? index+rate : list_of_information.size()-1;     
                for(int j = number_of_simulations*rate; j < index_time_horizon; j++){
                    //commande uX,uY,uZ
                    Interval uX = Interval(list_of_information[j][0]);
                    Interval uY = Interval(list_of_information[j][1]);
                    Interval uZ = Interval(list_of_information[j][2]);

                    Interval alphaX = Interval(0,1);
                    Interval alphaY = Interval(-0.4,0.4);
                    Interval alphaZ = Interval(-0.4,0.4);

                    // Interval betaX = Interval(-atof(argv[2]),0.2);
                    // Interval betaY = Interval(-atof(argv[2]),atof(argv[2]));
                    // Interval betaZ = Interval(-atof(argv[2]),atof(argv[2]));

                    Interval betaX = Interval(-0.1,0.1);
                    Interval betaY = Interval(-0.1,0.1);
                    Interval betaZ = Interval(0,0.1);

                    Function ydot = Function(y,Return( 
                                        uX*alphaX + betaX,
                                        uY*alphaY + betaY,
                                        uZ*alphaZ + betaZ)
                                    );

                    ivp_ode problem = ivp_ode(ydot,list_of_information[j][10],yinit); //Modèle dynamique, temps de départ, yinit

                    simulation simu = simulation(&problem,list_of_information[j+1][10],HEUN,1e-2,1e-3);

                    simu.run_simulation();
                    yinit = simu.get_last();

                    //inclsion ? intersection non vide ?
                    IntervalVector test(3);
                    test[0] = Interval(list_of_information[j+1][3] - deltax, list_of_information[j+1][3] + deltax);
                    test[1] = Interval(list_of_information[j+1][4] - deltay, list_of_information[j+1][4] + deltay);
                    test[2] = Interval(list_of_information[j+1][5] - deltaz, list_of_information[j+1][5] + deltaz);


                    if (test.is_subset(yinit))
                        cpt_sub++;

                    if (!(test&yinit).is_empty())
                            cpt_inter++;

            //Verification no obstacles touched, and destination reached
                    for(int k = 0; k < obstacles_itv.size(); ++k){
                        if( !(yinit&obstacles_itv[k]).is_empty() ){
                            collision = true;
                        }
                    }
                    
                    // std::cout << "Step Destination reached ? " << (simu.finished_in(destination_itvs) == false)  << std::endl;
                    
            //For the last run which is shortened
                    if(index+rate > list_of_information.size()-1){
                        index = list_of_information.size(); //exits the while loop 
                        std::cerr << j << "/" << list_of_information.size() - 1 << endl <<  std::flush;
                        std::cerr << std::flush;


                        //Dernier trace de la trajectoire
                        double x_goal = list_of_information[j+1][3];
                        double y_goal = list_of_information[j+1][4];
                        double z_goal = list_of_information[j+1][5];

                        Interval xGoal(x_goal);
                        Interval yGoal(y_goal);
                        Interval zGoal(z_goal);
                        IntervalVector destination_itvs = IntervalVector(6);
                        destination_itvs[0] = xGoal + Interval(-0.03, 0.03);
                        destination_itvs[1] = yGoal + Interval(-0.03, 0.03);
                        destination_itvs[2] = zGoal + Interval(-0.03, 0.03);
                        
                        trajectory_boxes<< destination_itvs[0] <<
                        " ; " << destination_itvs[1] <<
                        " ; " << destination_itvs[2] << std::endl;
                    }
                    //Write CSV for the 3D boxes for the plot
                    export_3d << yinit[0] <<
                    " ; " << yinit[1] <<
                    " ; " << yinit[2] << std::endl;
            
                }//end for, sur l'horizon de temps
                number_of_simulations+=1;
        }//end if simulation


        //Write CSV for the 3D boxes representinf the trajectory
            trajectory_boxes<< destination_itvs[0] <<
            " ; " << destination_itvs[1] <<
            " ; " << destination_itvs[2] << std::endl;

         ++index;
        
    }//end while

    std::cout << "Objectives done accordingly ? (No collision ?) " << !collision  << std::endl;

    std::cout << "Intersection : " << cpt_inter << endl;
    std::cout << "Inclusion : " << cpt_sub << endl;
    export_3d.close(); 

    return 0;

}
