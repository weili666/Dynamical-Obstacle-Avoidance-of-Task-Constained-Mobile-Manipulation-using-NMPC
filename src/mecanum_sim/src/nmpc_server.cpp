/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */



 /**
 *    \file   examples/simulation_environment/simple_mpc.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */

#include "ros/ros.h"
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <mecanum_sim/NMPCStep.h>
#include <iostream>

using namespace std;

USING_NAMESPACE_ACADO

bool nmpcPlan(mecanum_sim::NMPCStep::Request &req,
              mecanum_sim::NMPCStep::Response &res)
{
    DifferentialState x1;
    DifferentialState x2;
    DifferentialState x3;
    DifferentialState x4;

    cout<<"hello world 1"<<endl;

    Control w1;
    Control w2;
    Control w3;
    Control w4;
    Control w5;
    Control w6;
    Control w7;
    Control w8;
    Control w9;

    cout<<"hello world 2"<<endl;

    Parameter T;
    double j11, j12, j13, j14, j15, j16, j17, j18, j19;
    double j21, j22, j23, j24, j25, j26, j27, j28, j29;
    double j31, j32, j33, j34, j35, j36, j37, j38, j39;
    j11 = req.jacobian[0]; j12 = req.jacobian[1]; j13 = req.jacobian[2]; j14 = req.jacobian[3]; j15 = req.jacobian[4]; j16 = req.jacobian[5]; j17 = req.jacobian[6]; j18 = req.jacobian[7]; j19 = req.jacobian[8];
    j21 = req.jacobian[9]; j22 = req.jacobian[10]; j23 = req.jacobian[11]; j24 = req.jacobian[12]; j25 = req.jacobian[13]; j26 = req.jacobian[14]; j27 = req.jacobian[15]; j28 = req.jacobian[16]; j29 = req.jacobian[17];
    j31 = req.jacobian[18]; j32 = req.jacobian[19]; j33 = req.jacobian[20]; j34 = req.jacobian[21]; j35 = req.jacobian[22]; j36 = req.jacobian[23]; j37 = req.jacobian[24]; j38 = req.jacobian[25]; j39 = req.jacobian[26];

    cout<<"hello world 3"<<endl;

    DifferentialEquation f;

    cout<<"hello world 4"<<endl;

    f << dot(x1) == -x1*x1*((x2*j11+x3*j21+x4*j31)*w1+(x2*j12+x3*j22+x4*j32)*w2+(x2*j13+x3*j23+x4*j33)*w3+(x2*j14+x3*j24+x4*j34)*w4+(x2*j15+x3*j25+x4*j35)*w5+(x2*j16+x3*j26+x4*j36)*w6+(x2*j17+x3*j27+x4*j37)*w7+(x2*j18+x3*j28+x4*j38)*w8+(x2*j19+x3*j29+x4*j39)*w9);
    f << dot(x2) == x1*(j11*w1+j12*w2+j13*w3+j14*w4+j15*w5+j16*w6+j17*w7+j18*w8+j19*w9)-x1*x2*((x2*j11+x3*j21+x4*j31)*w1+(x2*j12+x3*j22+x4*j32)*w2+(x2*j13+x3*j23+x4*j33)*w3+(x2*j14+x3*j24+x4*j34)*w4+(x2*j15+x3*j25+x4*j35)*w5+(x2*j16+x3*j26+x4*j36)*w6+(x2*j17+x3*j27+x4*j37)*w7+(x2*j18+x3*j28+x4*j38)*w8+(x2*j19+x3*j29+x4*j39)*w9);
    f << dot(x3) == x1*(j21*w1+j22*w2+j23*w3+j24*w4+j25*w5+j26*w6+j27*w7+j28*w8+j29*w9)-x1*x3*((x2*j11+x3*j21+x4*j31)*w1+(x2*j12+x3*j22+x4*j32)*w2+(x2*j13+x3*j23+x4*j33)*w3+(x2*j14+x3*j24+x4*j34)*w4+(x2*j15+x3*j25+x4*j35)*w5+(x2*j16+x3*j26+x4*j36)*w6+(x2*j17+x3*j27+x4*j37)*w7+(x2*j18+x3*j28+x4*j38)*w8+(x2*j19+x3*j29+x4*j39)*w9);
    f << dot(x4) == x1*(j31*w1+j32*w2+j33*w3+j34*w4+j35*w5+j36*w6+j37*w7+j38*w8+j39*w9)-x1*x4*((x2*j11+x3*j21+x4*j31)*w1+(x2*j12+x3*j22+x4*j32)*w2+(x2*j13+x3*j23+x4*j33)*w3+(x2*j14+x3*j24+x4*j34)*w4+(x2*j15+x3*j25+x4*j35)*w5+(x2*j16+x3*j26+x4*j36)*w6+(x2*j17+x3*j27+x4*j37)*w7+(x2*j18+x3*j28+x4*j38)*w8+(x2*j19+x3*j29+x4*j39)*w9);

    cout<<"hello world 5"<<endl;

    Function h;

    cout<<"hello world 6"<<endl;

    h << x1;
    h << x2;
    h << x3;
    h << x4;
    h << w1;
    h << w2;
    h << w3;
    h << w4;
    h << w5;
    h << w6;
    h << w7;
    h << w8;
    h << w9;

    cout<<"hello world 7"<<endl;

    DMatrix Q(13, 13);
    Q.setIdentity();
    Q(0, 0) = 10.0;

    cout<<"hello world 8"<<endl;

    DVector r(13);
    r.setAll(0.0);

    cout<<"hello world 9"<<endl;

    const double t_start = 0.0;
    const double t_end = 1.0;

    OCP ocp( t_start, t_end, 20 );

    cout<<"hello world 10"<<endl;

    ocp.minimizeLSQ( Q, h, r );

    cout<<"hello world 11"<<endl;

    ocp.subjectTo( f );

    cout<<"hello world 12"<<endl;

//    ocp.subjectTo( AT_START, x1 ==  req.xstates[0] );
//    ocp.subjectTo( AT_START, x2 ==  req.xstates[1] );
//    ocp.subjectTo( AT_START, x3 ==  req.xstates[2] );
//    ocp.subjectTo( AT_START, x4 ==  req.xstates[3] );

    ocp.subjectTo( -0.5 <= w1 <=  0.5   );
    ocp.subjectTo( -0.5 <= w2 <=  0.5   );
    ocp.subjectTo( -0.5 <= w3 <=  0.5   );
    ocp.subjectTo( -0.5 <= w4 <=  0.5   );
    ocp.subjectTo( -0.5 <= w5 <=  0.5   );
    ocp.subjectTo( -0.5 <= w6 <=  0.5   );
    ocp.subjectTo( -0.5 <= w7 <=  0.5   );
    ocp.subjectTo( -0.2 <= w8 <=  0.2   );
    ocp.subjectTo( -0.2 <= w9 <=  0.2   );

    cout<<"hello world 13"<<endl;

    // SETTING UP THE (SIMULATED) PROCESS:
       // -----------------------------------
    OutputFcn identity;

    cout<<"hello world 14"<<endl;

    DynamicSystem dynamicSystem( f,identity );

    cout<<"hello world 15"<<endl;

    Process process( dynamicSystem,INT_RK45 );

    cout<<"hello world 16"<<endl;

    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------
    RealTimeAlgorithm alg( ocp,0.05 );
    alg.set( MAX_NUM_ITERATIONS, 2 );

    cout<<"hello world 17"<<endl;
    StaticReferenceTrajectory zeroReference;

    Controller* controller = new Controller( alg,zeroReference );
    cout<<"hello world 18"<<endl;



    // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
       // ----------------------------------------------------------
    SimulationEnvironment* sim = new SimulationEnvironment( 0.0,1.0,process,*controller );
    cout<<"hello world 19"<<endl;

    DVector x0(4);
    x0(0) = req.xstates[0];
    x0(1) = req.xstates[1];
    x0(2) = req.xstates[2];
    x0(3) = req.xstates[3];
    cout<<"x0(0):"<<req.xstates[0]<<"  x0(1):"<<req.xstates[1]<<"  x0(2):"<<req.xstates[2]<<"  x0(3):"<<req.xstates[3]<<endl;
    cout<<"hello world 20"<<endl;

    sim->init( x0 );

    cout<<"hello world 21"<<endl;


    sim->run( );

//    if (sim.init( x0 ) != SUCCESSFUL_RETURN)
//        exit( EXIT_FAILURE );
//    if (sim.run( ) != SUCCESSFUL_RETURN)
//        exit( EXIT_FAILURE );

    cout<<"hello world 22"<<endl;

    // ...AND PLOT THE RESULTS
          // ----------------------------------------------------------
    VariablesGrid sampledProcessOutput;
    sim->getSampledProcessOutput( sampledProcessOutput );

    VariablesGrid feedbackControl;
    sim->getFeedbackControl( feedbackControl );


    // DEFINE A PLOT WINDOW:
    // ---------------------
//    GnuplotWindow window;
//        window.addSubplot( sampledProcessOutput(0),"DifferentialState x1" );
//        window.addSubplot( sampledProcessOutput(1),"DifferentialState x2" );
//        window.addSubplot( sampledProcessOutput(2),"DifferentialState x3" );
//        window.addSubplot( sampledProcessOutput(3),"DifferentialState x4" );
//        window.addSubplot( feedbackControl(0),"DifferentialState w1" );
//        window.addSubplot( feedbackControl(1),"DifferentialState w2" );
//        window.addSubplot( feedbackControl(2),"DifferentialState w3" );
//        window.addSubplot( feedbackControl(3),"DifferentialState w4" );
//        window.addSubplot( feedbackControl(4),"DifferentialState w5" );
//        window.addSubplot( feedbackControl(5),"DifferentialState w6" );
//        window.addSubplot( feedbackControl(6),"DifferentialState w7" );
//        window.addSubplot( feedbackControl(7),"DifferentialState w8" );
//        window.addSubplot( feedbackControl(8),"DifferentialState w9" );
//        window.plot( );

    int num_steps = sim->getNumSteps();
    cout<<"num_steps :"<<num_steps<<endl;
    double w1_all, w2_all, w3_all, w4_all, w5_all, w6_all, w7_all, w8_all, w9_all;
    for(int i = 0; i < num_steps/2; i ++)
    {
         w1_all += feedbackControl(0,i);
         w2_all += feedbackControl(1,i);
         w3_all += feedbackControl(2,i);
         w4_all += feedbackControl(3,i);
         w5_all += feedbackControl(4,i);
         w6_all += feedbackControl(5,i);
         w7_all += feedbackControl(6,i);
         w8_all += feedbackControl(7,i);
         w9_all += feedbackControl(8,i);
    }
    double w1_avg, w2_avg, w3_avg, w4_avg, w5_avg, w6_avg, w7_avg, w8_avg, w9_avg;
    cout<<"jacobian(1, 1):"<<j11<<endl;
    w1_avg = w1_all/(num_steps/2);
    cout<<"w1_avg:"<<w1_avg<<endl;
    w2_avg = w2_all/(num_steps/2);
    cout<<"w2_avg:"<<w2_avg<<endl;
    w3_avg = w3_all/(num_steps/2);
    cout<<"w3_avg:"<<w3_avg<<endl;
    w4_avg = w4_all/(num_steps/2);
    cout<<"w4_avg:"<<w4_avg<<endl;
    w5_avg = w5_all/(num_steps/2);
    cout<<"w5_avg:"<<w5_avg<<endl;
    w6_avg = w6_all/(num_steps/2);
    cout<<"w6_avg:"<<w6_avg<<endl;
    w7_avg = w7_all/(num_steps/2);
    cout<<"w7_avg:"<<w7_avg<<endl;
    w8_avg = w8_all/(num_steps/2);
    cout<<"w8_avg:"<<w8_avg<<endl;
    w9_avg = w9_all/(num_steps/2);
    cout<<"w9_avg:"<<w9_avg<<endl;


    res.omega1 = w1_avg;res.omega2 = w2_avg;res.omega3 = w3_avg;res.omega4 = w4_avg;res.omega5 = w5_avg;
    res.omega6 = w6_avg;res.omega7 = w7_avg;res.omega8 = w8_avg;res.omega9 = w9_avg;
    cout<<"res.omegas:"<<res.omega1<<", "<<res.omega2<<", "<<res.omega3<<", "<<res.omega4<<", "<<res.omega5<<", "<<res.omega6<<", "
       <<res.omega7<<", "<<res.omega8<<", "<<res.omega9<<", "<<endl;

    delete sim;
    delete controller;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nmpc_plan_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("nmpc_plan", nmpcPlan);
    ROS_INFO("Ready to plan the input omega use Nonlinear Model Predictive Control");
    ros::spin();
    return 0;
}
