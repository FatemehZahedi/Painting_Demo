
#include <sys/time.h>
#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <string.h> // strstr
#include "PositionControlClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <time.h>
#include <sys/shm.h>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace KUKA::FRI;
using namespace Eigen;

#define DEFAULT_PORTID 30200
#define DEFAULT_IP "192.170.10.2"  //ip address set for KONI connection
#define SHM_SIZE 1024

FILE *NewDataFile(void);


int *MakeFloatSharedMemory(int HowBig);
float *MakeFloatSharedMemory2(int HowBig2);


// Main
int main(int argc, char** argv)
{
	//******************---------------------------------------
	//---------------------------------------
	// May be changed-------------
	double elapsed = 0;
	double vel_depend = 0;
	double velocity_2 = 0;
	double w2 = 0;
	double velocity_old = 0;
	double velocity_new = 0;
	double velocity_mean = 0;
	double error_delta = 0;
	double checkval = 0;
	double mag = 0.01;
	//double dur = 1000;
	double dur = 5000;
	int trig_stat = 0;
	int outerdelay = 1;
	int delay_gate = 0;
	int met = 0;
	double ftx;
	double fty;
	double ftx_un;
	double fty_un;
	double zerox = 0;
	double zeroy = 0;
	double x_disp;
	double y_disp;
	double ftx_0 = 0.0;
	double fty_0 = 0.0;
	int *data;	//pointer to shared memory
	data = MakeFloatSharedMemory(2);
	float *data2; //pointer to shared memory
	data2 = MakeFloatSharedMemory2(2);
	int c = 0;
	int cc = 0;
	double al = 0.0005;
	double al2 = 0.1;
	int firstIt = 0;
	int trigger = 1;
	double fty_freeze;
	double T06_freeze_x;
	double T06_freeze_y;
	double T06_freeze_z;
	double phi_euler_freeze;
	double theta_euler_freeze;
	double psi_euler_freeze;
	int w = 0;
	int random_num;
	int random_num2;
	int ww = 0;
	double ft[2];
	int steady = 0;
	double FK_x = 0;
	double FK_x0 = 0;
	double gate1 = 0.355;
	double gate2 = -0.1;
	int count3 = 0;
	double delta_t = 0.001;
	double stupid = 1000.0;
	int perturb_flag = 0;
	int rr = 1;
	int count_ran1 = 0;
	int count_ran2 = 0;
	int count_ran3 = 0;
	int count_ran4 = 0;
	int count_ran_null = 0;
	int exit_ran = 0;
	int perturb_count = 0;
	int j1 = 0;
	int i1 = 0;
	int i2 = dur - 1;
	int sketch = 1;

	double phi_euler = 0;
	double theta_euler = 0;
	double psi_euler = 0;
	//-----------------------------------------------
	//********************-----------------------------------------------



	// DH Parameter----------------------------------------------------
	MatrixXd alpha(1, 7); alpha << M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2, M_PI / 2, -M_PI / 2, 0;
	MatrixXd a(1, 7); a << 0, 0, 0, 0, 0, 0, 0;
	MatrixXd d(1, 7); d << 0.36, 0, 0.42, 0, 0.4, 0, 0.126;
	MatrixXd theta(1, 7); theta << 0, 0, 0, 0, 0, 0, 0;



	//******************---------------------------------------
	//---------------------------------------
	// May be changed-------------
	MatrixXd x_0(6, 1); x_0 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_00(6, 1); x_00 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_incr(6, 1); x_incr << 0, 0, 0, 0, 0, 0;
	MatrixXd x_incr_0(6, 1); x_incr_0 << 0, 0, -0.1, 0, 0, 0;
	MatrixXd test_new(6, 1); test_new << 0, 0, 0, 0, 0, 0;


	MatrixXd qc(6, 1); qc << 0, 0, 0, 0, 0, 0;
	MatrixXd delta_q(6, 1); delta_q << 0, 0, 0, 0, 0, 0;
	MatrixXd q_freeze(6, 1); q_freeze << 0, 0, 0, 0, 0, 0;

	MatrixXd q_0(6, 1); q_0 << 0, 0, 0, 0, 0, 0;
	MatrixXd t_0(6, 1); t_0 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_e(6, 1); x_e << 0, 0, 0, 0, 0, 0;
	MatrixXd force(6, 1); force << 0, 0, 0, 0, 0, 0;
	MatrixXd q_new(6, 1); q_new << 0, 0, 0, 0, 0, 0;
	MatrixXd q_delay(6, 1); q_delay << 0, 0, 0, 0, 0, 0;
	MatrixXd x_new(6, 1); x_new << 0, 0, 0, 0, 0, 0;
	MatrixXd v(6, 1); v << 0, 0, 0, 0, 0, 0;
	MatrixXd q_old(6, 1); q_old << 0, 0, 0, 0, 0, 0;
	MatrixXd test0(6, 6); test0 << 1, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0,
		0, 0, 0, 2, 0, 0,
		0, 0, 0, 0, 2, 0,
		0, 0, 0, 0, 0, 2;
	MatrixXd randnum(12,1); randnum << 1, 2, 1, 3, 4, 2, 2, 4, 4, 3, 1, 3;
	//-------------------------------------------------
	//*****************************--------------------------



	//******************---------------------------------
	// I don't know what it is
	//filter
	int sample = 200;
	int sample2 = 100;
	MatrixXd tdata(6, sample); tdata.setZero(6, sample);
	//***********************-----------------------------


	//*********************-----------------------------------
	// First angles of Kuka
	// not sure about the usage
	MatrixXd qdata(6, sample); qdata.setZero(6, sample);//Initializing joint angles

	/*for (int i = 0; i<sample; i++)
	{
		qdata(0, i) = -1.779862; //UPRIGHT
		qdata(1, i) = 0.821814;
		qdata(2, i) = -0.067855;
		qdata(3, i) = 1.302481;
		qdata(4, i) = 0.284275;
		qdata(5, i) = -1.118251;
	}*/
	//*********************-------------------------------------

	//************************----------------------------
	// May be changed
	MatrixXd t_sum(6, 1); t_sum.setZero(6, 1);
	MatrixXd q_sum(6, 1); q_sum.setZero(6, 1);
	MatrixXd t_ave(6, 1); t_ave << 0, 0, 0, 0, 0, 0;
	MatrixXd q_ave(6, 1); q_ave << 0, 0, 0, 0, 0, 0;
	MatrixXd torques_0(6, 1); torques_0 << 0, 0, 0, 0, 0, 0;
	MatrixXd t_e(6, 1); t_e << 0, 0, 0, 0, 0, 0;
	//***********************-----------------------------

	//**********************-----------------------------
	// Not sure what it is
	struct timespec start2, finish2;
	struct timeval start, end;
	long mtime, seconds, useconds;
	gettimeofday(&start, NULL);
	//*********************------------------------------

	// parse command line arguments
	if (argc < 1)
	{
		printf(
			"\nKUKA LBR Position Control application\n\n"
			"\tCommand line arguments:\n"
			"\t1) filename of trajectory file"
			"\t2) remote hostname (optional)\n"
			"\t3) port ID (optional)\n"
		);
		return 1;
	}

	char* filename = argv[1]; //Requiered command line argument for the file name for trajectory file
	const char* hostname = (argc >= 3) ? argv[2] : DEFAULT_IP; //optional command line argument for ip address (default is for KONI)
	int port = (argc >= 4) ? atoi(argv[3]) : DEFAULT_PORTID; //optional comand line argument for port
	char path[4096];

	//FILE *InputFile; // Probably should be commented
	FILE *OutputFile;
	FILE *fp;

	bool success = true;
	bool success2 = true;

	int count = 0;
	int count2 = 0;
	int enough = 0;
	int rows = 0;
	int i = 0;
	int j = 0;


	int stime = 0;
	float sampletime = 0; //will be determined from FRI
	double MJoint[7] = { 0 };//{-1.776415,1.025433,-0.064599,1.656986,0.290444,-0.971846,-0.223775}; //last measured joint position (not sure time when controller measured, be careful using this for feedback loop)
	double ETorque[7] = { 0 }; //External torque :supposedly the torque applied to the robot (subtracts the torque due to the robot, ackording to Kuka)

	double MaxRadPerSec[7] = { 1.7104,1.7104,1.7453,2.2689,2.4435,3.14159,3.14159 }; //absolute max velocity (no load from KUKA manual for iiwa 800)
																					 //double MaxRadPerSec[7]={1.0,1.0,1.0,1.0,1.0,1.0,1.0}; //more conservative velocity limit
	double MaxRadPerStep[7] = { 0 };//will be calculated
	double MaxJointLimitRad[7] = { 2.9671,2.0944,2.9671,2.0944,2.9671,2.0944,3.0543 };//Max joint limits in radians (can be changed to further restrict motion of robot)
	double MinJointLimitRad[7] = { -2.9671,-2.0944,-2.9671,-2.0944,-2.9671,-2.0944,-3.0543 }; //Min joint limits in radians (can be changed to further restrict motion of robot)
	double FirstPositionDelta[7] = { 0.0175,0.0175,0.0175,0.0175,0.0175,0.0175,0.0175 }; //maximum deviation from initial position in trajectory from start position in robot(radians)


	//Get value for the time step of the command cycle (used for determining max velocity)
	// sampletime=client.GetTimeStep();
	sampletime = 0.001;
	fprintf(stdout, "Sample Time:%f seconds\n", sampletime);

	//calculate max step value
	for (i = 0; i<7; i++)
	{
		MaxRadPerStep[i] = sampletime*MaxRadPerSec[i]; //converts angular velocity to discrete time step
	}


	// create new joint position client
	PositionControlClient client;
	client.intvalues(MaxRadPerStep, MaxJointLimitRad, MinJointLimitRad);




	// create new udp connection
	UdpConnection connection;


	// pass connection and client to a new FRI client application
	ClientApplication app(connection, client);

	// connect client application to KUKA Sunrise controller
	app.connect(port, hostname);




	//create file for output
	OutputFile = NewDataFile();
	
	//open trajectory file
	//InputFile = fopen(filename, "r"); 
/*
	//get the first value in the file. This should be the number of rows(or the number of joint commands)
	success2 = fscanf(InputFile, "%d", &rows);
	//cout<<rows<<endl;
	if (rows<4)
	{
		fprintf(stdout, "Number of Rows is %d\n Are you sure the file is in correct format?\n First number should be number of rows.  \nThen each row is 7 joint values in radians.\n", rows);
		return 1;
	} // probably should be commented
	if (!success2)
	{
		fprintf(stdout, "Error in reading file!\n");
		return 1;
	}


	//read in first 7 joint positions (these should be in radians)
	success2 = fscanf(InputFile, "%lf %lf %lf %lf %lf %lf %lf", &client.NextJoint[0], &client.NextJoint[1], &client.NextJoint[2], &client.NextJoint[3], &client.NextJoint[4], &client.NextJoint[5], &client.NextJoint[6]);
	memcpy(client.LastJoint, client.NextJoint, 7 * sizeof(double));
	
	if (!success2)
	{
	fprintf(stdout,"Error in reading file!\n");
	return 1;
	}	 // Probably should be commented
*/
	//*****************************---------------------------------------------------------------
	//memcpy(MJoint, client.GetMeasJoint(), sizeof(double) * 7);
	
	/*client.NextJoint[0] = -1.570796;
	client.NextJoint[1] = 1.570796;
	client.NextJoint[2] = -1.570796;
	client.NextJoint[3] = 0.523599;
	client.NextJoint[4] = 1.570796;
	client.NextJoint[5] = -1.570796;
	client.NextJoint[6] = 1.0472;*/
	
	client.NextJoint[0] = -1.779862;
	client.NextJoint[1] = 0.821814;
	client.NextJoint[2] = -0.067855;
	client.NextJoint[3] = 1.302481;
	client.NextJoint[4] = 0.284275;
	//client.NextJoint[5] = -1.118251;
	client.NextJoint[5] = 1.9241;
	client.NextJoint[6] = -0.958709;
	
	
	memcpy(client.LastJoint, client.NextJoint, 7 * sizeof(double));

	//*******************************---------------------------------------
	// I don't understand them
	while (!enough)
	{
		clock_gettime(CLOCK_MONOTONIC, &start2);
		//---------------------------timestamp----------------------------------
		gettimeofday(&end, NULL);
		seconds = end.tv_sec - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		mtime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;
		//----------------------------------------------------------------------


		success = app.step();//step through program

		if (client.KukaState == 4)
		{
			count++; //count initialized at 0
			if (count == 1)//first time inside
			{
				sampletime = client.GetTimeStep();
				//calculate max step value
				for (i = 0; i < 7; i++)
				{
					client.MaxRadPerStep[i] = sampletime*MaxRadPerSec[i]; //converts angular velocity to discrete time step
				}

			}
			//*********************-------------------------------------------------------------


			memcpy(MJoint, client.GetMeasJoint(), sizeof(double) * 7); //gets the most recent measured joint value
			memcpy(ETorque, client.GetExtTor(), sizeof(double) * 7);//gets the external torques at the robot joints (supposedly subtracts the torques caused by the robot)

		// Forward Kinematic
			theta << MJoint[0], MJoint[1], MJoint[2], MJoint[3], MJoint[4], MJoint[5], MJoint[6];

			MatrixXd A1(4, 4); A1 << cos(theta(0, 0)), -sin(theta(0, 0))*cos(alpha(0, 0)), sin(theta(0, 0))*sin(alpha(0, 0)), a(0, 0)*cos(theta(0, 0)),
				sin(theta(0, 0)), cos(theta(0, 0))*cos(alpha(0, 0)), -cos(theta(0, 0))*sin(alpha(0, 0)), a(0, 0)*sin(theta(0, 0)),
				0, sin(alpha(0, 0)), cos(alpha(0, 0)), d(0, 0),
				0, 0, 0, 1;
			MatrixXd A2(4, 4); A2 << cos(theta(0, 1)), -sin(theta(0, 1))*cos(alpha(0, 1)), sin(theta(0, 1))*sin(alpha(0, 1)), a(0, 1)*cos(theta(0, 1)),
				sin(theta(0, 1)), cos(theta(0, 1))*cos(alpha(0, 1)), -cos(theta(0, 1))*sin(alpha(0, 1)), a(0, 1)*sin(theta(0, 1)),
				0, sin(alpha(0, 1)), cos(alpha(0, 1)), d(0, 1),
				0, 0, 0, 1;
			MatrixXd A3(4, 4); A3 << cos(theta(0, 2)), -sin(theta(0, 2))*cos(alpha(0, 2)), sin(theta(0, 2))*sin(alpha(0, 2)), a(0, 2)*cos(theta(0, 2)),
				sin(theta(0, 2)), cos(theta(0, 2))*cos(alpha(0, 2)), -cos(theta(0, 2))*sin(alpha(0, 2)), a(0, 2)*sin(theta(0, 2)),
				0, sin(alpha(0, 2)), cos(alpha(0, 2)), d(0, 2),
				0, 0, 0, 1;
			MatrixXd A4(4, 4); A4 << cos(theta(0, 3)), -sin(theta(0, 3))*cos(alpha(0, 3)), sin(theta(0, 3))*sin(alpha(0, 3)), a(0, 3)*cos(theta(0, 3)),
				sin(theta(0, 3)), cos(theta(0, 3))*cos(alpha(0, 3)), -cos(theta(0, 3))*sin(alpha(0, 3)), a(0, 3)*sin(theta(0, 3)),
				0, sin(alpha(0, 3)), cos(alpha(0, 3)), d(0, 3),
				0, 0, 0, 1;
			MatrixXd A5(4, 4); A5 << cos(theta(0, 4)), -sin(theta(0, 4))*cos(alpha(0, 4)), sin(theta(0, 4))*sin(alpha(0, 4)), a(0, 4)*cos(theta(0, 4)),
				sin(theta(0, 4)), cos(theta(0, 4))*cos(alpha(0, 4)), -cos(theta(0, 4))*sin(alpha(0, 4)), a(0, 4)*sin(theta(0, 4)),
				0, sin(alpha(0, 4)), cos(alpha(0, 4)), d(0, 4),
				0, 0, 0, 1;
			MatrixXd A6(4, 4); A6 << cos(theta(0, 5)), -sin(theta(0, 5))*cos(alpha(0, 5)), sin(theta(0, 5))*sin(alpha(0, 5)), a(0, 5)*cos(theta(0, 5)),
				sin(theta(0, 5)), cos(theta(0, 5))*cos(alpha(0, 5)), -cos(theta(0, 5))*sin(alpha(0, 5)), a(0, 5)*sin(theta(0, 5)),
				0, sin(alpha(0, 5)), cos(alpha(0, 5)), d(0, 5),
				0, 0, 0, 1;

			MatrixXd T01(4, 4); T01 << A1;
			MatrixXd T02(4, 4); T02 << A1*A2;
			MatrixXd T03(4, 4); T03 << A1*A2*A3;
			MatrixXd T04(4, 4); T04 << A1*A2*A3*A4;
			MatrixXd T05(4, 4); T05 << A1*A2*A3*A4*A5;
			MatrixXd T06(4, 4); T06 << A1*A2*A3*A4*A5*A6;


			//*******************--------------------------
			// I don't know what are these for?
			FK_x = T06(2, 3);



			x_disp = T06(0, 3);
			data2[0] = x_disp;
			y_disp = T06(2, 3);
			data2[1] = y_disp;

			//*******************----------------------------

			// Inverse Kinematic

			phi_euler = atan2(T06(1, 2), T06(0, 2));
			theta_euler = atan2(sqrt(pow(T06(1, 2), 2) + pow(T06(0, 2), 2)), T06(2, 2));
			psi_euler = atan2(T06(2, 1), -T06(2, 0));

			MatrixXd z0(3, 1); z0 << 0, 0, 1;
			MatrixXd z1(3, 1); z1 << T01(0, 2), T01(1, 2), T01(2, 2);
			MatrixXd z2(3, 1); z2 << T02(0, 2), T02(1, 2), T02(2, 2);
			MatrixXd z3(3, 1); z3 << T03(0, 2), T03(1, 2), T03(2, 2);
			MatrixXd z4(3, 1); z4 << T04(0, 2), T04(1, 2), T04(2, 2);
			MatrixXd z5(3, 1); z5 << T05(0, 2), T05(1, 2), T05(2, 2);
			MatrixXd z6(3, 1); z6 << T06(0, 2), T06(1, 2), T06(2, 2);

			MatrixXd p0(3, 1); p0 << 0, 0, 0;
			MatrixXd p1(3, 1); p1 << T01(0, 3), T01(1, 3), T01(2, 3);
			MatrixXd p2(3, 1); p2 << T02(0, 3), T02(1, 3), T02(2, 3);
			MatrixXd p3(3, 1); p3 << T03(0, 3), T03(1, 3), T03(2, 3);
			MatrixXd p4(3, 1); p4 << T04(0, 3), T04(1, 3), T04(2, 3);
			MatrixXd p5(3, 1); p5 << T05(0, 3), T05(1, 3), T05(2, 3);
			MatrixXd p6(3, 1); p6 << T06(0, 3), T06(1, 3), T06(2, 3);

			MatrixXd J1(6, 1); J1 << z0(1, 0)*(p6(2, 0) - p0(2, 0)) - z0(2, 0)*(p6(1, 0) - p0(1, 0)),
				-z0(0, 0)*(p6(2, 0) - p0(2, 0)) + z0(2, 0)*(p6(0, 0) - p0(0, 0)),
				z0(0, 0)*(p6(1, 0) - p0(1, 0)) - z0(1, 0)*(p6(0, 0) - p0(0, 0)),
				z0(0, 0), z0(1, 0), z0(2, 0);
			MatrixXd J2(6, 1); J2 << z1(1, 0)*(p6(2, 0) - p1(2, 0)) - z1(2, 0)*(p6(1, 0) - p1(1, 0)),
				-z1(0, 0)*(p6(2, 0) - p1(2, 0)) + z1(2, 0)*(p6(0, 0) - p1(0, 0)),
				z1(0, 0)*(p6(1, 0) - p1(1, 0)) - z1(1, 0)*(p6(0, 0) - p1(0, 0)),
				z1(0, 0), z1(1, 0), z1(2, 0);
			MatrixXd J3(6, 1); J3 << z2(1, 0)*(p6(2, 0) - p2(2, 0)) - z2(2, 0)*(p6(1, 0) - p2(1, 0)),
				-z2(0, 0)*(p6(2, 0) - p2(2, 0)) + z2(2, 0)*(p6(0, 0) - p2(0, 0)),
				z2(0, 0)*(p6(1, 0) - p2(1, 0)) - z2(1, 0)*(p6(0, 0) - p2(0, 0)),
				z2(0, 0), z2(1, 0), z2(2, 0);
			MatrixXd J4(6, 1); J4 << z3(1, 0)*(p6(2, 0) - p3(2, 0)) - z3(2, 0)*(p6(1, 0) - p3(1, 0)),
				-z3(0, 0)*(p6(2, 0) - p3(2, 0)) + z3(2, 0)*(p6(0, 0) - p3(0, 0)),
				z3(0, 0)*(p6(1, 0) - p3(1, 0)) - z3(1, 0)*(p6(0, 0) - p3(0, 0)),
				z3(0, 0), z3(1, 0), z3(2, 0);
			MatrixXd J5(6, 1); J5 << z4(1, 0)*(p6(2, 0) - p4(2, 0)) - z4(2, 0)*(p6(1, 0) - p4(1, 0)),
				-z4(0, 0)*(p6(2, 0) - p4(2, 0)) + z4(2, 0)*(p6(0, 0) - p4(0, 0)),
				z4(0, 0)*(p6(1, 0) - p4(1, 0)) - z4(1, 0)*(p6(0, 0) - p4(0, 0)),
				z4(0, 0), z4(1, 0), z4(2, 0);
			MatrixXd J6(6, 1); J6 << z5(1, 0)*(p6(2, 0) - p5(2, 0)) - z5(2, 0)*(p6(1, 0) - p5(1, 0)),
				-z5(0, 0)*(p6(2, 0) - p5(2, 0)) + z5(2, 0)*(p6(0, 0) - p5(0, 0)),
				z5(0, 0)*(p6(1, 0) - p5(1, 0)) - z5(1, 0)*(p6(0, 0) - p5(0, 0)),
				z5(0, 0), z5(1, 0), z5(2, 0);

			MatrixXd Jg(6, 6); Jg << J1, J2, J3, J4, J5, J6;

			//***********
			// I should check the definition in notes
			MatrixXd Tphi(6, 6); Tphi << 1, 0, 0, 0, 0, 0,
				0, 1, 0, 0, 0, 0,
				0, 0, 1, 0, 0, 0,
				0, 0, 0, 0, -sin(phi_euler), cos(phi_euler)*sin(theta_euler),
				0, 0, 0, 0, cos(phi_euler), sin(phi_euler)*sin(theta_euler),
				0, 0, 0, 1, 0, cos(theta_euler);

			MatrixXd Ja(6, 6); Ja << Tphi.inverse()*Jg;

			//*****************

			// Initializing Stiffness Damping and Inertia

			/*MatrixXd stiffness(6, 6); stiffness << 10000, 0, 0, 0, 0, 0, //toward varun desk
				0, 10000000, 0, 0, 0, 0, //up
				0, 0, 0, 0, 0, 0, //out toward workshop
				0, 0, 0, 1000000, 0, 0,
				0, 0, 0, 0, 1000000, 0,
				0, 0, 0, 0, 0, 1000000;*/
			MatrixXd stiffness(6, 6); stiffness << 0, 0, 0, 0, 0, 0, //toward varun desk
				0, 10000000, 0, 0, 0, 0, //up
				0, 0, 0, 0, 0, 0, //out toward workshop
				0, 0, 0, 1000000, 0, 0,
				0, 0, 0, 0, 1000000, 0,
				0, 0, 0, 0, 0, 1000000;
			/*MatrixXd damping(6, 6); damping << 100000, 0, 0, 0, 0, 0,
				0, 100000, 0, 0, 0, 0,
				0, 0, 500, 0, 0, 0,
				0, 0, 0, 500, 0, 0,
				0, 0, 0, 0, 500, 0,
				0, 0, 0, 0, 0, 500;*/
			MatrixXd damping(6, 6); damping << 0, 0, 0, 0, 0, 0,
				0, 100000, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0,
				0, 0, 0, 500, 0, 0,
				0, 0, 0, 0, 500, 0,
				0, 0, 0, 0, 0, 500;
			/*MatrixXd inertia(6, 6); inertia << 1, 0, 0, 0, 0, 0,
				0, 1, 0, 0, 0, 0,
				0, 0, 1, 0, 0, 0,
				0, 0, 0, 100, 0, 0,
				0, 0, 0, 0, 100, 0,
				0, 0, 0, 0, 0, 100;*/
			MatrixXd inertia(6, 6); inertia << 1000, 0, 0, 0, 0, 0,
				0, 1, 0, 0, 0, 0,
				0, 0, 1000, 0, 0, 0,
				0, 0, 0, 100, 0, 0,
				0, 0, 0, 0, 100, 0,
				0, 0, 0, 0, 0, 100;

			MatrixXd torques(6, 1); torques << ETorque[0], ETorque[1], ETorque[2], ETorque[3], ETorque[4], ETorque[5];

			t_e << al*torques + (1 - al)*torques_0;
			torques_0 << t_e;

			//**************************--------------------
			// Don't know
			if (firstIt == 0)//first time inside
			{
				firstIt = 1;
				std::cout << firstIt << std::endl;

				x_e << T06(0, 3), T06(1, 3), T06(2, 3), phi_euler, theta_euler, psi_euler;

				t_0 << -0.0313991, -0.414115, -0.00613078, -0.450773, 0.265899, -0.0418674;
			}
			//**********************------------------------

			//*********************-------------------------
			//??
			ftx = (double)data[0] / 1000000 - zerox; //toward varun desk
			ftx_un = (double)data[0] / 1000000 - zerox;

			fty = (double)data[1] / 1000000 - zeroy;
			fty_un = (double)data[1] / 1000000 - zeroy;

			ftx = al*ftx + (1 - al)*ftx_0;
			ftx_0 = ftx;
			fty = al*fty + (1 - al)*fty_0;
			fty_0 = fty;

			//force << 3.0*sin(M_PI*ww/500),0,3.0*sin(M_PI*ww/500),0,0,0;
			ww = ww + 1;
			force << ftx,0,fty,0,0,0;
			//force << 0, 0, 0, 0, 0, 0;
			//force << 2, 0, 2, 0, 0, 0;  // This force is for evaluting damping
			steady = steady + 1;
			//std::cout << ftx << std::endl;

			perturb_flag = 0;

			//**********************---------------------------

			//?? what is the usage of q_0
			q_0 << MJoint[0], MJoint[1], MJoint[2], MJoint[3], MJoint[4], MJoint[5];

			//**********************
			//?? dono at all
			delay_gate = delay_gate + 1;
			if ((FK_x < gate1) && (FK_x0 > gate1) && (cc == 0) && (delay_gate > 2000))//first time inside
			{
				cc = 1;
				exit_ran = 0;
			}

			FK_x0 = FK_x;

			if (cc == 1)
			{
				if (sketch ==1)
				{
					if (rr == 1)
					{
						x_incr << -0.1*sin(((M_PI / 6)*i1) /dur), 0, -0.1*cos(((M_PI / 6)*i1) / (dur)), 0, 0, 0;
						if (i1 == dur)
						{
							rr = 2;
							i1 = 0;
							i2 = dur - 1;
						}
					}

					if (rr == 2)
					{
						x_incr << -0.1*sin(((M_PI / 6)*i2) / dur), 0, -0.1*cos(((M_PI / 6)*i2) / (dur)), 0, 0, 0;
						if (i2 == -1)
						{
							rr = 3;
							i1 = 0;
							i2 = dur - 1;
						}
					}

					if (rr == 3)
					{
						x_incr << -0.1*sin(((-M_PI / 6)*i1) / dur), 0, - 0.1*cos(((-M_PI / 6)*i1) / (dur)), 0, 0, 0;
						if (i1 == dur)
						{
							rr = 4;
							i1 = 0;
							i2 = dur - 1;
						}
					}

					if (rr == 4)
					{
						x_incr << -0.1*sin(((-M_PI / 6)*i2) / dur), 0, - 0.1*cos(((-M_PI / 6)*i2) / (dur)), 0, 0, 0;
						if (i2 == -1)
						{
							rr = 5;
							i1 = 0;
							i2 = dur - 1;
						}
					}

					if (rr == 5)
					{
						x_incr << (-0.0721*sin((-((M_PI / 10)*i1) / dur) - 391 * M_PI / 1250) - 0.06), 0, (-0.0721*cos((-((M_PI / 10)*i1) / dur) - 391 * M_PI / 1250) - 0.06), 0, 0, 0;
						if (i1 == dur)
						{
							rr = 6;
							i1 = 0;
							i2 = dur - 1;
						}
					}

					if (rr == 6)
					{
						x_incr << (-0.0094*sin((-((M_PI)*i1) / dur) - M_PI / 2)), 0, (-0.0094*cos((-((M_PI)*i1) / dur) - M_PI / 2) - 0.0795), 0, 0, 0;
						if (i1 == dur)
						{
							rr = 7;
							i1 = 0;
							i2 = dur - 1;
						}
					}

					if (rr == 7)
					{
						x_incr << (-0.0694*sin((((M_PI / 6)*i1) / dur) + M_PI / 2) + 0.06), 0, (-0.0694*cos((((M_PI / 6)*i1) / dur) + M_PI / 2) - 0.0795), 0, 0, 0;
						if (i1 == dur)
						{
							rr = 8;
							i1 = 0;
							i2 = dur - 1;
						}
					}

					if (rr == 8)
					{
						x_incr << (-0.0095*sin((((2 * M_PI)*i1) / dur) + M_PI / 2) + 0.0094), 0, (-0.0095*cos((((2 * M_PI)*i1) / dur) + M_PI / 2) - 0.0448), 0, 0, 0;
						if (i1 == dur)
						{
							rr = 9;
							i1 = 0;
							i2 = dur - 1;
						}
					}

					if (rr == 9)
					{
						x_incr << (0.0093*sin((((2 * M_PI)*i1) / dur) + M_PI / 2) - 0.0094), 0, (0.0093*cos((((2 * M_PI)*i1) / dur) + M_PI / 2) - 0.0449), 0, 0, 0;
						if (i1 == dur)
						{
							rr = 10;
							i1 = 0;
							i2 = dur - 1;
						}
					}

					if (rr == 10)
					{
						x_incr << (-0.0901*sin((((M_PI / 5)*i1) / dur) + M_PI / 2) + 0.09), 0, (-0.0901*cos((((M_PI / 5)*i1) / dur) + M_PI / 2) - 0.0448), 0, 0, 0;
						if (i1 == dur)
						{
							rr = 11;
							i1 = 0;
							i2 = dur - 1;
						}
					}

					if (rr == 11)
					{
						x_incr << (0.0702*sin((((2 * M_PI)*i1) / dur) + 0.246)), 0, (0.0702*cos((((2 * M_PI)*i1) / dur) + 0.246) - 0.06), 0, 0, 0;
						if (i1 == dur)
						{
							rr = 12;
							i1 = 0;
							i2 = dur - 1;
						}
					}

					if (rr == 12)
					{
						sketch = 0;
						x_incr << x_incr_0;
						
					}
					
					qc << Ja.inverse()*(x_incr - x_incr_0);//+(q_old-q_0)*0.3;
					delta_q << delta_q + qc;
					q_new << q_freeze + delta_q;
					x_incr_0 << x_incr;


					w = w + 1;
					w2 = w;
					i1++;
					i2--;
				}
			}
			//***********************************************

			//***********************************************

			if (steady < 2000)
			{
				delay_gate = 0;
				ww = 0;
				force << 0, 0, 0, 0, 0, 0;
				zerox = 100 * al*(double)data[0] / 1000000 + (1 - 100 * al)*zerox;
				zeroy = 100 * al*(double)data[1] / 1000000 + (1 - 100 * al)*zeroy;

				x_00 << x_0;
				x_0 << T06(0, 3), T06(1, 3), T06(2, 3), phi_euler, theta_euler, psi_euler;

				x_new << (inertia / 0.00001 + damping / 0.001 + stiffness).inverse()*(force + inertia*(x_0 - x_00) / 0.00001 + stiffness*(x_e - x_0)) + x_0;
				q_new(0) = -1.779862;
				q_new(1) = 0.821814;
				q_new(2) = -0.067855;
				q_new(3) = 1.302481;
				q_new(4) = 0.284275;
				//q_new(5) = -1.118251;
				q_new(5) = 1.9241;
				q_freeze << q_new;
				
			}

			met = met + 1;

			if (met == 2000)
			{
				met = 0;
			}

			if (steady > 2500)
			{
				cc = 1;
			}

			stime = client.GetTimeStamp();

			fprintf(OutputFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %1f %d %lf %lf %lf %lf %lf %lf\n", count, MJoint[0], MJoint[1], MJoint[2], MJoint[3], MJoint[4], MJoint[5], MJoint[6], ftx_un, fty_un, perturb_flag, q_new(0), q_new(1), q_new(2), q_new(3), q_new(4), q_new(5));
			
			//fprintf(OutputFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %1f %d %lf %lf %lf %lf %lf %lf\n", count, MJoint[0], MJoint[1], MJoint[2], MJoint[3], MJoint[4], MJoint[5], MJoint[6], force(0), force(2), perturb_flag, x_new(0), x_new(1), x_new(2), x_new(3), x_new(4), x_new(5));
			
			//fprintf(OutputFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %1f %d %lf %lf %lf %lf %lf %lf\n", count, x_incr(0), x_incr(1), x_incr(2), x_incr(3), x_incr(4), x_incr(5), MJoint[6], ftx_un, fty_un, perturb_flag, q_new(0), q_new(1), q_new(2), q_new(3), q_new(4), q_new(5));
			
			if (c == 1)
			{
				count2++;

				if (count2 == 200 || !success) //was "rows" not 400
				{
					c = 0;
					count2 = 0;
					std::cout << c << std::endl;
				}
				
			}
			else
			{
				x_00 << x_0;
				x_0 << T06(0, 3), T06(1, 3), T06(2, 3), phi_euler, theta_euler, psi_euler;

				x_new << (inertia + damping + stiffness).inverse()*(force + inertia*(x_0 - x_00) + stiffness*(x_e - x_0)) + x_0;
				v=(x_new-x_0)/0.001;
				//std::cout << x_new << std::endl;

				if (trigger == 1) //prevent new joints commands from being sent during position perturbation
				{
					if (0.9 < x_new(2) & x_new(2) < 1.1)
					{
					  if (-0.09<x_new(0) & x_new(0)<0.09)
					  {
						//q_new << Ja.inverse()*(inertia + damping + stiffness).inverse()*(force + inertia*(x_0 - x_00) + stiffness*(x_e - x_0)) + q_0;//+(q_old-q_0)*0.3;
						//std::cout << q_new << std::endl;
					  }																									 // std::cout << q_new << std::endl;
					}
				}

				q_old << q_new;

				q_delay << Ja.inverse()*(inertia + damping + stiffness).inverse()*(force + inertia*(x_0 - x_00) + stiffness*(x_e - x_0)) + q_0;
				if (outerdelay > 9)
				{
					outerdelay = 0;

				}
				outerdelay = outerdelay + 1;

				client.NextJoint[0] = q_new(0);
				client.NextJoint[1] = q_new(1);
				client.NextJoint[2] = q_new(2);
				client.NextJoint[3] = q_new(3);
				client.NextJoint[4] = q_new(4);
				client.NextJoint[5] = q_new(5);
				//client.NextJoint[6] = 1.0472;//-0.7854;
				client.NextJoint[6] = -0.958709;
				
				/*client.NextJoint[0] = -1.570796;
				client.NextJoint[1] = 1.570796;
				client.NextJoint[2] = -1.570796;
				client.NextJoint[3] = 0.523599;
				client.NextJoint[4] = 1.570796;
				client.NextJoint[5] = -1.570796;
				client.NextJoint[6] = 1.0472;*/
				//std::cout << q_new << std::endl;	
				
				//LBR3
				/*client.NextJoint[0] = -1.779862;
				client.NextJoint[1] = 0.821814;
				client.NextJoint[2] = -0.067855;
				client.NextJoint[3] = 1.302481;
				client.NextJoint[4] = 0.284275;
				client.NextJoint[5] = -1.118251;
				client.NextJoint[6] = -0.958709;*/
				
				//LBR6
				/*client.NextJoint[0] = -1.779862;
				client.NextJoint[1] = 0.821814;
				client.NextJoint[2] = -0.067855;
				client.NextJoint[3] = 1.302481;
				client.NextJoint[4] = 0.284275;
				client.NextJoint[5] = 1.9241;
				client.NextJoint[6] = -0.958709;*/
				
				//memcpy(client.LastJoint, client.NextJoint,7*sizeof(double));
				//success2 = fscanf(InputFile, "%lf %lf %lf %lf %lf %lf %lf", &client.NextJoint[0],&client.NextJoint[1],&client.NextJoint[2],&client.NextJoint[3],&client.NextJoint[4],&client.NextJoint[5],&client.NextJoint[6]);

			}
		}
	}

	fclose(OutputFile);
	fprintf(stdout, "File closed.\n\n\n");
	// disconnect from controller

	fprintf(stdout, "Shhh.. I'm sleeping!\n");
	usleep(10000000);//microseconds //wait for close on other side
	app.disconnect();

	gettimeofday(&start, NULL);

	return 1;
}



FILE *NewDataFile(void) //this may be specific to linux OS
{
	FILE *fp;
	time_t rawtime;
	struct tm *timeinfo;
	char namer[40];

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(namer, 40, "Output%Y-%j-%H_%M_%S.txt", timeinfo);//creates a file name that has year-julian day-hour min second (unique for each run, no chance of recording over previous data)
	fp = fopen(namer, "w");//open output file
	return fp;
}


// Shared Memory-------------------------------------------------------
int *MakeFloatSharedMemory(int HowBig)
{
	key_t key;
	int shmid;
	int *dataShared;

	dataShared = (int *)malloc(HowBig * sizeof(int));
	/* make the key */
	if ((key = ftok("/home/justin/Desktop/FRI-Client-SDK_Cpp/example/PositionControl2/shmfile", 'R')) == -1)
	{
		perror("ftok-->");
		exit(1);
	}

	if ((shmid = shmget(key, SHM_SIZE, 0666 | IPC_CREAT)) == -1)
	{
		perror("shmget");
		exit(1);
	}

	dataShared = (int *)shmat(shmid, (void *)0, 0);

	if (dataShared == (int *)(-1))
	{
		perror("shmat");
		exit(1);
	}

	for (int i = 0; i<HowBig; i++)
	{
		dataShared[i] = 0.0;
	}

	return dataShared;
}


// Shared Memory Display--------------------------------------------------------------
float *MakeFloatSharedMemory2(int HowBig2)
{
	key_t key2;
	int shmid2;
	float *dataShared2;

	dataShared2 = (float *)malloc(HowBig2 * sizeof(float));
	/* make the key */
	if ((key2 = ftok("/home/justin/Desktop/FRI-Client-SDK_Cpp/example/PositionControl2/shmfile2", 'R')) == -1)
	{
		perror("ftok-->");
		exit(1);
	}

	if ((shmid2 = shmget(key2, SHM_SIZE, 0666 | IPC_CREAT)) == -1)
	{
		perror("shmget");
		exit(1);
	}

	dataShared2 = (float *)shmat(shmid2, (void *)0, 0);

	if (dataShared2 == (float *)(-1))
	{
		perror("shmat");
		exit(1);
	}

	for (int i = 0; i<HowBig2; i++)
	{
		dataShared2[i] = 0.0;
	}

	return dataShared2;
}