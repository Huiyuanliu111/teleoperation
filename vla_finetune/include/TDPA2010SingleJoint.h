#define DoF 7

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
//#include <string.h>
//#include <math.h>
//#include <list>
#include <queue>

inline void innerProduct(double a, double b, double &dest);
inline double vecLength(double a);
/*****************************************************
* note: if use 3D TDPA, the energy and power vector is only available for the  [0] element
********************************************************/



class TDPA_Leader 
{
public:

	void init();

	void energyObserver(double v, double f, double deltaT);
	void energyController(double& force, double velocity, double E_F_in_delayed, double deltaT);
	double getInputPowerFlow() { return P_L_in; }    //input energy from port 2. Need to transmit to the Follower side.
	double getInputEnergyFlow() { return E_L_in; }    //input energy from port 2. Need to transmit to the Follower side.
	double getOutputEnergyFlow() { return E_L_out; }
	double getDissipatedEnergyFlow() { return E_diss; }
	double getAlpha() { return alpha; }

	double alpha = 0;

	double E_L_in = 0;    //input energy at Leader Side
	double E_L_out = 0;   //output energy at Leader Side

	double P_L_in = 0;    //input power at Leader Side
	double P_L_out = 0;   //output power at Leader Side

	double E_F_in_delayed = 0;

	double E_diss = 0;  // dissipated energy

	double v_L_old = 0;
private:

};


class TDPA_Follower
{
public:

	void init();

	void energyObserver(double v, double f, double deltaT);
	void energyController(double& velocity, double force, double E_L_in_delayed, double deltaT);

	double getInputEnergyFlow() { return E_F_in; }
	double getOutputEnergyFlow() { return E_F_out; }
	double getDissipatedEnergyFlow() { return E_diss; }
	double getBeta() { return beta; }

	double beta = 0;

	double E_F_in = 0;   //input Energy at follower side
	double E_F_out = 0;   //output Energy at follower side

	double P_F_in = 0;   //input Power at follower side
	double P_F_out = 0;   //output Power at follower side

	double E_L_in_delayed = 0;

	double E_diss = 0;  // dissipated energy

	double f_F_old = 0;

private:

};



// class MassSpringFilter
// {
// public:
// 	MassSpringFilter(double m = 0.001, double k = 1000, double time = 0.001, double x[DoF] = { 0 });
// 	~MassSpringFilter() {}

// 	void applyFilter_Z(double fout_n[DoF], double fin_n[DoF]);
// 	void applyFilter_S(double fout_n[DoF], double fin_n[DoF]);
// 	void applyFilter_damping(double fout_n[DoF], double fin_n[DoF]);
// 	void applyFilter_timedomain(double Vout[DoF], double Fout[DoF], double Vin[DoF], double Fin[DoF]);
// 	void setInitPos(double x[DoF]);

// private:
// 	double mass;
// 	double stiffKc;
// 	double sampletime;

// 	double xmc[DoF];   //position virtual mass
// 	double vmc[DoF];   //velocity output (virtual mass)
// 	double amc[DoF];
// 	double f0[DoF];    //force output

// 					 //double fout_n[3];
// 	double fout_n_1[DoF];
// 	double fout_n_2[DoF];

// 	double fin_n[DoF];
// 	double fin_n_1[DoF];
// 	double fin_n_2[DoF];

// };

struct vector3
{
	double x = 0;
	double y = 0;
	double z = 0;
};

struct vector7
{
	double x0 = 0;
	double x1 = 0;
	double x2 = 0;
	double x3 = 0;
	double x4 = 0;
	double x5 = 0;
	double x6 = 0;
};

