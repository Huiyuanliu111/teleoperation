/******************** Haptic Communication Library *************************
* Software License Agreement (BSD License)                                *
*                                                                         *
*  Copyright (c) 2017,				                                       *
*  Lehrstuhl f�r Medientechnik                                            *
*  Technische Universit�t M�nchen, Germany                                *
*  All rights reserved.                                                   *
*                                                                         *
*  Redistribution and use in source and binary forms, with or without     *
*  modification, are permitted provided that the following conditions     *
*  are met:                                                               *
*                                                                         *
*  - Redistributions of source code must retain the above copyright       *
*     notice, this list of conditions and the following disclaimer.       *
*  - Redistributions in binary form must reproduce the above              *
*     copyright notice, this list of conditions and the following         *
*     disclaimer in the documentation and/or other materials provided     *
*     with the distribution.                                              *
*  - Neither the name of Technische Universit�t M�nchen nor the names of  *
*     its contributors may be used to endorse or promote products derived *
*     from this software without specific prior written permission.       *
*                                                                         *
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    *
*  'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      *
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS      *
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE         *
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,    *
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,   *
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;       *
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER       *
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT     *
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN      *
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE        *
*  POSSIBILITY OF SUCH DAMAGE.                                            *
***************************************************************************
*
* Description : Time domain passivity control. M. Panzirsch 2019, Mechatronics.
*
*
*
* Author: Xiao Xu
* e-mail: xiao.xu@tum.de
* Contributors:
*
*
* IMPORTANT NOTE: The author of this code doesn't guarantee you safety on your hardware
* even it is the same hardware configuration. It is your responsibility to take care
* of safety issues in case you copy and use this application
*
/** \file     TDPA.h
\brief    TDPA Library header file
*
* Version April 2019
*/

#define DoF 7

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
//#include <string.h>
//#include <math.h>
//#include <list>
#include <queue>

inline void innerProduct(double a[DoF], double b[DoF], double &dest);
inline double vecLength(double a[DoF]);
/*****************************************************
* note: if use 3D TDPA, the energy and power vector is only available for the  [0] element
********************************************************/

class TDPA {

public:
	TDPA() {}
	TDPA(double timestamp); // initializes a deadband class for a 3 DoF haptic signal
	~TDPA(); // kills the deadband class

	void init();

	virtual void energyObserver(double v[DoF], double f[DoF], const int port) {}
	virtual void energyMonitor(double target[DoF], double ref[DoF], double P_limit[3]) {}
	double sampleTime;
	int mode;

private:
	// 0 - classic TDPA with position drift,  1 - new TDPA with energy storage, 2 - new TDPA with PDB sending energy
	double TotalEnergy[3] = { 0 };  // energy sum

};


class TDPA_Master : public TDPA
{
public:
	TDPA_Master() {}
	TDPA_Master(double timestamp); // initializes a deadband class for a 3 DoF haptic signal
	~TDPA_Master(); // kills the deadband class

	void init();

	void energyObserver(double v[DoF], double f[DoF], int portNo);
	void energyMonitor(double target[DoF], double ref[DoF], double P_limit[3]);
	double* getInputPowerFlow() { return P2_L2R; }    //input energy from port 2. Need to transmit to the slave side.
	double* getInputEnergyFlow() { return E2_L2R; }    //input energy from port 2. Need to transmit to the slave side.

	double damper[DoF] = { 0 };


	double E1_L2R[3] = { 0 };   //input E at port 1
	double E1_R2L[3] = { 0 };   //output E port 1
	double E2_L2R[3] = { 0 };   //input port 2
	double E2_R2L[3] = { 0 };   //output port 2

	double P_limit[3] = { 0 };  //upper bound of output powere   
	double E_limit[3] = { 0 };  //upper bound of output energy   

	double P1_L2R[3] = { 0 };     //power at port 1
	double P1_R2L[3] = { 0 };     //power at port 1
	double P2_L2R[3] = { 0 };     //power at port 2
	double P2_R2L[3] = { 0 };     //power at port 2

	double E_diss[3] = { 0 };  // dissipated energy
private:

};


class TDPA_Slave : public TDPA
{
public:
	TDPA_Slave() {}
	TDPA_Slave(double timestamp); // initializes a deadband class for a 3 DoF haptic signal
	~TDPA_Slave(); // kills the deadband class

	void init();

	void energyObserver(double v[DoF], double f[DoF], int portNo);
	void energyMonitor(double target[DoF], double ref[DoF]);
	void energyAssignment(double P2_L2R_recv[3]);   //compute Est, E_des

	double* getDesPower2Master() { return P_R2L_des; }    //input energy from port 2. Need to transmit to the slave side.
	double* getDesEnergy2Master() { return E_R2L_des; }    //input energy from port 2. Need to transmit to the slave side.

	double* getDesPower2Slave() { return P_L2R_des; }    //input energy from port 2. Need to transmit to the slave side.

	double getEst() { return E_st[0]; };

	double damper[DoF] = { 0 };



	void energyStorageUpdate();

	double E3_L2R[3] = { 0 };   //input E at port 3
	double E3_R2L[3] = { 0 };   //output E port 3

	double E4_L2R[3] = { 0 };   //output port 4
	double E4_R2L[3] = { 0 };   //input port 4

	double E5_L2R[3] = { 0 };   //output port 5
	double E5_R2L[3] = { 0 };   //input port 5

	double E_st[3] = { 0 };     //energy storage
	double P_limit[3] = { 0 };  //upper bound of output powere   
	double E_limit[3] = { 0 };  //upper bound of output energy   

	double P3_L2R[3] = { 0 };     //power at port 3
	double P3_R2L[3] = { 0 };     //power at port 3
	double P4_L2R[3] = { 0 };     //power at port 4
	double P4_R2L[3] = { 0 };     //power at port 4
	double P5_L2R[3] = { 0 };     //power at port 5
	double P5_R2L[3] = { 0 };     //power at port 5

	double P_out_act[3] = { 0 };   //actual out from the energy storage
	double P_exc[3] = { 0 };          //excessive power
	double P_exc_R2L[3] = { 0 };     //excessive to the master
	double P_exc_L2R[3] = { 0 };     //excessive to the slave

	double P_R2L_des[3] = { 0 };  //power limit to the master
	double P_L2R_des[3] = { 0 };  //power limit to the slave
	double E_R2L_des[3] = { 0 };  //energy limit to the master
	double E_L2R_des[3] = { 0 };  //energy limit to the slave

	double E_diss[3] = { 0 };  // dissipated energy
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

// struct vector3
// {
// 	double x = 0;
// 	double y = 0;
// 	double z = 0;
// };

// struct vector7
// {
// 	double x0 = 0;
// 	double x1 = 0;
// 	double x2 = 0;
// 	double x3 = 0;
// 	double x4 = 0;
// 	double x5 = 0;
// 	double x6 = 0;
// };

