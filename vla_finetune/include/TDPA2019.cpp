#include "TDPA.h"
#include <memory>

int energyMode = 2;

inline void innerProduct(double a[DoF], double b[DoF], double &dest)        //7 dof
{
	dest = 0;
	for (int i=0; i<DoF; i++){
		dest += a[i] * b[i];
	}
}

inline void elementProduct(double a[DoF], double b[DoF], double dest[DoF])   // 7x1dof
{
	for (int i=0; i<DoF; i++){
		dest[i] = a[i] * b[i];
	}
}

// inline double vecLength(double a[DoF])   // 7x1dof
// {
// 	double sum = 0;
// 	for (int i=0; i<DoF; i++){
// 		sum += a[i] * a[i];
// 	}
// 	return sqrt(sum);
// }

inline void resetSamples(double x[DoF])
{
	for (int i=0; i<DoF; i++){
		x[i] = 0;
	}
}


//----------parent class-----------
TDPA::TDPA(double time)
{
	sampleTime = time;
	mode = energyMode;
}

TDPA::~TDPA()
{}

void TDPA::init()
{
	resetSamples(TotalEnergy);
}

//-------------master---------------
TDPA_Master::TDPA_Master(double timestamp)
{
	sampleTime = timestamp;
	mode = energyMode;
}

TDPA_Master::~TDPA_Master()
{}

void TDPA_Master::init()
{
	resetSamples(damper);
	resetSamples(E1_L2R);
	resetSamples(E1_R2L);
	resetSamples(E2_L2R);
	resetSamples(E2_R2L);
	resetSamples(E_limit);
	resetSamples(P_limit);
	resetSamples(P1_L2R);
	resetSamples(P1_R2L);
	resetSamples(P2_L2R);
	resetSamples(P2_R2L);
	resetSamples(E_diss);
}

void TDPA_Master::energyObserver(double v[DoF], double f[DoF], const int portNo)
{
	double power = 0, p_in = 0, p_out = 0;
	innerProduct(v, f, power);
	if (power > 0)
		p_in = power;
	else
		p_out = -1 * power;

	switch (portNo)
	{
	case 1:
		P1_L2R[0] = p_in;
		P1_R2L[0] = p_out;
		E1_L2R[0] += p_in*sampleTime;
		E1_R2L[0] += p_out*sampleTime;
		break;
	case 2:
		P2_L2R[0] = p_in;
		P2_R2L[0] = p_out;
		E2_L2R[0] += p_in*sampleTime;
		E2_R2L[0] += p_out*sampleTime;
		break;
	default:
		std::cout << "incorrect port number!" << std::endl;
		break;
	}
}

void TDPA_Master::energyMonitor(double target[DoF], double ref[DoF], double P_des[3])
{
	if (mode == 2)
		E_limit[0] = P_des[0];   //energy mode, P_des is the accumulative input energy
	else
	{
		P_limit[0] = P_des[0];
		P_limit[1] = P_des[1];
		P_limit[2] = P_des[2];

		E_limit[0] += P_limit[0] * sampleTime;
	}

	//double E_pc = E_limit[0] - E2_R2L[0] - E_diss[0];
	double E_pc = E_limit[0] - E2_R2L[0] - E_diss[0];

	if (E_pc > 0)
		damper[0] = 0;
	else
	{
		//3D TDPA mode 1: decompose target
		double ref_length = 0;
		innerProduct(ref, ref, ref_length);
		if (ref_length > 0.000001)
		{
			for (int i=0; i<DoF; i++){
				damper[i] = -1 * E_pc / (sampleTime*ref_length);
				target[i] = target[i] + damper[i] * ref[i];
			}
		}

		E_diss[0] += E_pc;   // E_diss is negative
	}
}


//--------slave----------------
TDPA_Slave::TDPA_Slave(double timestamp)
{
	sampleTime = timestamp; // initializes a deadband class for a 3 DoF haptic signal
	mode = energyMode;
}

TDPA_Slave::~TDPA_Slave()
{}

void TDPA_Slave::init()
{
	resetSamples(damper);
	resetSamples(E3_L2R);
	resetSamples(E3_R2L);
	resetSamples(E4_L2R);
	resetSamples(E4_R2L);
	resetSamples(E5_L2R);
	resetSamples(E5_R2L);
	resetSamples(E_st);
	resetSamples(E_limit);
	resetSamples(P_limit);
	resetSamples(P3_L2R);
	resetSamples(P3_R2L);
	resetSamples(P3_L2R);
	resetSamples(P4_L2R);
	resetSamples(P4_R2L);
	resetSamples(P5_L2R);
	resetSamples(P5_R2L);
	resetSamples(P_out_act);
	resetSamples(P_exc);
	resetSamples(P_exc_R2L);
	resetSamples(P_exc_L2R);
	resetSamples(P_R2L_des);
	resetSamples(P_L2R_des);
	resetSamples(E_R2L_des);
	resetSamples(E_L2R_des);
	resetSamples(E_diss);
}

void TDPA_Slave::energyObserver(double v[DoF], double f[DoF], int portNo)
{
	double power = 0, p_in = 0, p_out = 0;
	innerProduct(v, f, power);

	switch (portNo)
	{
	case 3:
		if (power > 0)
			p_in = power;
		else
			p_out = -1 * power;

		P3_L2R[0] = p_in;
		P3_R2L[0] = p_out;
		E3_L2R[0] += p_in*sampleTime;
		E3_R2L[0] += p_out*sampleTime;
		break;
	case 4:
		//for port 4 and 5, the sign of the energy flow should be changed.
		if (power < 0)
			p_in = -1 * power;
		else
			p_out = power;

		P4_R2L[0] = p_in;
		P4_L2R[0] = p_out;
		E4_R2L[0] += p_in*sampleTime;
		E4_L2R[0] += p_out*sampleTime;
		break;
	case 5:
		if (power < 0)
			p_in = -1 * power;
		else
			p_out = power;

		P5_R2L[0] = p_in;
		P5_L2R[0] = p_out;
		E5_R2L[0] += p_in*sampleTime;
		E5_L2R[0] += p_out*sampleTime;
		break;
	default:
		std::cout << "incorrect port number!" << std::endl;
		break;
	}
}

void TDPA_Slave::energyAssignment(double P2_L2R_recv[3])   //should send E2_L2R, otherwise too conservative by packet loss
{
	if (mode == 2)
		E_st[0] = P2_L2R_recv[0] + E4_R2L[0];
	else
	{
		E_st[0] = E_st[0] + (P2_L2R_recv[0] + P4_R2L[0])*sampleTime;   //eq.24   modified as Est=E2in+E4in-E3des-E4des
	}

	P_out_act[0] = P3_R2L[0] + P4_L2R[0];   //eq. 25
	P_exc[0] = E_st[0] / sampleTime - P_out_act[0];   //eq. 28

													  //eq. 26, 27
	if (P_exc[0] >= 0 || abs(P_out_act[0])<0.00000000001)
	{
		P_exc_R2L[0] = 0;
		P_exc_L2R[0] = 0;
	}
	else
	{
		//P_exc_* is negative
		P_exc_R2L[0] = P_exc[0] * P3_R2L[0] / P_out_act[0];
		P_exc_L2R[0] = P_exc[0] * P4_L2R[0] / P_out_act[0];
	}

	//eq. 29, 31
	P_L2R_des[0] = P4_L2R[0] + P_exc_L2R[0];
	E_L2R_des[0] += P_L2R_des[0] * sampleTime;
	P_R2L_des[0] = P3_R2L[0] + P_exc_R2L[0];
	E_R2L_des[0] += P_R2L_des[0] * sampleTime;


}

void TDPA_Slave::energyMonitor(double target[DoF], double ref[DoF])
{
	P_limit[0] = P_L2R_des[0];
	P_limit[1] = P_L2R_des[1];
	P_limit[2] = P_L2R_des[2];

	E_limit[0] += (P_limit[0] * sampleTime);

	double E_pc = E_limit[0] - E4_L2R[0] - E_diss[0];
	//printf("energy: %f", E_pc);
	if (E_pc > 0)
		damper[0] = 0;
	else
	{
		//3D TDPA mode 1: decompose target
		double ref_length = 0;
		innerProduct(ref, ref, ref_length);

		if (ref_length > 0.000001)
		{
			for (int i=0; i<7; i++){
				damper[i] = -1 * E_pc / (sampleTime*ref_length);
				target[i] = target[i] - damper[i] * ref[i];
			}
		}

		E_diss[0] += E_pc;   // E_diss is negative

		energyStorageUpdate();
	}
}


void TDPA_Slave::energyStorageUpdate()
{
	E_st[0] = E_st[0] - (P_L2R_des[0] + P_R2L_des[0])*sampleTime;   // eq. 33
																	//if (E_st[0] < 0)
																	// E_st[0] = 0;
}


//------------------------------------
// MassSpringFilter::MassSpringFilter(double m, double k, double time, double x[DoF])
// {
// 	mass = m;
// 	stiffKc = k;
// 	sampletime = time;

// 	memcpy(xmc, x, DoF * sizeof(double));

// 	resetSamples(vmc);
// 	resetSamples(amc);
// 	resetSamples(f0);

// 	resetSamples(fout_n_1);
// 	resetSamples(fout_n_2);
// 	resetSamples(fin_n);
// 	resetSamples(fin_n_1);
// 	resetSamples(fin_n_2);

// }

// void MassSpringFilter::applyFilter_Z(double out[DoF], double in[DoF])
// {
// 	memcpy(fin_n, in, DoF * sizeof(double));

// 	double KT2 = stiffKc*sampletime*sampletime;
// 	double part_fin[DoF];
// 	double part_fout[DoF];

// 	for (int i = 0; i < DoF; ++i)
// 	{
// 		part_fin[i] = KT2*(fin_n[i] + 2 * fin_n_1[i] + fin_n_2[i]);
// 		part_fout[i] = (8 * mass - 2 * KT2)*fout_n_1[i] - (4 * mass + KT2)*fout_n_2[i];

// 		out[i] = (part_fin[i] + part_fout[i]) / (4 * mass + KT2);
// 	}

// 	memcpy(fin_n_2, fin_n_1, DoF * sizeof(double));
// 	memcpy(fin_n_1, fin_n, DoF * sizeof(double));
// 	memcpy(fout_n_2, fout_n_1, DoF * sizeof(double));
// 	memcpy(fout_n_1, out, DoF * sizeof(double));
// }


// void MassSpringFilter::applyFilter_S(double out[DoF], double in[DoF])
// {
// 	memcpy(fin_n, in, DoF * sizeof(double));

// 	double tau = 0.001;

// 	double KT2 = stiffKc*sampletime*sampletime;
// 	double part_fin[DoF];
// 	double part_fout[DoF];

// 	for (int i = 0; i < DoF; ++i)
// 	{
// 		part_fin[i] = KT2*fin_n[i];
// 		part_fout[i] = 2 * mass * fout_n_1[i] - mass*fout_n_2[i] + tau*sampletime*fout_n_1[i];

// 		out[i] = (part_fin[i] + part_fout[i]) / (mass + KT2 + tau*sampletime);
// 	}

// 	memcpy(fin_n_2, fin_n_1, DoF * sizeof(double));
// 	memcpy(fin_n_1, fin_n, DoF * sizeof(double));
// 	memcpy(fout_n_2, fout_n_1, DoF * sizeof(double));
// 	memcpy(fout_n_1, out, DoF * sizeof(double));
// }

// void MassSpringFilter::applyFilter_damping(double out[DoF], double in[DoF])
// {
// 	memcpy(fin_n, in, DoF * sizeof(double));

// 	double tau = 0.05;

// 	double KT2 = stiffKc*sampletime*sampletime;
// 	double part_fin[DoF];
// 	double part_fout[DoF];

// 	for (int i = 0; i < DoF; ++i)
// 	{
// 		part_fin[i] = sampletime*fin_n[i];
// 		part_fout[i] = tau*fout_n_1[i];

// 		out[i] = (part_fin[i] + part_fout[i]) / (tau + sampletime);
// 	}

// 	memcpy(fin_n_2, fin_n_1, DoF * sizeof(double));
// 	memcpy(fin_n_1, fin_n, DoF * sizeof(double));
// 	memcpy(fout_n_2, fout_n_1, DoF * sizeof(double));
// 	memcpy(fout_n_1, out, DoF * sizeof(double));
// }

// void MassSpringFilter::applyFilter_timedomain(double Xout[DoF], double Fout[DoF], double Xin[DoF], double Fin[DoF])
// {
// 	/**********************************
// 	* fout = k(xm-xmc)
// 	* amc = (fout-fin)/mc
// 	* vmc = vmc+amc*T
// 	* xmc = xmc+vmc*T
// 	***********************************/
// 	double damper = 0.5;
// 	mass = 0.05;

// 	for (int i = 0; i < DoF; ++i)
// 	{
// 		Fin[i] = 0;

// 		Fout[i] = stiffKc*(Xin[i] - xmc[i]);
// 		amc[i] = (Fout[i] - Fin[i] - damper*vmc[i]) / mass;
// 		vmc[i] = vmc[i] + amc[i] * sampletime;
// 		xmc[i] = xmc[i] + vmc[i] * sampletime;

// 		Xout[i] = xmc[i];
// 	}
// }

// void MassSpringFilter::setInitPos(double x[DoF])
// {
// 	memcpy(xmc, x, DoF * sizeof(double));
// }




