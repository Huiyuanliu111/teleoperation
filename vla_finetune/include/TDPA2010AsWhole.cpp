#include "TDPA2010AsWhole.h"
#include <memory>

int energyMode = 2;

inline void innerProduct(double a[DoF], double b[DoF], double &dest) // 7 dof
{
	dest = 0;
	for (int i = 0; i < DoF; i++)
	{
		dest += a[i] * b[i];
	}
}

inline void resetSamples(double *x, int size)
{
	for (int i = 0; i < size; i++)
	{
		x[i] = 0;
	}
}

//-------------Leader---------------

void TDPA_Leader::init()
{
	resetSamples(&alpha, 1);
	resetSamples(&E_L_in, 1);
	resetSamples(&E_L_out, 1);
	resetSamples(&P_L_in, 1);
	resetSamples(&P_L_out, 1);
	resetSamples(&E_diss, 1);
	resetSamples(v_L_old, 7);
}

void TDPA_Leader::energyObserver(double v[DoF], double f[DoF], double deltaT)
{
	double power = 0, p_in = 0, p_out = 0;

	innerProduct(v, f, power);
	if (power > 0)
		p_in = power;
	else
		p_out = -1 * power;

	P_L_in = p_in;
	P_L_out = p_out;
	E_L_in += p_in * deltaT;
	E_L_out += p_out * deltaT;

	// double v_L_old_length = 0;
	// innerProduct(v_L_old, v_L_old, v_L_old_length);
	// E_diss += deltaT * alpha * v_L_old_length;
}

void TDPA_Leader::energyController(double force[DoF], double velocity[DoF], double E_F_in_delayed, double deltaT)
{
	double E_pc = E_F_in_delayed - E_L_out + E_diss;

	alpha = 0;

	if (E_pc >= 0)
		alpha = 0;
	else
	{
		double velocity_length = 0;
		innerProduct(velocity, velocity, velocity_length);
		if (velocity_length > 0.000001)
		{
			alpha = -1 * E_pc / (deltaT * velocity_length);
			for (int i = 0; i < DoF; i++)
			{
				force[i] = force[i] + alpha * velocity[i];
			}
		}

		// E_diss += E_pc; // E_diss is negative
		E_diss += deltaT * alpha * velocity_length;
	}

	// for (int i = 0; i < 7; i++)
	// {
	// 	v_L_old[i] = velocity[i];
	// }
}

//--------Follower----------------

void TDPA_Follower::init()
{
	resetSamples(&beta, 1);
	resetSamples(&E_F_in, 1);
	resetSamples(&E_F_out, 1);
	resetSamples(&P_F_in, 1);
	resetSamples(&P_F_out, 1);
	resetSamples(&E_diss, 1);
	resetSamples(f_F_old, 7);
}

void TDPA_Follower::energyObserver(double v[DoF], double f[DoF], double deltaT)
{
	double power = 0, p_in = 0, p_out = 0;
	innerProduct(v, f, power);

	if (power > 0)
		p_in = power;
	else
		p_out = -1 * power;

	P_F_in = p_in;
	P_F_out = p_out;
	E_F_in += p_in * deltaT;
	E_F_out += p_out * deltaT;

	// double f_F_old_length = 0;

	// innerProduct(f_F_old, f_F_old, f_F_old_length);
}

void TDPA_Follower::energyController(double velocity[DoF], double force[DoF], double E_L_in_delayed, double deltaT)
{
	double E_pc = E_L_in_delayed - E_F_out + E_diss;
	// printf("energy: %f", E_pc);
	beta = 0;

	if (E_pc >= 0)
		beta = 0;
	else
	{
		double force_length = 0;
		innerProduct(force, force, force_length);

		if (force_length > 0.000001)
		{
			beta = -1 * E_pc / (deltaT * force_length);
			for (int i = 0; i < 7; i++)
			{
				velocity[i] = velocity[i] + beta * force[i];
			}
		}

		// E_diss += E_pc; // E_diss is negative
		E_diss += deltaT * beta * force_length;
	}

	// for (int i = 0; i < 7; i++)
	// {
	// 	f_F_old[i] = force[i];
	// }
}

// //------------------------------------
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

// 	double KT2 = stiffKc * sampletime * sampletime;
// 	double part_fin[DoF];
// 	double part_fout[DoF];

// 	for (int i = 0; i < DoF; ++i)
// 	{
// 		part_fin[i] = KT2 * (fin_n[i] + 2 * fin_n_1[i] + fin_n_2[i]);
// 		part_fout[i] = (8 * mass - 2 * KT2) * fout_n_1[i] - (4 * mass + KT2) * fout_n_2[i];

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

// 	double KT2 = stiffKc * sampletime * sampletime;
// 	double part_fin[DoF];
// 	double part_fout[DoF];

// 	for (int i = 0; i < DoF; ++i)
// 	{
// 		part_fin[i] = KT2 * fin_n[i];
// 		part_fout[i] = 2 * mass * fout_n_1[i] - mass * fout_n_2[i] + tau * sampletime * fout_n_1[i];

// 		out[i] = (part_fin[i] + part_fout[i]) / (mass + KT2 + tau * sampletime);
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

// 	double KT2 = stiffKc * sampletime * sampletime;
// 	double part_fin[DoF];
// 	double part_fout[DoF];

// 	for (int i = 0; i < DoF; ++i)
// 	{
// 		part_fin[i] = sampletime * fin_n[i];
// 		part_fout[i] = tau * fout_n_1[i];

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
// 	 * fout = k(xm-xmc)
// 	 * amc = (fout-fin)/mc
// 	 * vmc = vmc+amc*T
// 	 * xmc = xmc+vmc*T
// 	 ***********************************/
// 	double damper = 0.5;
// 	mass = 0.05;

// 	for (int i = 0; i < DoF; ++i)
// 	{
// 		Fin[i] = 0;

// 		Fout[i] = stiffKc * (Xin[i] - xmc[i]);
// 		amc[i] = (Fout[i] - Fin[i] - damper * vmc[i]) / mass;
// 		vmc[i] = vmc[i] + amc[i] * sampletime;
// 		xmc[i] = xmc[i] + vmc[i] * sampletime;

// 		Xout[i] = xmc[i];
// 	}
// }

// void MassSpringFilter::setInitPos(double x[DoF])
// {
// 	memcpy(xmc, x, DoF * sizeof(double));
// }
