#include "TDPA2010.h"
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

inline void elementProduct(double a[DoF], double b[DoF], double dest[DoF]) // 7x1dof
{
	for (int i = 0; i < DoF; i++)
	{
		dest[i] = a[i] * b[i];
	}
}

inline double vecLength(double a[DoF]) // 7x1dof
{
	double sum = 0;
	for (int i = 0; i < DoF; i++)
	{
		sum += a[i] * a[i];
	}
	return sqrt(sum);
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
	resetSamples(alpha, 7);
	resetSamples(E_L_in, 7);
	resetSamples(E_L_out, 7);
	resetSamples(P_L_in, 7);
	resetSamples(P_L_out, 7);
	resetSamples(E_diss, 7);
	resetSamples(v_L_old, 7);
}

void TDPA_Leader::energyObserver(double v[DoF], double f[DoF], double deltaT)
{
	double power[DoF] = {0}, p_in[DoF] = {0}, p_out[DoF] = {0};
	for (int i = 0; i < 7; i++)
	{
		power[i] = v[i] * f[i];
		if (power[i] > 0)
			p_in[i] = power[i];
		else
			p_out[i] = -1 * power[i];

		P_L_in[i] = p_in[i];
		P_L_out[i] = p_out[i];
		E_L_in[i] += p_in[i] * deltaT;
		E_L_out[i] += p_out[i] * deltaT;

		E_diss[i] += deltaT * alpha[i] * v_L_old[i] * v_L_old[i];
	}
}

void TDPA_Leader::energyController(double force[DoF], double velocity[DoF], double E_F_in_delayed[DoF], double deltaT)
{
	double E_pc[DoF] = {0};
	double velocity_length = 0;

	for (int i = 0; i < 7; i++)
	{
		alpha[i] = 0;
		E_pc[i] = E_F_in_delayed[i] - E_L_out[i] + E_diss[i];

		if (E_pc[i] > 0)
			alpha[i] = 0;
		else
		{
			velocity_length = velocity[i] * velocity[i];
			if (velocity_length > 0.00000000001)
			{
				alpha[i] = -1 * E_pc[i] / (deltaT * velocity_length);
				force[i] = force[i] + alpha[i] * velocity[i];
			}

			// E_diss += E_pc; // E_diss is negative
		}

		v_L_old[i] = velocity[i];
	}
}

//--------Follower----------------

void TDPA_Follower::init()
{
	resetSamples(beta, 7);
	resetSamples(E_F_in, 7);
	resetSamples(E_F_out, 7);
	resetSamples(P_F_in, 7);
	resetSamples(P_F_out, 7);
	resetSamples(E_diss, 7);
	resetSamples(f_F_old, 7);
}

void TDPA_Follower::energyObserver(double v[DoF], double f[DoF], double deltaT)
{
	double power[DoF] = {0}, p_in[DoF] = {0}, p_out[DoF] = {0};

	for (int i = 0; i < 7; i++)
	{
		power[i] = v[i] * f[i];
		if (power[i] < 0)
			p_in[i] = power[i];
		else
			p_out[i] = -1 * power[i];

		P_F_in[i] = p_in[i];
		P_F_out[i] = p_out[i];
		E_F_in[i] += p_in[i] * deltaT;
		E_F_out[i] += p_out[i] * deltaT;

		E_diss[i] += deltaT * beta[i] * f_F_old[i] * f_F_old[i];
	}
}

void TDPA_Follower::energyController(double velocity[DoF], double force[DoF], double E_L_in_delayed[DoF], double deltaT)
{
	double E_pc[DoF] = {0};
	double force_length = 0;

	for (int i = 0; i < 7; i++)
	{
		beta[i] = 0;
		E_pc[i] = E_L_in_delayed[i] - E_F_out[i] + E_diss[i];
		// printf("energy: %f", E_pc);
		if (E_pc[i] > 0)
			beta[i] = 0;
		else
		{
			
			force_length = force[i] * force[i];

			if (force_length > 0.000001)
			{
				beta[i] = -1 * E_pc[i] / (deltaT * force_length);
				velocity[i] = velocity[i] - beta[i] * force[i];
			}

			// E_diss += E_pc; // E_diss is negative
		}

		f_F_old[i] = force[i];
	}
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
