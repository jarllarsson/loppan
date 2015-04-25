#include "ParamChanger.h"

Random ParamChanger::s_randomEngine;


std::vector<float> ParamChanger::change(const std::vector<float>& p_params,
	const std::vector<float>& p_Pmin, const std::vector<float>& p_Pmax,
	int p_iteration)
{
	int size = p_params.size();
	std::vector<float> result(size, 0.0f);
	std::vector<double> deltaP = getDeltaP(p_params,
		p_Pmin, p_Pmax, p_iteration);
	for (unsigned int i = 0; i < size; i++)
	{
		result[i] = (float)((double)p_params[i] + deltaP[i]);
		result[i] = clamp(result[i], p_Pmin[i], p_Pmax[i]);
	}

	return result;
}

std::vector<float> ParamChanger::getS(unsigned int p_size)
{
	int changeProbabilityPercent = 20;
	std::vector<float> S(p_size, 0.0f);
	for (int i = 0; i < p_size; i++)
	{
		S[i] = s_randomEngine.getRandomInt(0, 99) < changeProbabilityPercent ? 1.0f : 0.0f;
	}
	return S;
}

std::vector<double> ParamChanger::getDeltaP(const std::vector<float>& p_P,
	const std::vector<float>& p_Pmin, const std::vector<float>& p_Pmax,
	int p_iteration)
{
	int size = p_P.size();

	// Get S vector
	std::vector<float> S = getS(size);

	// Calculate delta-P
	std::vector<double> deltaP(size, 0.0);
	std::vector<double> U = s_randomEngine.getRealUniformList(-0.1, 0.1, size);
	for (unsigned int i = 0; i < size; i++)
	{
		double P = (double)p_P[i];
		double R = p_Pmax[i] - p_Pmin[i];
		double c = U[i] * R;

		deltaP[i] = (double)S[i] * c;
	}
	return deltaP;
}

std::pair<float, float> ParamChanger::getMinMaxOfList(const std::vector<float>& p_list)
{
	float valuemin = 999999999.0f;
	float valuemax = 0.0f;
	for (unsigned int i = 0; i<(unsigned int)p_list.size(); i++)
	{
		if (p_list[i] > valuemax) valuemax = p_list[i];
		if (p_list[i] < valuemin) valuemin = p_list[i];
	}
	return pair<float, float>(valuemin, valuemax);
}
