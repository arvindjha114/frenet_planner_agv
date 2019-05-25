#include "include/cubic_spline_planner.hpp"

vecD gPT(vecD x, vecD y)
{
	vecD theta_g;

	for(int i = 0; i < x.size() - 1; i++)
	{
		double dx = x[i + 1] - x[i];
		double dy = y[i + 1] - y[i];

		double theta = atan2(dy, dx);
		theta_g.push_back(theta);
	}

	if(theta_g.size() > 0)
		theta_g.push_back(theta_g.back());

	return theta_g;
}

int main()
{
	vecD x = { 0.0, 1.0, 2.0,  3.0};
	vecD y = {10.0, 7.0, 8.5, -1.0};

	Spline2D csp(x, y);
	Spline csp1;
	csp1.init(x, y);

	cout << "Using spline class :\n";

	vecD t = csp.calc_s(x, y);

	for(int i = 0; i < x.size(); i++)
	{
		cout << csp.calc_yaw(t[i]) << endl;
		// cout << "Using direct spline :\n";	
		// cout << atan(csp1.calcd(x[i])) << endl;
	}

	cout << "Using frenet code :\n";

	printVecD(gPT(x, y));
}