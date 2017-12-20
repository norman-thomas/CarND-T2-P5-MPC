#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
const string hasData(const string &s)
{
	const auto found_null = s.find("null");
	const auto b1 = s.find_first_of("[");
	const auto b2 = s.rfind("}]");
	if (found_null != string::npos)
	{
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos)
	{
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

// Evaluate a polynomial.
const double polyeval(const Eigen::VectorXd &coeffs, const double x)
{
	double result = 0.0;
	for (size_t i = 0; i < coeffs.size(); i++)
	{
		result += coeffs[i] * pow(x, i);
	}
	return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
const Eigen::VectorXd polyfit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, const int order)
{
	assert(xvals.size() == yvals.size());
	assert(order >= 1 && order <= xvals.size() - 1);
	Eigen::MatrixXd A(xvals.size(), order + 1);

	for (size_t i = 0; i < xvals.size(); i++)
	{
		A(i, 0) = 1.0;
	}

	for (size_t j = 0; j < xvals.size(); j++)
	{
		for (size_t i = 0; i < order; i++)
		{
			A(j, i + 1) = A(j, i) * xvals(j);
		}
	}

	const auto Q = A.householderQr();
	const auto result = Q.solve(yvals);
	return result;
}

int main()
{
	uWS::Hub h;

	// MPC is initialized here!
	MPC mpc;

	h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
					   uWS::OpCode opCode)
				{
					// "42" at the start of the message means there's a websocket message event.
					// The 4 signifies a websocket message
					// The 2 signifies a websocket event
					string sdata = string(data).substr(0, length);
					cout << sdata << endl;
					if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
					{
						const string s = hasData(sdata);
						if (s != "")
						{
							auto j = json::parse(s);
							string event = j[0].get<string>();
							if (event == "telemetry")
							{
								// j[1] is the data JSON object
								vector<double> ptsx = j[1]["ptsx"];
								vector<double> ptsy = j[1]["ptsy"];
								const double px = j[1]["x"];
								const double py = j[1]["y"];
								const double psi = j[1]["psi"];
								const double v = j[1]["speed"];

								const double psi_sin = sin(psi);
								const double psi_cos = cos(psi);
								for (size_t i = 0; i < ptsx.size(); i++)
								{
									const double dx = ptsx[i] - px;
									const double dy = ptsy[i] - py;
									ptsx[i] = dx * psi_cos + dy * psi_sin;
									ptsy[i] = -dx * psi_sin + dy * psi_cos;
								}

								Eigen::Map<Eigen::VectorXd> xvals(ptsx.data(), ptsx.size());
								Eigen::Map<Eigen::VectorXd> yvals(ptsy.data(), ptsy.size());
								const Eigen::VectorXd coeffs = polyfit(xvals, yvals, 3);

								const double cte = polyeval(coeffs, 0);
								const double oe = -atan(coeffs[1]);
								cout << "Errors: cte=" << cte << " oe=" << oe << std::endl;

								Eigen::VectorXd state(6);
								state << 0, 0, 0, v, cte, oe;

								// Use MPC to calculate steering and throttle
								const vector<double> result = mpc.Solve(state, coeffs);
								const double steer_value = result[0];
								const double throttle_value = result[1];

								json msgJson;
								// NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
								// Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
								msgJson["steering_angle"] = steer_value;
								msgJson["throttle"] = throttle_value;

								//Display the MPC predicted trajectory
								vector<double> mpc_x_vals;
								vector<double> mpc_y_vals;

								//.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
								// the points in the simulator are connected by a Green line

								for (size_t i = 1; i < result.size() / 2; i++)
								{
									mpc_x_vals.push_back(result[2 * i]);
									mpc_y_vals.push_back(result[2 * i + 1]);
								}

								msgJson["mpc_x"] = mpc_x_vals;
								msgJson["mpc_y"] = mpc_y_vals;

								//Display the waypoints/reference line
								msgJson["next_x"] = ptsx;
								msgJson["next_y"] = ptsy;


								auto msg = "42[\"steer\"," + msgJson.dump() + "]";
								std::cout << msg << std::endl;
								// Latency
								// The purpose is to mimic real driving conditions where
								// the car does actuate the commands instantly.
								//
								// Feel free to play around with this value but should be to drive
								// around the track with 100ms latency.
								//
								// NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
								// SUBMITTING.
								this_thread::sleep_for(chrono::milliseconds(100));
								ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
							}
						}
						else
						{
							// Manual driving
							std::string msg = "42[\"manual\",{}]";
							ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
						}
					}
				});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
					   size_t, size_t)
					{
						const std::string s = "<h1>Hello world!</h1>";
						if (req.getUrl().valueLength == 1)
						{
							res->end(s.data(), s.length());
						}
						else
						{
							// i guess this should be done more gracefully?
							res->end(nullptr, 0);
						}
					});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
				   {
					   std::cout << "Connected!!!" << std::endl;
				   });

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
						   char *message, size_t length)
					  {
						  ws.close();
						  std::cout << "Disconnected" << std::endl;
					  });

	int port = 4567;
	if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
