/*
############################################################
## AUTHOR: James Beasley                                  ##
## DATE: August 13, 2017                                  ##
## UDACITY SDC: Project 10 (Model Predictive Controllers) ##
############################################################
*/

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
string hasData(string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
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
double polyeval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0;

    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
    {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

int main()
{
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
            {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                string sdata = string(data).substr(0, length);
                cout << sdata << endl;

                if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
                {
                    string s = hasData(sdata);
                    if (s != "")
                    {
                        auto j = json::parse(s);
                        string event = j[0].get<string>();
                        if (event == "telemetry")
                        {
                            // j[1] is the data JSON object

                            //way-points (should be 6 supplied denoting reference trajectory), this is the target trajectory we fit our poly to
                            vector<double> ptsx = j[1]["ptsx"];
                            vector<double> ptsy = j[1]["ptsy"];

                            //x and y position of the vehicle
                            double px = j[1]["x"];
                            double py = j[1]["y"];

                            //angle of the vehicle (orientation)
                            double psi = j[1]["psi"];

                            //position and orientation above constitute the vehicle's pose

                            //speed of the vehicle
                            double v = j[1]["speed"];

                            //conduct translation and rotation of all way points to simplify future steps (such as cte computation)
                            //the way points are being transformed to the vehicle's perspective
                            for (int i = 0; i < ptsx.size(); i++)
                            {
                                //this allows us to center the car at the origin (x = 0, y = 0)
                                //translate (center the coordinates)
                                double translated_x = ptsx[i] - px;
                                double translated_y = ptsy[i] - py;
                                //this allows us to make psi zero
                                //rotate (counter-clockwise, i.e., psi is negative)
                                ptsx[i] = (translated_x * cos(-psi)) - (translated_y * sin(-psi));
                                ptsy[i] = (translated_x * sin(-psi)) + (translated_y * cos(-psi));
                            }

                            //fit a polynomial to the supplied way points (reference trajectory)

                            //convert vectors to eigen vectors so we can supply them to polyfit
                            Eigen::Map<Eigen::VectorXd> x_way_points(ptsx.data(), ptsx.size());
                            Eigen::Map<Eigen::VectorXd> y_way_points(ptsy.data(), ptsy.size());

                            //fit a third order polynomial to the supplied way points (reference trajectory)
                            Eigen::VectorXd coeffs = polyfit(x_way_points, y_way_points, 3);

                            //compensate for delay/latency, estimate 100ms (our latency estimator) in the future what the state will be to
                            //compensate for the amount of time it takes for the actuation commands (steering and throttle) to propagate through our system
                            //and actually take effect
                            double latency = 0.1; //predict state in 100ms
                            double Lf = 2.67; //distance between the front of the vehicle and its center of gravity
                            double delta = j[1]["steering_angle"]; //current steering angle of vehicle from simulator
                            double acceleration = j[1]["throttle"]; //current acceleration of vehicle from simulator
                            //given our transformations above, px, py, and psi will be zero
                            px = py = psi = 0;
                            //build latency into kinematic equations before sending to solver
                            px = px + v * cos(0) * latency;
                            py = py + v * sin(0) * latency; //this will always be zero given sin(0) = 0
                            psi = psi - (v / Lf) * delta * latency;
                            v = v + acceleration * latency;

                            //compute cte
                            //this would be polyeval(coeffs, px) - py, but since we've the transformation, px and py are always zero
                            double cte = polyeval(coeffs, 0);

                            //compute epsi
                            //this would be psi - desired orientation (i.e., arctan(f'(px)), but since px, py, and psi are always zero it is
                            //greatly simplified, from: psi - atan(coeffs[1] + (2 * px * coeffs[2]) + 3 * coeffs[3] * pow(px, 2)))
                            //to the below:
                            double epsi = -atan(coeffs[1]);

                            //adjust cte and epsi to compensate for latency as well
                            cte = cte + v * sin(epsi) * latency;
                            epsi = epsi + (v / Lf) * delta * latency;

                            //package state
                            Eigen::VectorXd state(6);
                            state << px, py, psi, v, cte, epsi;

                            //compute optimized control inputs for the next time step using MPC, the returned vector will also have the
                            //x/y values for the entire predicted path to show us where the MPC was planning to take the vehicle if we
                            //were to execute all control inputs (steering, throttle) it computed
                            vector<double> optimized_control_inputs_plus_predicted_path = mpc.Solve(state, coeffs);

                            double steer_value;
                            double throttle_value;

                            //extract control inputs
                            steer_value = optimized_control_inputs_plus_predicted_path[0] / (deg2rad(25) * Lf);
                            throttle_value = optimized_control_inputs_plus_predicted_path[1];

                            json msgJson;
                            // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                            // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                            msgJson["steering_angle"] = steer_value;
                            msgJson["throttle"] = throttle_value;

                            //GREEN PREDICTED TRAJECTORY LINE

                            //Display the MPC predicted trajectory
                            vector<double> mpc_x_vals;
                            vector<double> mpc_y_vals;

                            //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                            // the points in the simulator are connected by a Green line

                            //skip the optimized control inputs (first 2 values) and push the predicted x/y values so we can paint the green path
                            for (int i = 2; i < optimized_control_inputs_plus_predicted_path.size(); i++)
                            {
                                //x values (even)
                                if ((i % 2) == 0)
                                {
                                    mpc_x_vals.push_back(optimized_control_inputs_plus_predicted_path[i]);
                                }
                                //y values (odd)
                                else
                                {
                                    mpc_y_vals.push_back(optimized_control_inputs_plus_predicted_path[i]);
                                }
                            }

                            msgJson["mpc_x"] = mpc_x_vals;
                            msgJson["mpc_y"] = mpc_y_vals;

                            //YELOW REFERENCE TRAJECTORY LINE

                            //Display the waypoints/reference line
                            vector<double> next_x_vals;
                            vector<double> next_y_vals;

                            //add the x and y coords for each reference trajectory point we want to lay out using the polynomial coefficients
                            //we computed earlier
                            int num_ref_trajectory_points = 25; //number of reference trajectory points we want to plot in front of the vehicle
                            int spacing_factor = 2; //each point on teh reference trajectory line should be 2 units apart (2 meters in unity - the simulator)
                            for (int i = 1; i <= num_ref_trajectory_points; i++)
                            {
                                //x values are spaced every 2 meters
                                next_x_vals.push_back(i * spacing_factor);
                                //y values are computed using the polynomial coefficients, "i * spacing_factor" is where the polynomial is evaluated at
                                //i.e., f(i * spacing_factor) or said differently f(x)
                                next_y_vals.push_back(polyeval(coeffs, i * spacing_factor));
                            }

                            msgJson["next_x"] = next_x_vals;
                            msgJson["next_y"] = next_y_vals;

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
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
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

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
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
