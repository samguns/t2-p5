#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <fstream>
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
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          for (size_t i = 0; i < ptsx.size(); i++) {
            /**
             * Shift car reference angle to 90 degrees
             */
            double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;

            ptsx[i] = (shift_x * cos(0 - psi) - shift_y * sin(0 - psi));
            ptsy[i] = (shift_x * sin(0 - psi) + shift_y * cos(0 - psi));
          }

          double *ptrx = &ptsx[0];
          double *ptry = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);
          Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

          auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);
          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);

          /**
           * According to Section 7 of Lesson 19:
           * One approach (to address actuator latency) would be
           * running a simulation using the vehicle model starting
           * from the current state for the duration of the latency.
           * The resulting state from the simulation is the new
           * initial state for MPC.
           *
           * So, given current px, py, psi, v, steer and throttle
           * the vehicle feeds back to us, we apply the kinematic
           * model to compute the new initial state for MPC after
           * 100ms latency.
           * The vehicle uses MPH metric, we have to convert it
           * to Meters Per Second to comply Unity metric (1 meter/unit).
           * 1 Mile per Hour = 0.44704 Meter per second
           */
          v *= 0.44704;
          double Lf = 2.67;
          double dt = 0.1;
          px = 0;
          py = 0;
          psi = 0;
          px += v * cos(psi) * dt;
          py += v * sin(psi) * dt;
          cte += v * sin(epsi) * dt;
          epsi += v / Lf * steer_value * dt;
          /**
           * In the simulator, a positive psi value implies a right turn
           * and a negative value implies a left turn. The workaround to address
           * this problem is change the psi update equation to be:
           * psi_[t+1] = psi[t] - v[t] / Lf * delta[t] * dt
           */
          psi -= v / Lf * steer_value * dt;
          v += throttle_value * dt;

          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;

          /**
           * Read in parameters for online tuning.
           */
          std::ifstream params_file;
          params_file.open("params.csv");
          std::string step_s, int_s, ref_v_s, cte_s, epsi_s, steer_s, a_s, steer_diff_s, a_diff_s, reset;
          size_t step;
          double interval, ref_v, cte_f, epsi_f, steer_f, a_f, steer_diff_f, a_diff_f;
          while (params_file.good()) {
            std::getline(params_file, step_s, ',');
            std::getline(params_file, int_s, ',');
            std::getline(params_file, ref_v_s, ',');
            std::getline(params_file, cte_s, ',');
            std::getline(params_file, epsi_s, ',');
            std::getline(params_file, steer_s, ',');
            std::getline(params_file, a_s, ',');
            std::getline(params_file, steer_diff_s, ',');
            std::getline(params_file, a_diff_s, ',');
            std::getline(params_file, reset, ',');
          }
          step = std::stoul(step_s);
          interval = std::stod(int_s);
          ref_v = std::stod(ref_v_s);
          cte_f = std::stod(cte_s);
          epsi_f = std::stod(epsi_s);
          steer_f = std::stod(steer_s);
          a_f = std::stod(a_s);
          steer_diff_f = std::stod(steer_diff_s);
          a_diff_f = std::stod(a_diff_s);
          if ("1" == reset) {
            SetParams(step, interval, ref_v, cte_f, epsi_f, steer_f, a_f, steer_diff_f, a_diff_f);
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          } else {

            /*
            * Calculate steering angle and throttle using MPC.
            *
            * Both are in between [-1, 1].
            *
            */
            auto vars = mpc.Solve(state, coeffs);

            steer_value = vars[0] / (deg2rad(25));
            throttle_value = vars[1];

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
            for (size_t i = 2; i < vars.size(); i++) {
              if (i % 2 == 0) {
                mpc_x_vals.push_back(vars[i]);
              } else {
                mpc_y_vals.push_back(vars[i]);
              }
            }

            msgJson["mpc_x"] = mpc_x_vals;
            msgJson["mpc_y"] = mpc_y_vals;

            //Display the waypoints/reference line
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
            // the points in the simulator are connected by a Yellow line
            double poly_inc = 2.5;
            int num_points = 25;
            for (int n = 1; n < num_points; n++) {
              next_x_vals.push_back(poly_inc * n);
              next_y_vals.push_back(polyeval(coeffs, poly_inc * n));
            }

            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            // std::cout << msg << std::endl;
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
      } else {
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
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
