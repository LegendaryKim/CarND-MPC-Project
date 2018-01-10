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

#include "gnuplot-iostream.h"

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

  // Gnuplot
  std::vector<double> cte_vector;
  std::vector<double> epsi_vector;
  std::vector<double> steering_vector;
  std::vector<double> throttle_vector;
  std::vector<double> v_vector;

  Gnuplot gp;
  std::cout << "Press Ctrl-C to quit (closing gnuplot window doesn't quit)." << std::endl;
  int temp_i = 0;

  //  gp << "set multiplot layout 2,1\n";
  //  gp << "set title 'CTE'";
  gp << "set terminal qt size 400, 400\n";
  gp << "set terminal qt position 50,50\n";

  h.onMessage([&mpc,&gp,&cte_vector,&epsi_vector,&steering_vector,&throttle_vector,&v_vector,&temp_i](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
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

          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          /*
          * Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          // Transform global waypoints coordinates to vehicle coordinates
          size_t n_waypoints = ptsx.size();
          Eigen::VectorXd ptsx_transformed(n_waypoints);
          Eigen::VectorXd ptsy_transformed(n_waypoints);
          for (int i = 0; i < n_waypoints; i++) {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            ptsx_transformed[i] = dx * cos(-psi) - dy * sin(-psi);
            ptsy_transformed[i] = dx * sin(-psi) + dy * cos(-psi);
          }

          // fit a polynomial to the above x and y coordinates
          auto coeffs = polyfit(ptsx_transformed, ptsy_transformed, 3);

          //  Length from front to CoG
          const double Lf = 2.67;


          // Latancy

          // Actualtor Latency
          const int delay_millisec = 100;
          const double delay = delay_millisec/1000.0;

          // Initial state.
          const double x0 = 0;
          const double y0 = 0;
          const double psi0 = 0;
          const double cte0 = coeffs[0];
          const double epsi0 = -atan(coeffs[1]);

          // State after delay.
          double x_delay = x0 + ( v * cos(psi0) * delay );
          double y_delay = y0 + ( v * sin(psi0) * delay );
          double psi_delay = psi0 - ( v * delta * delay / Lf );
          double v_delay = v + a * delay;
          double cte_delay = cte0 + ( v * sin(epsi0) * delay );
          double epsi_delay = epsi0 + psi_delay;

          // Define state
          Eigen::VectorXd state(6);
          state << x_delay, y_delay, psi_delay, v_delay, cte_delay, epsi_delay;

          // Find Solution
          auto mpc_out = mpc.Solve(state, coeffs);
          int num_pts = (mpc_out.size() - 2) / 2;

          double steer_value = -mpc_out[0];
          double throttle_value = mpc_out[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value/0.436332;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (int i = 0; i < num_pts; i++) {
            mpc_x_vals.push_back(mpc_out[i + 2]);
            mpc_y_vals.push_back(mpc_out[num_pts + i + 2]);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for (int i = 0; i < 30; i++) {
            next_x_vals.push_back(i*5);
            next_y_vals.push_back(polyeval(coeffs, i*5));
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
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
          this_thread::sleep_for(chrono::milliseconds(delay_millisec));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          //Gnuplot
          cte_vector.push_back(cte0);
          epsi_vector.push_back(epsi0);
          steering_vector.push_back(delta);
          throttle_vector.push_back(a);
          v_vector.push_back(v);
          if (temp_i % 10 == 0){
            gp << "set multiplot layout 5,1\n";
            gp << "set lmargin at screen 0.10\n";

            gp << "set ylabel 'CTE' offset 1,0\n";
            gp << "plot '-' binary" << gp.binFmt1d(cte_vector, "array") << "with lines notitle lc 'red'\n";
            gp.sendBinary1d(cte_vector);

            gp << "set ylabel 'Epsi' offset 1,0\n";
            gp << "plot '-' binary" << gp.binFmt1d(epsi_vector, "array") << "with lines notitle lc 'red'\n";
            gp.sendBinary1d(epsi_vector);

            gp << "set ylabel 'Steering' offset 1,0\n";
            gp << "plot '-' binary" << gp.binFmt1d(steering_vector, "array") << "with lines notitle lc 'blue'\n";
            gp.sendBinary1d(steering_vector);

            gp << "set ylabel 'Throttle' offset 1,0\n";
            gp << "plot '-' binary" << gp.binFmt1d(throttle_vector, "array") << "with lines notitle lc 'blue'\n";
            gp.sendBinary1d(throttle_vector);

            gp << "set ylabel 'Velocity' offset 1,0\n";
            gp << "plot '-' binary" << gp.binFmt1d(v_vector, "array") << "with lines notitle lc 'blue'\n";
            gp.sendBinary1d(v_vector);

            gp << "unset multiplot\n";
          }
          temp_i ++;

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
