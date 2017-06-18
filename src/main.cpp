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
#include <cppad/cppad.hpp>

#define DEBUG true;

// for convenience
using json = nlohmann::json;
using CppAD::AD;
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double latency = .1;
const double Lf = 2.67;

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

/********************************
* Convert the ptsx and ptsy from global map coordinate system to the vehicle coordinate system.
********************************/

void ConvertPointIntoCarSpace(Eigen::VectorXd *xvals, Eigen::VectorXd *yvals, 
							  vector<double> ptsx, vector<double> ptsy, double psi, double x_car, double y_car){

	Eigen::VectorXd ptsxVals = Eigen::VectorXd(ptsx.size());
	Eigen::VectorXd ptsyVals = Eigen::VectorXd(ptsy.size());

	for(int i = 0; i<ptsx.size(); i++) 
	  	ptsxVals[i] = (ptsx[i] - x_car) * cos(psi) + (ptsy[i] - y_car) * sin(psi);

	for(int i = 0; i<ptsy.size(); i++) 
	  	ptsyVals[i] = (ptsy[i] - y_car) * cos(psi) - (ptsx[i] - x_car) * sin(psi);

	*xvals = ptsxVals;
	*yvals = ptsyVals;
}

Eigen::VectorXd ComputeStateWithLatency(double v, double steer, double throttle, double cte, double epsi){

  	//factor in delay
	double delay_x = v * latency;
	double delay_y = 0;
	double delay_psi = -v*steer / Lf * latency;

	double delay_v = v + throttle*latency;
	double delay_cte = cte + v*sin(epsi)*latency;
	double delay_epsi = epsi-v*steer /Lf * latency;

	Eigen::VectorXd state(6);
	state << delay_x, delay_y, delay_psi, delay_v, delay_cte, delay_epsi;
	return state;
}


int main() {

  // MPC is initialized here!
  MPC mpc;
  mpc.Init();
  uWS::Hub h;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

          double x_car = j[1]["x"];
          double y_car = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

		  double steer_value = j[1]["steering_angle"];
		  double throttle = j[1]["throttle"];

		  Eigen::VectorXd xvals = Eigen::VectorXd(ptsx.size()); 
		  Eigen::VectorXd yvals = Eigen::VectorXd(ptsy.size() ) ;

		  ConvertPointIntoCarSpace(&xvals, &yvals, ptsx, ptsy, psi, x_car, y_car);  

		  // Fit the point with a 3 order polynomium.
		  auto coeffs = polyfit(xvals, yvals, 3);
			
		  // The cross track error is calculated by evaluating at polynomial at x, f(x)
		  // and subtracting y.

		  double cte = polyeval(coeffs, x_car) - y_car;
		  // Due to the sign starting at 0, the orientation error is -f'(x).
		  // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
		  //double epsi = psi - atan(coeffs[1] + 2*coeffs[2]*px + 3*coeffs[3]*CppAD::pow(px, 2));

		  double epsi = - atan(coeffs[1] +
                              (2 * coeffs[2] * x_car) +
                              (3 * coeffs[3] * pow(x_car,2) ) );
;
		  Eigen::VectorXd state(6);

		  state = ComputeStateWithLatency(v, steer_value, throttle, cte, epsi);
		
          mpc.Solve(state, coeffs);

          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
		  
		  double computed_steer_value;

		  computed_steer_value     = - mpc.steering_angle;
		  double computed_throttle = mpc.throttle ;

		  cout << "__________________________________"<< endl;
		  cout << "steering angle: " << mpc.steering_angle<< endl;
		  cout << "throttle: " << mpc.throttle<< endl;
		  cout << "__________________________________"<< endl;
		  
		  json msgJson;
          msgJson["steering_angle"] = computed_steer_value;
          msgJson["throttle"] = computed_throttle;


          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

		  msgJson["mpc_x"] = mpc.x_computed;
          msgJson["mpc_y"] = mpc.y_computed;
          
		  //Display the waypoints/reference line
          
		  //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
		  vector<double> next_x_vals;
          vector<double> next_y_vals;


          for (int i = 1; i < xvals.size(); i++)
          {
          		next_x_vals.push_back(xvals[i]);
          		next_y_vals.push_back(yvals[i]);
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
