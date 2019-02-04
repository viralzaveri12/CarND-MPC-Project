#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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
    std::cout << sdata << std::endl;
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

          /**
           * TODO: Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */
          // Tranform from Map Coordinates System to Car Coordinate System

          double Lf = 2.67;

          for (int i = 0; i < ptsx.size(); i++){
            //Transformation Matrix
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;

            // initial orientation angle = 90
            ptsx[i] = dx * cos(0-psi) - dy * sin(0-psi);
            ptsy[i] = dx * sin(0-psi) + dy * cos(0-psi);
          }
          
          //New ptsx and ptsy are STL vectors and not Eigen Library vectors. They need to be
          //converted to Eigen vector so that we can pass these values to polyfit function.

          //To do this, first we will store the address of the 0th index of vector in a pointer

          double* new_ptsx = &ptsx[0];
          double* new_ptsy = &ptsy[0];

          Eigen::Map<Eigen::VectorXd> transformed_ptsx(new_ptsx,6);
          Eigen::Map<Eigen::VectorXd> transformed_ptsy(new_ptsy,6);

          double steer_value;
          double throttle_value;
          auto coeffs = polyfit(transformed_ptsx,transformed_ptsy,3) ;
          
		  //Now we are in Car coordinate system and car's heading is in x direction. y is on the left.
          //Car position is now (0,0) ie. px = 0 and py = 0. Also steering angle psi value is 0.
          //Below commented lines are actual equation and can be simplified as shown.
          
          //Calculation of cte and orientation error
          //double cte = polyeval(coeffs, px) - py;
          //double cte = polyeval(coeffs, 0) - 0;
          //double cte = polyeval(coeffs, 0);

          //double epsi = psi - atan(coeffs[1] + 2 * coeffs[2] * px + 3 * coeffs[3] * px * px); 
          //double epsi = 0 - atan(coeffs[1] + 2 * coeffs[2] * 0 + 3 * coeffs[3] * 0 * 0);
          //double epsi = -atan(coeffs[1]);


          //Considering the latency
          //There are two logics to implement latency. First is to implement latency
          //and then transformation. Second is to do tranformation first and then
          //implement latency. I am going with the second approach.
          //In this approach, we have already in car's coordinate system
          //So px,py and psi are all 0.
          //Basically, to consider the latency we have to predict state values in future
          //by latency time period.
          
		  double latency = 0.1;
          
		  //Below two lines, first terms are 0. We also consider velocity as constant
          //during the small latendy time. Every change happens in descrete time period.
          //like t, t + 1, t + 2 etc.
          // double new_px = 0 + v * cos(psi) * latency;
          // double new_py = 0 + v * sin(psi) * latency;
          // double new_psi = 0 - v * delta /Lf * latency;
          // double new_epsi = new_psi - atan(coeffs[1]);
          // double new_cte = polyeval(coeffs,0) - 0 + v * sin(new_epsi) * latency;
          // double new_v = v + accel * latency;

          px = v * latency;
          py = 0;
          psi = -v * delta * latency/Lf;
          double epsi = -atan(coeffs[1]) + psi; 
          double cte = polyeval(coeffs, 0) + v * sin(epsi) * latency;
          v += a * latency;

          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;
          
		  //state << new_px, new_py, new_psi, new_v, new_cte, new_epsi;

          //steer_value,throttle_value = mpc.Solve(state,coeffs);
          //auto vars = mpc.Solve(state,coeffs);

          //vars stores 0th element of delta value and acceleration value which we need for calculation.
          //It also includes N values of (x,y) just for trajectory drawing and visual debugging purpose.
          //Check main.cpp result vector for detail explanation how it is defined.

          vector<double>vars = mpc.Solve(state,coeffs);
          //so vars[0] and vars[1] are stored in steer_value and throttle value
          steer_value = vars[0] / (deg2rad(25)*Lf);
          throttle_value = vars[1];
          
		  json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          mpc_x_vals.push_back(0);
          mpc_y_vals.push_back(0);
          
          for(int i=2;i<vars.size();i++){
            if (i%2 == 0){
              mpc_x_vals.push_back(vars[i]);
            }
            else{
              mpc_y_vals.push_back(vars[i]);
            }
          }

          /**
           * TODO: add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Green line
           */
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Yellow line
           */
          double poly_inc = 2.5;
          int num_points = 25;

          for (int i = 1; i < num_points; i++)
          {
            next_x_vals.push_back(i*poly_inc);
            next_y_vals.push_back(polyeval(coeffs,i*poly_inc));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
