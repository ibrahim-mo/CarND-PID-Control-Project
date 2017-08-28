#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <fstream>
#include <time.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

//Uncomment this line only if you want to run fine tuning with twiddle
//#define APPLY_TWIDDLE

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variables
  // Kp, Ki, Kd

  // parameter values for the manual tuning method
  //pid.Init(0.01, 0.0, 0.0);
  //pid.Init(0.05, 0.0, 0.0);
  //pid.Init(0.1, 0.0, 0.0); //Ku=0.1 -> Kp=0.5*Ku=0.05
  //pid.Init(0.5, 0.0, 0.0);
  //pid.Init(0.05, 0.0001, 0.0);
  //pid.Init(0.05, 0.0005, 0.0);
  //pid.Init(0.05, 0.001, 0.0);
  //pid.Init(0.05, 0.001, 0.5);
  //pid.Init(0.05, 0.001, 1.0);
  //pid.Init(0.05, 0.001, 2.0);

  // Using the Ziegler–Nichols method
  //pid.Init(0.1, 0.0, 0.0); //Ku=0.1 (similar to manual method)
  //pid.Init(0.06, 0.0012, 0.75); //after applying Ku=0.1 & Tu=100

  // Final fine tuned values using twiddle
  // starting with inital values (0.05, 0.001, 1.0)
  pid.Init(0.250829, 0.00256535, 5.91271);

  //parameters needed for applying twiddle (gradient descent algorithm)
  int n=300, it=0, i=0;
  double err=0.0, best_err=0.0;
  double dp[3] = {pid.Kp, pid.Ki, pid.Kd};// d_Kp, d_Ki, d_Kd
  double err_tol = 0.001; // error tolerance
  bool reset = false; //reset simulator flag
  int state = 0; //variable to control states of events in twittle
  std::ofstream ofs("result.txt");
  auto t = time(0);

  h.onMessage([&pid,&n,&it,&i,&err,&best_err,&dp,&err_tol,&reset,&state,&ofs,&t](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

#ifdef APPLY_TWIDDLE
          // use twiddle to cacluate Kp, Ki, and Kd
          if (it%n == 0)
            err = 0.0;
          err += cte * cte / n;

          if (it == 2*n-1)
            best_err = err;

          double p[3] = {pid.Kp, pid.Ki, pid.Kd};
          //double sum_dp = dp[0] + dp[1] + dp[2];

          std::cout << "it=" << it << std::endl;
          std::cout << "err=" << err << ", best_err=" << best_err << std::endl;
          std::cout << "Kp=" << p[0] << ", Ki=" << p[1] << ", Kd=" << p[2] << std::endl;
          std::cout << "d_Kp=" << dp[0] << ", d_Ki=" << dp[1] << ", d_Kd=" << dp[2] << std::endl;
          //std::cout << "sum(dp)=" << sum_dp << std::endl;

          if ((it+1)%(2*n) == 0 && best_err > err_tol) {
            switch (state) {
              case 0:
                p[i] += dp[i];
                state = 1;
                break;
              case 1:
                if (err < best_err) {
                  best_err = err;
                  dp[i] *= 1.1;
                  i = (i+1)%3;
                  //as if going back to case 0, but without extra 2*n delay
                  p[i] += dp[i];
                  state = 1;
                }
                else {
                  p[i] -= 2*dp[i];
                  state = 2;
                }
                break;
              case 2:
                if (err < best_err) {
                  best_err = err;
                  dp[i] *= 1.1;
                }
                else {
                  p[i] += dp[i];
                  dp[i] *= 0.9;
                }
                i = (i+1)%3;
                //as if going back to case 0, but without extra 2*n delay
                p[i] += dp[i];
                state = 1;
                break;
            }
            reset = true;
          }
#endif //APPLY_TWIDDLE

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          // write steering values into a file for later simulation (see pid_sim.py)
          ofs << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

#ifdef APPLY_TWIDDLE
          pid.Kp = p[0];
          pid.Ki = p[1];
          pid.Kd = p[2];
          it++;

          if(reset) {
            // Reset simulator
            std::cout << "Reseting simulator!" << std::endl;
            std::string rst_msg = "42[\"reset\",{}]";
            ws.send(rst_msg.data(), rst_msg.length(), uWS::OpCode::TEXT);
            reset = false;
            auto t2 = time(0);
            std::cout << "Time elapsed=" << (t2 - t) << std::endl;
            t = t2;
          }
#endif //APPLY_TWIDDLE
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
