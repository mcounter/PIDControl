#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "pid.h"
#include <math.h>

// Set this parameter True to force use new uWS version for OS other than WINDOWS

#if defined(_WINDOWS) || defined(WINDOWS) || defined(_WIN32) || defined(WIN32) || defined(_WIN64) || defined(WIN64)
#define _HOST ("127.0.0.1")
#define _NEW_UWS_VERSION (true)
#else
#define _NEW_UWS_VERSION (false)
#endif

#define _DEBUG_INFO (false)

#define M_PI       3.14159265358979323846   // pi

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

int main()
{
  uWS::Hub h;

  PID pid; // Will be initialized after the first measurement

#if _NEW_UWS_VERSION
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER>* ws, char *data, size_t length, uWS::OpCode opCode) {
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
				  
				  double steering_value = 0.0;
				  double throttle_value = 0.3;
				  pid.update(cte, speed, angle, steering_value, throttle_value);

				  if (_DEBUG_INFO)
				  {
					  // DEBUG
					  std::cout << "CTE: " << cte << " Speed: " << speed << " Angle: " << angle << std::endl;
					  std::cout << "CTE: " << cte << " Steering Value: " << steering_value << std::endl;
				  }

				  json msgJson;
				  msgJson["steering_angle"] = steering_value;
				  msgJson["throttle"] = throttle_value;
				  auto msg = "42[\"steer\"," + msgJson.dump() + "]";

				  if (_DEBUG_INFO)
				  {
					  std::cout << msg << std::endl;
				  }

				  ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			  }
		  }
		  else {
			  // Manual driving
			  std::string msg = "42[\"manual\",{}]";
			  ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER>* ws, uWS::HttpRequest req) {
	  std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER>* ws, int code, char *message, size_t length) {
	  ws->close();
	  std::cout << "Disconnected" << std::endl;
  });
#else
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

			double steering_value = 0.0;
			double throttle_value = 0.3;
			pid.update(cte, speed, angle, steering_value, throttle_value);

			if (_DEBUG_INFO)
			{
				// DEBUG
				std::cout << "CTE: " << cte << " Speed: " << speed << " Angle: " << angle << std::endl;
				std::cout << "CTE: " << cte << " Steering Value: " << steering_value << std::endl;
			}

			json msgJson;
			msgJson["steering_angle"] = steering_value;
			msgJson["throttle"] = throttle_value;
			auto msg = "42[\"steer\"," + msgJson.dump() + "]";
			
			if (_DEBUG_INFO)
			{
				std::cout << msg << std::endl;
			}

			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
#endif

  int port = 4567;

#ifdef _HOST
  auto host = _HOST;

  if (h.listen(host, port))
#else
  if (h.listen(port))
#endif
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
