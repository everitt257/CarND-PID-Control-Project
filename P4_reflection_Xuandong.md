## Project PID Controller
---
**Feedback control with cross track error and speed error**

The goals/steps of this project are the following:

* Perform feedback control to control the steerning angle and throttle
* Setting parameters for the PID control scheme
* The vehicle in the simulator should run fairly stable

## Rubric points
**Here I will consider the rubric points individually and describe how I addressed each point in my implementation.**

### Describe the effect each of the P, I, D components had in your implementation.
1. The P component of the controller scales with current error. If the error is large, then the system will also feedback a large force to diminish to error. Implementing a large P means the rise time of the system will be small, however the system might suffer from overshoot or oscillation.
2. The I component of the controller deals with past error. This error often come from small flaws in the system, such as the drifting angle of the car. This error is cumlative.
3. The D component of the controller deals with future error. The D component is often used to dampen the oscillation or to make the system responses curve more smooth. 

For this project I choosed `P = 0.3`, `I = 0.0`, `D = 0.6`.

### Describe how the final hyperparameters were chosen.
I manually choosen to these values for my parameter. I first set P to a small number, then slowly increase it until the vehicle oscillate a bit. Next I set D to dampen the oscillation. I didn't use the I term because no information was given about drifting angle. To drive more stable I also added the throttle pid controller. I set my desired speed to be 25 mh/h.

```C++
  PID pid;
  PID pid_throttle;
  // TODO: Initialize the pid variable.
  double Kp = 0.3;
  double Ki = 0.0;
  double Kd = 1;
  
  double Kp2 = 0.3;
  double Ki2 = 0.0;
  double Kd2 = 0.6;

  pid.Init(Kp,Ki,Kd);
  pid_throttle.Init(Kp2,Ki2,Kd2);

  h.onMessage([&pid,&pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    double desiredSpeed = 25.0;

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
          double throttle_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          // try to keep both cte and speed error constant 
          pid.UpdateError(cte);
          steer_value = -pid.TotalError();

          pid_throttle.UpdateError(speed - desiredSpeed);
          throttle_value = -pid_throttle.TotalError();

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

```
Here's my code snippet for implementing it.

---
### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.
This is fairly straight forward project. I wish there were more instructions on how to implement the `twiddle` or `sgd` optimization in c++, I have no clue at how to do so. This PD controller is tuned by hand and I know it's not optimized, so at some curvy road the vehicle could swing a bit. Hopefully I will find a solution to solve it in the future.