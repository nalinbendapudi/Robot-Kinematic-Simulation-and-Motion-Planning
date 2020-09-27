function update_pendulum_state(numerical_integrator, pendulum, dt, gravity) {
    // integrate pendulum state forward in time by dt
    // please use names 'pendulum.angle', 'pendulum.angle_previous', etc. in else codeblock between line 28-30

    if (typeof numerical_integrator === "undefined")
        numerical_integrator = "none";

    if (numerical_integrator === "euler") {

    // STENCIL: a correct Euler integrator is REQUIRED for assignment
		pendulum.angle_previous = pendulum.angle;
		pendulum.angle += pendulum.angle_dot*dt;
		pendulum.angle_dot += pendulum.angle_dot_dot * dt;
    
	}
    else if (numerical_integrator === "verlet") {

    // STENCIL: basic Verlet integration
		var temp = pendulum.angle;
		pendulum.angle = 2*pendulum.angle - pendulum.angle_previous + pendulum.angle_dot_dot*dt*dt
		pendulum.angle_dot = (pendulum.angle - pendulum.angle_previous) / (2*dt)
		pendulum.angle_previous = temp;
		
    }
    else if (numerical_integrator === "velocity verlet") {

    // STENCIL: a correct velocity Verlet integrator is REQUIRED for assignment
		pendulum.angle_previous = pendulum.angle;
		pendulum.angle += pendulum.angle_dot*dt + pendulum.angle_dot_dot*dt*dt/2;
		pendulum.angle_dot += (pendulum.angle_dot_dot + pendulum_acceleration(pendulum,gravity))/2*dt
		
    }
    else if (numerical_integrator === "runge-kutta") {

    // STENCIL: Runge-Kutta 4 integrator
		var l = pendulum.length;
		var tau = pendulum.control;
		var m = pendulum.mass;
		var g = gravity;
		
		x1 = pendulum.angle;
		v1 = pendulum.angle_dot;
		a1 = -g/l*Math.sin(x1) + tau/(m*l*l);
		
		x2 = x1 + v1*dt/2;
		v2 = v1 + a1*dt/2;
		a2 = -g/l*Math.sin(x2) + tau/(m*l*l);
		
		x3 = x1 + v2*dt/2;
		v3 = v1 + a2*dt/2;
		a3 = -g/l*Math.sin(x3) + tau/(m*l*l);
		
		x4 = x1 + v3*dt;
		v4 = v1 + a3*dt;
		a4 = -g/l*Math.sin(x4) + tau/(m*l*l);
		
		pendulum.angle += (v1 + 2*v2 + 2*v3 + v4)/6 * dt;
		pendulum.angle_dot += (a1 + 2*a2 + 2*a3 + a4)/6 *dt;
		pendulum.angle_previous = pendulum.angle;
		
    } 
    else {
        pendulum.angle_previous = pendulum.angle;
        pendulum.angle = (pendulum.angle+Math.PI/180)%(2*Math.PI);
        pendulum.angle_dot = (pendulum.angle-pendulum.angle_previous)/dt;
        numerical_integrator = "none";
    }

    return pendulum;
}

function pendulum_acceleration(pendulum, gravity) {
    // STENCIL: return acceleration(s) system equation(s) of motion 
    var tau = pendulum.control;
	var th = pendulum.angle;
	var l = pendulum.length;
	var m = pendulum.mass;
	var g = gravity;
	
	var acc = -g/l*Math.sin(th) + tau/(m*l*l);
	
	return acc;
}

function init_verlet_integrator(pendulum, t, gravity) {
    // STENCIL: for verlet integration, a first step in time is needed
    // return: updated pendulum state and time
	
	pendulum.angle += pendulum.angle_dot*dt + pendulum_acceleration(pendulum,gravity)*dt*dt/2
	t+=dt;
	
    return [pendulum, t];
}

function set_PID_parameters(pendulum) {
    // STENCIL: change pid parameters
    //pendulum.servo = {kp:200, kd:75, ki:1};
    pendulum.servo = {kp:500, kd:250, ki:5};
    
    return pendulum;
}

function PID(pendulum, accumulated_error, dt) {
    // STENCIL: implement PID controller
    // return: updated output in pendulum.control and accumulated_error
	var error_previous = pendulum.desired - pendulum.angle_previous;
	var error = pendulum.desired - pendulum.angle;
	accumulated_error += error;
	
	P = pendulum.servo.kp*error;
	I = pendulum.servo.ki*accumulated_error;
	D = pendulum.servo.kd*(error-error_previous)/dt;
	
	pendulum.control = P + I + D;
	
    return [pendulum, accumulated_error];
}