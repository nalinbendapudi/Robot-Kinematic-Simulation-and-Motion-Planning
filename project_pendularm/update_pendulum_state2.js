function update_pendulum_state(numerical_integrator, pendulum, dt, gravity) {
    // integrate pendulum state forward in time by dt
    

    if (typeof numerical_integrator === "undefined")
        numerical_integrator = "none";

    if (numerical_integrator === "euler") {

    // STENCIL: a correct Euler integrator is REQUIRED for assignment
		pendulum.angle_previous[0] = pendulum.angle[0];
		pendulum.angle[0] += pendulum.angle_dot[0]*dt;
		pendulum.angle_dot[0] += pendulum.angle_dot_dot[0] * dt;
		
		pendulum.angle_previous[1] = pendulum.angle[1];
		pendulum.angle[1] += pendulum.angle_dot[1]*dt;
		pendulum.angle_dot[1] += pendulum.angle_dot_dot[1] * dt;
    
    }
    else if (numerical_integrator === "verlet") {

    // STENCIL: basic Verlet integration
		var temp0 = pendulum.angle[0];
		pendulum.angle[0] = 2*pendulum.angle[0] - pendulum.angle_previous[0] + pendulum.angle_dot_dot[0]*dt*dt
		pendulum.angle_dot[0] = (pendulum.angle[0] - pendulum.angle_previous[0]) / (2*dt)
		pendulum.angle_previous[0] = temp0;
		
		var temp1 = pendulum.angle[1];
		pendulum.angle[1] = 2*pendulum.angle[1] - pendulum.angle_previous[1] + pendulum.angle_dot_dot[1]*dt*dt
		pendulum.angle_dot[1] = (pendulum.angle[1] - pendulum.angle_previous[1]) / (2*dt)
		pendulum.angle_previous[1] = temp1;
    
	}
    else if (numerical_integrator === "velocity verlet") {

    // STENCIL: a correct velocity Verlet integrator is REQUIRED for assignment
		acc = pendulum_acceleration(pendulum,gravity);
		
		pendulum.angle[0] += pendulum.angle_dot[0]*dt + pendulum.angle_dot_dot[0]*dt*dt/2;
		pendulum.angle_dot[0] += (pendulum.angle_dot_dot[0] + acc[0])/2*dt
		pendulum.angle_previous[0] = pendulum.angle[0];
		
		pendulum.angle[1] += pendulum.angle_dot[1]*dt + pendulum.angle_dot_dot[1]*dt*dt/2;
		pendulum.angle_dot[1] += (pendulum.angle_dot_dot[1] + acc[1])/2*dt
		pendulum.angle_previous[1] = pendulum.angle[1];
		
    }
    else if (numerical_integrator === "runge-kutta") {

    // STENCIL: Runge-Kutta 4 integrator
    } 
    else {
        pendulum.angle_previous[0] = pendulum.angle[0];
        pendulum.angle[0] = (pendulum.angle[0]+Math.PI/180)%(2*Math.PI);
        pendulum.angle_dot[0] = (pendulum.angle[0]-pendulum.angle_previous[0])/dt;
        pendulum.angle_previous[1] = pendulum.angle[1];
        pendulum.angle[1] = (pendulum.angle[1]-Math.PI/180)%(2*Math.PI);
        pendulum.angle_dot[1] = (pendulum.angle[1]-pendulum.angle_previous[1])/dt;
        numerical_integrator = "none";
    }

    return pendulum;
}

function pendulum_acceleration(pendulum, gravity) {
    // STENCIL: return acceleration(s) system equation(s) of motion 
    var t1 = pendulum.angle[0];
	var w1 = pendulum.angle_dot[0];
	
	var t2 = pendulum.angle[1];
	var w2 = pendulum.angle_dot[1];
	
	return calc_acceleration(t1,w1,t2,w2,pendulum,gravity);
}

function calc_acceleration(t1,w1,t2,w2,pendulum,gravity){
	var l1 = pendulum.length[0];
	var m1 = pendulum.mass[0];
	var l2 = pendulum.length[1];
	var m2 = pendulum.mass[1];
	var g = gravity;
	acc = [0,0];
	
	a11 = (m1+m2)*l1;
	a12 = m2*l2*Math.cos(t1-t2);
	a21 = l1*Math.cos(t1-t2);
	a22 = l2
	det = a11*a22 -a12*a21;
	b1 = -m2*l2*w2*w2*Math.sin(t1-t2)-(m1+m2)*g*Math.sin(t1)
	b2 = l1*w1*w1*Math.sin(t1-t2)-g*Math.sin(t2)
	
	acc[0] = (a22*b1 - a12*b2)/det;
	acc[1] = (-a21*b1 + a11*b2)/det;
	
	return acc;
}

function init_verlet_integrator(pendulum, t, gravity) {
    // STENCIL: for verlet integration, a first step in time is needed
    // return: updated pendulum state and time
	acc = pendulum_acceleration(pendulum,gravity);
	pendulum.angle[0] += pendulum.angle_dot[0]*dt + acc[0]*dt*dt/2
	pendulum.angle[1] += pendulum.angle_dot[1]*dt + acc[1]*dt*dt/2
	t+=dt;
	
    return [pendulum, t];
}

function set_PID_parameters(pendulum) {
    // STENCIL: change pid parameters
    pendulum.servo = {kp:[0,0], kd:[0,0], ki:[0,0]};  // no control
    return pendulum;
}

function PID(pendulum, accumulated_error, dt) {
    // STENCIL: implement PID controller
    // return: updated output in pendulum.control and accumulated_error

    return [pendulum, accumulated_error];
}