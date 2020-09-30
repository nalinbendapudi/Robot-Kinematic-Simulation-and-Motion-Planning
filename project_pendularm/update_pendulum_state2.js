function update_pendulum_state(numerical_integrator, pendulum, dt, gravity) {
    // integrate pendulum state forward in time by dt
    

    if (typeof numerical_integrator === "undefined")
        numerical_integrator = "none";

    if (numerical_integrator === "euler") {

    // STENCIL: a correct Euler integrator is REQUIRED for assignment
	// Please check RK-4 integrator.That works best
		pendulum.angle_previous[0] = pendulum.angle[0];
		pendulum.angle[0] += pendulum.angle_dot[0]*dt;
		pendulum.angle_dot[0] += pendulum.angle_dot_dot[0] * dt;
		
		pendulum.angle_previous[1] = pendulum.angle[1];
		pendulum.angle[1] += pendulum.angle_dot[1]*dt;
		pendulum.angle_dot[1] += pendulum.angle_dot_dot[1] * dt;
    
    }
    else if (numerical_integrator === "verlet") {

    // STENCIL: basic Verlet integration
	// Please check RK-4 integrator.That works best
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
	// Please check RK-4 integrator.That works best
		
		pendulum.angle_previous[0] = pendulum.angle[0];
		pendulum.angle_previous[1] = pendulum.angle[1];
		
		pendulum.angle[0] += pendulum.angle_dot[0]*dt + pendulum.angle_dot_dot[0]*dt*dt/2;
		pendulum.angle[1] += pendulum.angle_dot[1]*dt + pendulum.angle_dot_dot[1]*dt*dt/2;
		
		var acc = pendulum_acceleration(pendulum,gravity);
		
		pendulum.angle_dot[0] += (pendulum.angle_dot_dot[0] + acc[0])/2*dt
		pendulum.angle_dot[1] += (pendulum.angle_dot_dot[1] + acc[1])/2*dt
		
    }
    else if (numerical_integrator === "runge-kutta") {

    // STENCIL: Runge-Kutta 4 integrator
	// Please consider this integrator as part of ROB511 HW Submission
		
		var x11 = pendulum.angle[0];
		var x12 = pendulum.angle[1];
		var v11 = pendulum.angle_dot[0];
		var v12 = pendulum.angle_dot[1];
		var acc1 = calc_acceleration(x11,v11,x12,v12,pendulum,gravity)
		var a11 = acc1[0];
		var a12 = acc1[1];
		
		var x21 = x11 + v11*dt/2;
		var x22 = x12 + v12*dt/2;
		var v21 = v11 + a11*dt/2;
		var v22 = v12 + a12*dt/2;
		var acc2 = calc_acceleration(x21,v21,x22,v22,pendulum,gravity)
		var a21 = acc2[0];
		var a22 = acc2[1];
		
		var x31 = x11 + v21*dt/2;
		var x32 = x12 + v22*dt/2;
		var v31 = v11 + a21*dt/2;
		var v32 = v12 + a22*dt/2;
		var acc3 = calc_acceleration(x31,v31,x32,v32,pendulum,gravity)
		var a31 = acc3[0];
		var a32 = acc3[1];
		
		var x41 = x11 + v31*dt;
		var x42 = x12 + v32*dt;
		var v41 = v11 + a31*dt;
		var v42 = v12 + a32*dt;
		var acc4 = calc_acceleration(x41,v41,x42,v42,pendulum,gravity)
		var a41 = acc4[0];
		var a42 = acc4[1];
		
		pendulum.angle_previous[0] = pendulum.angle[0];
		pendulum.angle_previous[1] = pendulum.angle[1];
		pendulum.angle[0] += (v11 + 2*v21 + 2*v31 + v41)/6 * dt;
		pendulum.angle[1] += (v12 + 2*v22 + 2*v32 + v42)/6 * dt;
		pendulum.angle_dot[0] += (a11 + 2*a21 + 2*a31 + a41)/6 *dt;
		pendulum.angle_dot[1] += (a12 + 2*a22 + 2*a32 + a42)/6 *dt;
		
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
	
	var a11 = (m1+m2)*l1;
	var a12 = m2*l2*Math.cos(t1-t2);
	var a21 = l1*Math.cos(t1-t2);
	var a22 = l2
	var det = a11*a22 -a12*a21;
	var b1 = -m2*l2*w2*w2*Math.sin(t1-t2) -(m1+m2)*g*Math.sin(t1) +pendulum.control[0]/l1;
	var b2 = l1*w1*w1*Math.sin(t1-t2)-g*Math.sin(t2) +pendulum.control[1]/m2/l2;
	
	var acc = [0,0];
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
    pendulum.servo = {kp:[300,200], kd:[80,60], ki:[3,3]};
    return pendulum;
}

function PID(pendulum, accumulated_error, dt) {
    // STENCIL: implement PID controller
    // return: updated output in pendulum.control and accumulated_error
	
	var error_previous0 = pendulum.desired[0] - pendulum.angle_previous[0];
	var error0 = pendulum.desired[0] - pendulum.angle[0];
	accumulated_error[0] += error0;
	
	var error_previous1 = pendulum.desired[1] - pendulum.angle_previous[1];
	var error1 = pendulum.desired[1] - pendulum.angle[1];
	accumulated_error[1] += error1;
	
	P0 = pendulum.servo.kp[0]*error0;
	I0 = pendulum.servo.ki[0]*accumulated_error[0];
	D0 = pendulum.servo.kd[0]*(error0-error_previous0)/dt;
	
	P1 = pendulum.servo.kp[1]*error1;
	I1 = pendulum.servo.ki[1]*accumulated_error[1];
	D1 = pendulum.servo.kd[1]*(error1-error_previous1)/dt;
	
	pendulum.control = [P0+I0+D0,P1+I1+D1];
	
	
    return [pendulum, accumulated_error];
}