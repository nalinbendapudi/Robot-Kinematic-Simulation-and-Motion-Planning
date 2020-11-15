
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

    // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
            Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
            + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
            + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );

    // if target reached, increment scoring and generate new target location
    // KE 2 : convert hardcoded constants into proper parameters
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
        kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
        kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
        kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
        kineval.params.trial_ik_random.targets += 1;
        textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
    }
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration

    // [NOTICE]: Please assign the following 3 variables to test against CI grader

    // ---------------------------------------------------------------------------
    // robot.dx = []              // Error term, matrix size: 6 x 1, e.g., [[1],[1],[1],[0],[0],[0]]
    // robot.jacobian = []        // Jacobian matrix of current IK iteration matrix size: 6 x N
    // robot.dq = []              // Joint configuration change term (don't include step length)  
    // ---------------------------------------------------------------------------

    // Explanation of above 3 variables:
    // robot.dq = T(robot.jacobian) * robot.dx  // where T(robot.jacobian) means apply some transformations to the Jacobian matrix, it could be Transpose, PseudoInverse, etc.
    // dtheta = alpha * robot.dq   // alpha: step length
	
	
	// Get the chain of joints from base to EE
	var jointsList = joints_hierarchy(endeffector_joint);	// jointsList contains all joints from base to end-effector joint
	var num_joints = jointsList.length;
	
	// Find EE position in world frame
	var endeffector_xform = robot.joints[endeffector_joint].xform;
	var endeffector_position_world = matrix_multiply(endeffector_xform, endeffector_position_local);	// 4X1
	var endeffector_orientation_world = extractEulerAngles(endeffector_xform);							// 3X1
	
	// Find difference between desired and current pos of EE
	var delta_position = vector_subtract(vectorize(endeffector_target_world.position), vectorize(endeffector_position_world));	// Array(4)
	var delta_orientation = vector_subtract(vectorize(endeffector_target_world.orientation), vectorize(endeffector_orientation_world));	// Array(4)
	if (kineval.params.ik_orientation_included){      
		robot.dx = [[delta_position[0]], [delta_position[1]], [delta_position[2]], [delta_orientation[0]], [delta_orientation[1]], [delta_orientation[2]] ];	//6X1
    }
	else{
		robot.dx = [[delta_position[0]], [delta_position[1]], [delta_position[2]], [0], [0], [0] ];	//6X1
	}
	
	// Compute jacobian
	robot.jacobian = compute_jacobian(jointsList);
    
	// Approximate inverse of jacobian (either by pseudoinverse or transpose) 
	if (kineval.params.ik_pseudoinverse) {
        // Pseudo_inverse Jacobian
        var inv_jacobian = matrix_pseudoinverse(robot.jacobian);
    } else {
        // Transpose Jacobian
        var inv_jacobian = matrix_transpose(robot.jacobian);
    }

	// Calculate joint angles
    robot.dq = vectorize(matrix_multiply(inv_jacobian, robot.dx));		// Array(6)

    // Gradient Descent
    for (var j=0; j<num_joints; j++) {
        robot.joints[jointsList[j]].control += kineval.params.ik_steplength * robot.dq[j];
    }
	
}


function joints_hierarchy (endeffector_joint){
	var joints = [];
	var cur_joint = endeffector_joint;
	joints.push(cur_joint);
	
	while(robot.joints[cur_joint].parent!= robot.base){
		cur_joint = robot.links[robot.joints[cur_joint].parent].parent;
		joints.push(cur_joint);
	}
	return vector_reverse(joints);
}

function compute_jacobian (joints) {
	var num_joints = joints.length;
	endeffector_joint = robot.joints[joints[num_joints-1]];
	
	jacobian = [];
	
	for (var i = 0; i < num_joints; i++) {

        var cur_joint = robot.joints[joints[i]];
        var joint_type = cur_joint.type;

        var axis_local = matricize(cur_joint.axis); // 3x1
        var axis_world = matrix_multiply(extract_rotation_matrix(cur_joint.xform), axis_local); // 3x1

        if (joint_type == "prismatic")
        {
            jacobian[i] = [axis_world[0][0],axis_world[1][0],axis_world[2][0] , 0, 0, 0]; // Array(6)
        } 
        else
        {    
			var o_n_homo_world = extract_translation_vector(endeffector_joint.xform)	// Array(3)
            var o_cur_homo_world = extract_translation_vector(cur_joint.xform)	//Array(3)
            var o_delta = vector_subtract(o_n_homo_world, o_cur_homo_world); // Array(3)

            var J_pos = vector_cross(vectorize(axis_world), o_delta);
            var J_rot = vectorize(axis_world);

            jacobian[i] = [J_pos[0], J_pos[1], J_pos[2], J_rot[0], J_rot[1], J_rot[2]];
        }
    }

    jacobian = matrix_transpose(jacobian);
    return jacobian;

}

function extractEulerAngles (xform) {
	var theta1 = [Math.atan2( xform[2][1], xform[2][2])];
    var theta3 = [Math.atan2( xform[1][0], xform[0][0])];
    var temp = Math.pow(  Math.pow( xform[2][1], 2 ) + Math.pow( xform[2][2], 2 ) ,  0.5 );
    var theta2=[Math.atan2(-xform[2][0],temp)];
    var theta = [theta1,theta2,theta3];
    return theta;
}