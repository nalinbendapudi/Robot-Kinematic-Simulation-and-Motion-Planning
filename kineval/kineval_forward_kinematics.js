
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

    // STENCIL: implement kineval.buildFKTransforms();
	kineval.buildFKTransforms();
}

kineval.buildFKTransforms = function buildFKTransforms() {
	var I = generate_identity();
	mStack = [I];
	traverseFKBase();
	traverseFKLink(robot.base);
}

    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //

function traverseFKBase()  {
	
	mStack.push(stackTop(mStack));
	
	var xyz = robot.origin.xyz;
	var rpy = robot.origin.rpy;
	
	var T = generate_translation_matrix(xyz[0], xyz[1], xyz[2]);
	var Rz = generate_rotation_matrix_Z(rpy[2]);
	var Ry = generate_rotation_matrix_Y(rpy[1]);
	var Rx = generate_rotation_matrix_X(rpy[0]);
	var R = matrix_multiply(Rz, matrix_multiply(Ry,Rx));
	
	mStack[mStack.length-1] = matrix_multiply(stackTop(mStack), matrix_multiply(T,R));
	robot.links[robot.base].xform = stackTop(mStack);
	
	var headingVector = [[0],[0],[1],[1]];
	robot_heading = matrix_multiply(stackTop(mStack), headingVector);
	var lateralVector = [[1],[0],[0],[1]];
	robot_lateral = matrix_multiply(stackTop(mStack), lateralVector);
	
	if (robot.links_geom_imported) {
		mStack[mStack.length-1] = matrix_multiply(stackTop(mStack), matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2)));
	}
	robot.links[robot.base].xform = stackTop(mStack);
}

function traverseFKLink(linkName) {
	
	robot.links[linkName].xform = stackTop(mStack);
	
	if( typeof robot.links[linkName].children != "undefined") {
		for (var j=0; j< robot.links[linkName].children.length; j++){
			traverseFKJoint(robot.links[linkName].children[j]);
		}
	}
	
	mStack.pop();
	
}

function traverseFKJoint(jointName) {
	
	mStack.push(stackTop(mStack));
	
	var xyz = robot.joints[jointName].origin.xyz;
	var rpy = robot.joints[jointName].origin.rpy;
	
	var T = generate_translation_matrix(xyz[0], xyz[1], xyz[2]);
	var Rz = generate_rotation_matrix_Z(rpy[2]);
	var Ry = generate_rotation_matrix_Y(rpy[1]);
	var Rx = generate_rotation_matrix_X(rpy[0]);
	var R = matrix_multiply(Rz, matrix_multiply(Ry,Rx));
	
	var joint_movement;

    if (typeof robot.joints[jointName].type == "undefined" || robot.joints[jointName].type == "revolute" || robot.joints[jointName].type == "continuous") {
        var axis = robot.joints[jointName].axis;
        var angle = robot.joints[jointName].angle;
        var quat = kineval.quaternionFromAxisAngle(axis, angle);
        quat = kineval.quaternionNormalize(quat);
        joint_movement = kineval.quaternionToRotationMatrix(quat);

    } else if (robot.joints[jointName].type == "prismatic") {
        var axis = robot.joints[jointName].axis;
        var angle = robot.joints[jointName].angle;
        var trans = vector_scalar_product(axis, angle);
        joint_movement = generate_translation_matrix(trans[0], trans[1], trans[2]);
    } else {
        joint_movement = generate_identity();
    }

    mStack[mStack.length - 1] = matrix_multiply(stackTop(mStack), matrix_multiply(T, matrix_multiply(R, joint_movement)));
	robot.joints[jointName].xform = stackTop(mStack);
	
	traverseFKLink(robot.joints[jointName].child);
}

function stackTop(list){
	return list[list.length-1];
}