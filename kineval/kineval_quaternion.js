//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

// STENCIL: reference quaternion code has the following functions:
//   quaternion_from_axisangle
//   quaternion_normalize
//   quaternion_to_rotation_matrix
//   quaternion_multiply

// **** Function stencils are provided below, please uncomment and implement them ****//

kineval.quaternionFromAxisAngle = function quaternion_from_axisangle(axis,angle) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = {};
	q.a = Math.cos(angle/2);
	q.b = Math.sin(angle/2)*axis[0];
	q.c = Math.sin(angle/2)*axis[1];
	q.d = Math.sin(angle/2)*axis[2];
	return q;

}

kineval.quaternionNormalize = function quaternion_normalize(q1) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = {};
	var norm = q1.a*q1.a + q1.b*q1.b + q1.c*q1.c + q1.d*q1.d; 
	q.a = q1.a/norm;
	q.b = q1.b/norm;
	q.c = q1.c/norm;
	q.d = q1.d/norm;
	return q;

}

kineval.quaternionMultiply = function quaternion_multiply(q1,q2) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = {};
    
	var q1_axis = [q1.b, q1.c, q1.d];
    var q2_axis = [q2.b, q2.c, q2.d];
    var q_axis =  vector_cross(q1_axis, q2_axis) + vector_scalar_product(q1_axis, q2.a) + vector_scalar_product(q2_axis, q1.a);
    
	q.a = q1.a*q2.a - vector_dot(q1_axis, q2_axis);
    q.b = q_axis[0];
    q.c = q_axis[1];
    q.d = q_axis[2];

    return q;


}

kineval.quaternionToRotationMatrix = function quaternion_to_rotation_matrix (q) {
    // returns 4-by-4 2D rotation matrix
	var R = generate_identity ();
	
	R[0][0] = 1 - 2*q.c*q.c - 2*q.d*q.d ;
	R[0][1] = 2*q.b*q.c - 2*q.d*q.a ;
	R[0][2] = 2*q.b*q.d - 2*q.c*q.a ;
	R[1][0] = 2*q.b*q.c - 2*q.d*q.a ;
	R[1][1] = 1 - 2*q.b*q.b - 2*q.d*q.d ;
	R[1][2] = 2*q.c*q.d - 2*q.b*q.a ;
	R[2][0] = 2*q.b*q.d - 2*q.c*q.a ;
	R[2][1] = 2*q.c*q.d - 2*q.b*q.a ;
	R[2][2] = 1 - 2*q.b*q.b - 2*q.c*q.c ;
	
	return R;
}