//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}

function vector_copy(v) {

    var vec = [];
    var i;

    for (i=0;i<v.length;i++) { 
            vec[i] = v[i];
    }
    return vec;
}


function matrix_multiply (m1,m2) {
	if(m1[0].length != m2.length)
		console.log("matrix_multiply: matrices have incompatible sizes");
	var mat = [];
	var i,j,k;
	for (i=0; i<m1.length; i++){
		mat[i] = [];
		for(j=0; j<m2[0].length; j++){
			mat[i][j] = 0;
			for(k=0; k<m1[i].length; k++){
				mat[i][j] += m1[i][k]*m2[k][j];
			}
		}
	}
	return mat;
}

function matrix_transpose (m1){
	var mat = [];
    var i,j;

    for (i=0;i<m1[0].length;i++) {
        mat[i] = [];
        for (j=0;j<m1.length;j++) {
            mat[i][j] = m1[j][i];
        }
    }
    return mat;
}

function vector_normalize (v1){
	var vec = []
	var i;
	var norm=0;
	
	for (i=0; i<v1.length; i++){
		norm += v1[i]*v1[i];
	}
	norm = Math.sqrt(norm);
	for (i=0; i<v1.length; i++){
		vec[i] = v1[i]/norm;
	}
	return vec;
}

function vector_norm (v,n) {
	var norm = 0;
	for (var i=0; i<v.length; i++){
		norm += Math.pow(Math.abs(v[i]),n);
	}
	norm = Math.pow(norm, 1.0/n);
	return norm;
}

function vector_cross (v1, v2){
	var vec = []
	vec[0] = v1[1]*v2[2] - v1[2]*v2[1];
	vec[1] = v1[2]*v2[0] - v1[0]*v2[2];
	vec[2] = v1[0]*v2[1] - v1[1]*v2[0];
	return vec;
}

function vector_dot (v1, v2){
	var dot_product=0;
	for(var i=0; i<v1.length; i++){
		dot_product+= (v1[i]*v2[i]);
	}
	return dot_product;
}

function vector_scalar_product (v, s){
	var scaled = [];
	for(var i=0; i<v.length; i++){
		scaled[i] = (v[i]*s);
	}
	return scaled;
}

function generate_identity (){
	var mat = [];
    var i,j;

    for (i=0;i<4;i++) {
        mat[i] = [];
        for (j=0;j<4;j++) {
            if(i==j)
				mat[i][j] = 1;
			else
				mat[i][j] = 0;
        }
    }
    return mat;
}

function generate_translation_matrix (x,y,z){
	var mat = [];
    var i,j;

    for (i=0;i<4;i++) {
        mat[i] = [];
        for (j=0;j<4;j++) {
            if(i==j)
				mat[i][j] = 1;
			else
				mat[i][j] = 0;
        }
    }
	mat[0][3] = x;
	mat[1][3] = y;
	mat[2][3] = z;
	
	return mat;
}

function generate_rotation_matrix_X (theta){
	var mat = [];
    var i,j;

    for (i=0;i<4;i++) {
        mat[i] = [];
        for (j=0;j<4;j++) {
            if(i==j)
				mat[i][j] = 1;
			else
				mat[i][j] = 0;
        }
    }
	mat[1][1] = Math.cos(theta);
	mat[1][2] = -Math.sin(theta);
	mat[2][1] = Math.sin(theta);
	mat[2][2] = Math.cos(theta);
	
	return mat;
}

function generate_rotation_matrix_Y (theta){
	var mat = [];
    var i,j;

    for (i=0;i<4;i++) {
        mat[i] = [];
        for (j=0;j<4;j++) {
            if(i==j)
				mat[i][j] = 1;
			else
				mat[i][j] = 0;
        }
    }
	mat[0][0] = Math.cos(theta);
	mat[0][2] = Math.sin(theta);
	mat[2][0] = -Math.sin(theta);
	mat[2][2] = Math.cos(theta);
	
	return mat;
}

function generate_rotation_matrix_Z (theta){
	var mat = [];
    var i,j;

    for (i=0;i<4;i++) {
        mat[i] = [];
        for (j=0;j<4;j++) {
            if(i==j)
				mat[i][j] = 1;
			else
				mat[i][j] = 0;
        }
    }
	mat[0][0] = Math.cos(theta);
	mat[0][1] = -Math.sin(theta);
	mat[1][0] = Math.sin(theta);
	mat[1][1] = Math.cos(theta);
	
	return mat;
}

function matrix_pseudoinverse(mat) {
    var n = mat.length;
	var m = mat[0].length;
	var pseudoInv = [];

	if(n>m){			// (A_t * A)_inv * A_t
		pseudoInv = matrix_multiply(numeric.inv(matrix_multiply(matrix_transpose(mat),mat)),matrix_transpose(mat));
	}
	else if (n<m){		// A_t * (A * A_t)_inv
		pseudoInv = matrix_multiply(matrix_transpose(mat),numeric.inv(matrix_multiply(mat,matrix_transpose(mat))));
	}
	else {
		pseudoInv = numeric.inv(mat);
	}
	return pseudoInv;
}

function homogenize (v) {
	// input is an array of 3 elements
	// output is a 4X1 matrix 
	var vec = [ [v[0]], [v[1]], [v[2]], [1] ];
	return vec;
}

function vectorize (m) {
	// input is nX1 matrix
	// output is array of size n
	var vec = [];
	for (var i=0; i<m.length; i++){
		vec[i] = m[i][0];
	}
	return vec;
}

function matricize (v) {
	// input is an array of size n
	// output is an nX1 matrix
	var mat = [];
	for (var i=0; i<v.length; i++){
		mat[i] = [v[i]];
	}
	return mat;
}

function vector_subtract (v1,v2) {
	if(v1.length != v2.length)
		console.log("vector_subtract: vectors of different size");
	var n = v1.length;
	var vec = [];
	for (var i=0; i<n; i++){
		vec[i] = v1[i] - v2[i];
	}
	return vec;
}

function vector_add (v1,v2) {
	if(v1.length != v2.length)
		console.log("vector_subtract: vectors of different size");
	var n = v1.length;
	var vec = [];
	for (var i=0; i<n; i++){
		vec[i] = v1[i] + v2[i];
	}
	return vec;
}

function vector_reverse (v){
	var n = v.length;
	var vec = [];
	for (var i=0; i<n; i++){
		vec[i] = v[n-1-i];
	}
	return vec;
}

function extract_rotation_matrix (m) {
	var rot = 	[
				[ m[0][0] , m[0][1] , m[0][2] ],
				[ m[1][0] , m[1][1] , m[1][2] ],
				[ m[2][0] , m[2][1] , m[2][2] ]
				];
	return rot;
}

function extract_translation_vector (m) {
	var trans = [ m[0][3], m[1][3], m[2][3] ];
	return trans;
}