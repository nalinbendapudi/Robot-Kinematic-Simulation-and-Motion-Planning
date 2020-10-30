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


// STENCIL: reference matrix code has the following functions:
//   matrix_multiply
//   matrix_transpose
//   matrix_pseudoinverse
//   matrix_invert_affine
//   vector_normalize
//   vector_cross
//   generate_identity
//   generate_translation_matrix
//   generate_rotation_matrix_X
//   generate_rotation_matrix_Y
//   generate_rotation_matrix_Z



// **** Function stencils are provided below, please uncomment and implement them ****//



// function matrix_pseudoinverse(m) {
//     // returns pseudoinverse of matrix m

// }

// function matrix_invert_affine(m) {
//     // returns 2D array that is the invert affine of 4-by-4 matrix m

// }



	
function matrix_multiply (m1,m2) {
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
