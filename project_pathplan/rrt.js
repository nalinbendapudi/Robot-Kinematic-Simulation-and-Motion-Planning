/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | RRT Methods

    Stencil methods for student implementation of RRT-based search.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see search_canvas.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

function iterateRRT() {


    // STENCIL: implement a single iteration of an RRT algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree
}

function iterateRRTConnect() {


    // STENCIL: implement a single iteration of an RRT-Connect algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree
	
	var result = "failed";
	
	var q_rand = randomConfig();
	if (extendRRT(T_a, q_rand) != "trapped"){
		if (connectRRT(T_b, T_a.vertices[T_a.newest].vertex) == "reached") {
			search_iterate = false;
			drawHighlightedPath(dfsPath(T_a));
			drawHighlightedPath(dfsPath(T_b));
			return "succeeded";
		}
		return "extended";
	}
	
	// swapping trees T_a and T_b and repeating
	
	var q_rand = randomConfig();
	if (extendRRT(T_b, q_rand) != "trapped"){
		if (connectRRT(T_a, T_b.vertices[T_b.newest].vertex) == "reached") {
			search_iterate = false;
			drawHighlightedPath(dfsPath(T_a));
			drawHighlightedPath(dfsPath(T_b));
			return "succeeded";
		}
		return "extended";
	}
	
	return result;
}

function iterateRRTStar() {
	
}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement RRT-Connect functions here, such as:
    //   extendRRT
    //   connectRRT
    //   randomConfig
    //   newConfig
    //   findNearestNeighbor
    //   dfsPath
	

function randomConfig(){
	x_width = 6;
	y_height = 6;
	x0 = -1;
	y0 = -1;
	
	x = x0 + Math.random() * x_width;
	y = y0 + Math.random() * y_height;
	
	return [x,y];
}

function newConfig (q_near, q_rand) {
	var delta_q = [q_rand[0]-q_near[0], q_rand[1]-q_near[1]];
	var dist  = Math.pow ( Math.pow(delta_q[0],2) + Math.pow(delta_q[1],2) , 0.5);
	// eps is defined in infrastrucutre.js, eps=0.1
	if (dist>eps){
		var q_new   = [ q_near[0] + eps*delta_q[0]/dist , q_near[1] + eps*delta_q[1]/dist ];
	}
	else{
		var q_new = [q_rand[0],q_rand[1]];
	}
	if (testCollision(q_new)){
		return false;
	}
	return q_new;
}

function distance (q1, q2) {
	var delta_q = [q1[0]-q2[0], q1[1]-q2[1]];
	return Math.pow ( Math.pow(delta_q[0],2) + Math.pow(delta_q[1],2) , 0.5);
}

function findNearestNeighbor (tree, q_rand) {
	var dist_nearest = Number.POSITIVE_INFINITY;
    var nearest_idx = 0;
    for (var i=0; i<tree.vertices.length; i++) {
        var node = tree.vertices[i]
		var dist_to_rand = distance(node.vertex, q_rand);
        if (dist_to_rand < dist_nearest) {
            dist_nearest = dist_to_rand;
            q_near = node.vertex;
            nearest_idx = i;
        }
    }
    return [nearest_idx, q_near];
}

function extendRRT (tree, q_rand) {
	var q_near_info = findNearestNeighbor(tree, q_rand);
	var q_near = q_near_info[1];
	var q_near_idx = q_near_info[0];
	var q_new = newConfig(q_near, q_rand);
	if (q_new === false){
		return "trapped";
	}
	else{
		insertTreeVertex(tree, q_new);
		insertTreeEdge(tree, q_near_idx, tree.newest);
		if(q_new[0] == q_rand[0] && q_new[1] == q_rand[1]) {
			return "reached";
		}
		else{
			return "advanced";
		}
	}
}

function connectRRT (tree, q) {
	extendResult = "advanced";
	while(extendResult=="advanced"){
		extendResult = extendRRT(tree,q);
	}
	return extendResult;
}

function dfsPath(tree) {
    var path = [];
    var curr = tree.vertices[tree.newest];

    while (curr !== tree.vertices[0]) {
        path.push(curr);
        curr = curr.edges[0];
    }
    path.push(curr);
    return path;
}
