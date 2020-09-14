/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | Graph Search Methods

    Stencil methods for student implementation of graph search algorithms.

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

function initSearchGraph() {

    // create the search queue
    visit_queue = [];

    // initialize search graph as 2D array over configuration space
    //   of 2D locations with specified spatial resolution
    G = [];
    for (iind=0,xpos=-2;xpos<7;iind++,xpos+=eps) {
        G[iind] = [];
        for (jind=0,ypos=-2;ypos<7;jind++,ypos+=eps) {
            G[iind][jind] = {
                i:iind,j:jind, // mapping to graph array
                x:xpos,y:ypos, // mapping to map coordinates
                parent:null, // pointer to parent in graph along motion path
                distance:10000, // distance to start via path through parent
                visited:false, // flag for whether the node has been visited
                priority:null, // visit priority based on fscore
                queued:false // flag for whether the node has been queued for visiting
            };

            // STENCIL: determine whether this graph node should be the start
            //   point for the search
			if(xpos>=q_init[0]-eps/2 && xpos<q_init[0]+eps/2 && ypos>=q_init[1]-eps/2 && ypos<q_init[1]+eps/2){
				startNode = G[iind][jind];
				startNode.distance = 0;
				startNode.priority = heuristicDistance(startNode);
				startNode.queued = true;
				visit_queue.push(startNode);
				draw_2D_configuration([xpos,ypos],"queued");
			}
        }
    }
}

function iterateGraphSearch() {


    // STENCIL: implement a single iteration of a graph search algorithm
    //   for A-star (or DFS, BFS, Greedy Best-First)
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "iterating" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   drawHighlightedPathGraph - draws a path back to the start location
    //   draw_2D_configuration - draws a square at a given location
	
	if(visit_queue.length==0){
		search_iterate = false;
		return "failed";
	}
	
	var currNode = minheap_extract(visit_queue);
	if(currNode.visited == true)
		return "iterating";	
	currNode.visited = true;
	draw_2D_configuration([currNode.x,currNode.y],"visited");
	search_visited++;
	
	if(currNode.x>=q_goal[0]-eps/2 && currNode.x<q_goal[0]+eps/2 && currNode.y>=q_goal[1]-eps/2 && currNode.y<q_goal[1]+eps/2){
		drawHighlightedPathGraph(currNode);
		search_iterate = false;
		return "succeeded";
	}		
	
	var nbrRelativeX = [-1,0,0,1];
	var nbrRelativeY = [0,-1,1,0];
	var numNbrs = 4;
	for(var nbrIndex=0; nbrIndex<numNbrs; nbrIndex++){
		var nbrNode = G[currNode.i+nbrRelativeX[nbrIndex]][currNode.j+nbrRelativeY[nbrIndex]];
		var nbrNodePos = [nbrNode.x,nbrNode.y];
		if (nbrNode.visited==false && testCollision(nbrNodePos)==false && nbrNode.distance>currNode.distance+dist(nbrNode,currNode)){
			nbrNode.distance = currNode.distance + dist(nbrNode,currNode);
			nbrNode.priority = nbrNode.distance + heuristicDistance(nbrNode);
			nbrNode.parent = currNode;
			nbrNode.queued = true;
			minheap_insert(visit_queue,nbrNode);
			draw_2D_configuration(nbrNodePos,"queued");
		}
	}		
	return "iterating";
}

function iterateGreedyBF() {
	
	if(visit_queue.length==0){
		search_iterate = false;
		return "failed";
	}
	
	var currNode = minheap_extract(visit_queue);
	if(currNode.visited == true)
		return "iterating";	
	currNode.visited = true;
	draw_2D_configuration([currNode.x,currNode.y],"visited");
	search_visited++;
	
	if(currNode.x>=q_goal[0]-eps/2 && currNode.x<q_goal[0]+eps/2 && currNode.y>=q_goal[1]-eps/2 && currNode.y<q_goal[1]+eps/2){
		drawHighlightedPathGraph(currNode);
		search_iterate = false;
		return "succeeded";
	}		
	
	
	var nbrRelativeX = [-1,0,0,1];
	var nbrRelativeY = [0,-1,1,0];
	var numNbrs = 4;
	for(var nbrIndex=0; nbrIndex<numNbrs; nbrIndex++){
		var nbrNode = G[currNode.i+nbrRelativeX[nbrIndex]][currNode.j+nbrRelativeY[nbrIndex]];
		var nbrNodePos = [nbrNode.x,nbrNode.y];
		if (nbrNode.visited==false && testCollision(nbrNodePos)==false && nbrNode.distance>currNode.distance+dist(nbrNode,currNode)){
			nbrNode.distance = currNode.distance + dist(nbrNode,currNode);
			nbrNode.priority = heuristicDistance(nbrNode);
			nbrNode.parent = currNode;
			nbrNode.queued = true;
			minheap_insert(visit_queue,nbrNode);
			draw_2D_configuration(nbrNodePos,"queued");
		}
	}		
	return "iterating";
}


function iterateBFS() {
	
	if(visit_queue.length==0){
		search_iterate = false;
		return "failed";
	}
	
	var currNode = visit_queue.shift();
	if(currNode.visited == true)
		return "iterating";	
	currNode.visited = true;
	draw_2D_configuration([currNode.x,currNode.y],"visited");
	search_visited++;
	
	if(currNode.x>=q_goal[0]-eps/2 && currNode.x<q_goal[0]+eps/2 && currNode.y>=q_goal[1]-eps/2 && currNode.y<q_goal[1]+eps/2){
		drawHighlightedPathGraph(currNode);
		search_iterate = false;
		return "succeeded";
	}		
	
	
	var nbrRelativeX = [-1,0,0,1];
	var nbrRelativeY = [0,-1,1,0];
	var numNbrs = 4;
	for(var nbrIndex=0; nbrIndex<numNbrs; nbrIndex++){
		var nbrNode = G[currNode.i+nbrRelativeX[nbrIndex]][currNode.j+nbrRelativeY[nbrIndex]];
		var nbrNodePos = [nbrNode.x,nbrNode.y];
		if (nbrNode.visited==false && testCollision(nbrNodePos)==false && nbrNode.distance>currNode.distance+eps){
			nbrNode.distance = currNode.distance + eps;
			nbrNode.parent = currNode;
			nbrNode.queued = true;
			visit_queue.push(nbrNode);
			draw_2D_configuration(nbrNodePos,"queued");
		}
	}		
	return "iterating";	

}

function iterateDFS() {
	
	if(visit_queue.length==0){
		search_iterate = false;
		return "failed";
	}
	
	var currNode = visit_queue.pop();
	if(currNode.visited == true)
		return "iterating";	
	currNode.visited = true;
	draw_2D_configuration([currNode.x,currNode.y],"visited");
	search_visited++;
	
	if(currNode.x>=q_goal[0]-eps/2 && currNode.x<q_goal[0]+eps/2 && currNode.y>=q_goal[1]-eps/2 && currNode.y<q_goal[1]+eps/2){
		drawHighlightedPathGraph(currNode);
		search_iterate = false;
		return "succeeded";
	}		
	
	
	var nbrRelativeX = [-1,0,0,1];
	var nbrRelativeY = [0,-1,1,0];
	var numNbrs = 4;
	for(var nbrIndex=0; nbrIndex<numNbrs; nbrIndex++){
		var nbrNode = G[currNode.i+nbrRelativeX[nbrIndex]][currNode.j+nbrRelativeY[nbrIndex]];
		var nbrNodePos = [nbrNode.x,nbrNode.y];
		if (nbrNode.visited==false && testCollision(nbrNodePos)==false && nbrNode.distance>currNode.distance+eps){
			nbrNode.distance = currNode.distance + eps;
			nbrNode.parent = currNode;
			nbrNode.queued = true;
			visit_queue.push(nbrNode);
			draw_2D_configuration(nbrNodePos,"queued");
		}
	}		
	return "iterating";	
}

//////////////////////////////////////////////////
/////     HELPER FUNCTIONS
//////////////////////////////////////////////////

// function to return distance between nodes
function dist(node1,node2){
	return Math.sqrt((node1.x-node2.x)*(node1.x-node2.x) + (node1.y-node2.y)*(node1.y-node2.y));
}

function heuristicDistance(node){
	return Math.sqrt((node.x-q_goal[0])*(node.x-q_goal[0]) + (node.y-q_goal[1])*(node.y-q_goal[1]));
}

//////////////////////////////////////////////////
/////     MIN HEAP IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement min heap functions for graph search priority queue.
    //   These functions work use the 'priority' field for elements in graph.


// defining 'heapifyTopDown' function for minheap_extract
function heapifyTopDown (heap, i){
	var min_idx = i;
	var left = 2*i+1;
	var right = 2*i+2;
	if(left<heap.length && heap[left].priority < heap[min_idx].priority)
		min_idx = left;
	if(right<heap.length && heap[right].priority < heap[min_idx].priority)
		min_idx = right;
	if(min_idx!=i){
		var temp = heap[i];
		heap[i] = heap[min_idx];
		heap[min_idx] = temp;
		heapifyTopDown(heap,min_idx);
	}
}

// defining 'heapifyTopDown' function for minheap_insert
function heapifyBottomUp (heap, i){
	var parent = Math.floor((i-1)/2);
	if(parent>=0 && heap[i].priority < heap[parent].priority){
		var temp = heap[i];
		heap[i] = heap[parent];
		heap[parent] = temp;
		heapifyBottomUp(heap,parent);
	}
}

// define insert function for min binary heap
function minheap_insert(heap, new_element) {
	heap.push(new_element);
	heapifyBottomUp(heap,heap.length-1);
}

// define extract function for min binary heap
function minheap_extract(heap) {
	var topElem = heap[0];
	heap[0] = heap[heap.length-1];
	heap.pop();
	heapifyTopDown(heap,0);
	return topElem;
}
