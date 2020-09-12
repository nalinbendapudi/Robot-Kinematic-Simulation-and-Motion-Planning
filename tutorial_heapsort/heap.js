/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object
minheaper = {};

// defining 'heapifyTopDown' function for minheap_extract
function heapifyTopDown (heap, i){
	min_idx = i;
	left = 2*i+1;
	right = 2*i+2;
	if(left<heap.length && heap[left] < heap[min_idx])
		min_idx = left;
	if(right<heap.length && heap[right] < heap[min_idx])
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
	parent = Math.floor((i-1)/2);
	if(parent>=0 && heap[i] < heap[parent]){
		var temp = heap[i];
		heap[i] = heap[parent];
		heap[parent] = temp;
		heapifyBottomUp(heap,parent);
	}
}

// define insert function for min binary heap
function minheap_insert(heap, new_element) {
	heap[heap.length] = new_element;
	heapifyBottomUp(heap,heap.length-1);
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {
	root = heap[0];
	heap[0] = heap[heap.length-1];
	heap.length--;
	heapifyTopDown(heap,0);
	return root;
}

// assign extract function within minheaper object
minheaper.extract = minheap_extract;
