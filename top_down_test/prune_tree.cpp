#include <queue>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <queue>
#include <stack>

#include "grid_datastructure.cpp"
#include "./gmm/gmm_main.cpp"

void rec_prune(vector<Block> &tree, int index, bool &no_children, int &no_merge){

	bool no_children_left = false;
	bool no_children_right = false;
	
	bool empty_right = false;
	bool empty_left = false;
	
	int no_left_merge = 0;
	int no_right_merge = 0;
	
	if(tree[2*index].length != 0 && tree[2*index].width != 0 && tree[2*index].height != 0){
		rec_prune(tree, 2*index, no_children_left, no_left_merge);
	}
	else{
		no_children_left = true;
		empty_left = true;
	}
		
	if(tree[2*index + 1].length != 0 && tree[2*index + 1].width != 0 && tree[2*index + 1].height != 0){
		rec_prune(tree, 2*index + 1, no_children_right, no_right_merge);
	}
	else{
		no_children_right = true;
		empty_right = true;
	}
			
	
	if(empty_left && empty_right){
		no_children = true;
		return;
	}
	
	no_merge = max(no_left_merge, no_right_merge);
	
	if(no_children_left == true && no_children_right == true){
		
		vector<double> x(3);
		x[0] = (double)tree[2*index].height / (double)tree[2*index].length;
		x[1] = (double)tree[2*index].height / (double)tree[2*index].width;
		x[2] = (double)tree[2*index].width / (double)tree[2*index].length;
		
		double left_prob = get_probability(x);
	
		x[0] = (double)tree[2*index + 1].height / (double)tree[2*index + 1].length;
		x[1] = (double)tree[2*index + 1].height / (double)tree[2*index + 1].width;
		x[2] = (double)tree[2*index + 1].width / (double)tree[2*index + 1].length;
		
		double right_prob = get_probability(x);
	
		x[0] = (double)tree[index].height / (double)tree[index].length;
		x[1] = (double)tree[index].height / (double)tree[index].width;
		x[2] = (double)tree[index].width / (double)tree[index].length;
	
		double parent_prob = get_probability(x);
	
		cerr<<"Parent prob: "<<parent_prob<<" left_prob :"<<left_prob<<" right_prob :"<<right_prob<<"\n";
		
		//if( parent_prob > ( left_prob + right_prob ) / 2.0 && no_merge < 1){
		  if( ( parent_prob >  left_prob  || parent_prob > right_prob ) && no_merge < 1){
			tree[2*index] = Block(0,0,0,0,0,0);
			tree[2*index + 1] = Block(0,0,0,0,0,0);
			no_children = true;
			
			no_merge++;
			cerr<<"Pruned Index :"<<index<<"\n";
			
			return;
		}
		else{
			no_children = false;
			return;
		}	
		
	}
	else{
		no_children = false;
		return;
	}	
	

}


vector<Block> prune_tree(vector<Block> &tree){

	int index = 1;
	
	if(tree[index].length == 0 && tree[index].width == 0 && tree[index].height == 0)
		return tree;
	bool dummy = true;
	int no_merge = 0;	
	
	rec_prune(tree, index, dummy, no_merge);

	vector<Block> final_decom;
	
	stack<Block> s;
	stack <int> i;
	
	index = 1;
	
	int max_index = -1;
	
	if(tree[index].length != 0 && tree[index].width != 0 && tree[index].height != 0){
		s.push(tree[index]);
		i.push(index);
	}
	
	while(!s.empty()){
		Block top = s.top();
		int top_index = i.top();
		
		if(max_index < top_index)
			max_index = top_index;
		
		s.pop();
		i.pop();
		
		if(tree[2 * top_index].length == 0 && tree[2 * top_index + 1].length == 0){
			final_decom.push_back(top);
				
		}
		else{
		
			s.push(tree[2 * top_index]);
			i.push(2 * top_index);
			s.push(tree[2 * top_index + 1]);
			i.push(2 * top_index + 1);
		}
		
	}


	cerr<<"Max tree index: "<<max_index<<"\n";
	
	return final_decom;

}
