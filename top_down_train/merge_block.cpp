
#include <vector>
#include <list>
#include <utility>


#include <string>
#include <cstring>
#include <sys/types.h>
#include <cstdlib>
#include <sys/wait.h>
#include <unistd.h>

using namespace std;

bool find_compatible_x(Block i, Block j){

	// for x direction +ve

	if( i.x + i.length == j.x && i.y == j.y && i.z == j.z)
	{
		if( i.width == j.width && i.height == j.height)
			return true;
		else
			return false;

	}
	return false;
}

bool find_compatible_y(Block i, Block j){

	if( i.y + i.width == j.y && i.x == j.x && i.z == j.z)
	{
		if( i.length == j.length && i.height == j.height)
			return true;
		else
			return false;

	}
	return false;
}

int find_compatible_z(Block i, Block j){

	if( i.z + i.height == j.z && i.y == j.y && i.x == j.x )
	{
		if( i.length == j.length && i.width == j.width)
			return true;
		else
			return false;

	}
	return false;
}

Block find_min_block(Block i, Block j, int cases){

	Block t;

	// for merge in x

	if(cases == 1)
	{
		if( i.x < j.x )
			t = Block(i.x, i.y, i.z, i.length + j.length, i.width, i.height);
		else
			t = Block(j.x, j.y, j.z, i.length + j.length, i.width, i.height);
	}

	// for merge in y

	if(cases == 2)
	{
		if( i.y < j.y )
			t = Block(i.x, i.y, i.z, i.length, i.width + j.width, i.height);
		else
			t = Block(j.x, j.y, j.z, j.length, i.width + j.width, i.height);

	}

    // for merge in z

    if(cases == 3)
	{
		if( i.z < j.z )
			t = Block(i.x, i.y, i.z, i.length, i.width, i.height + j.height);
		else
			t = Block(j.x, j.y, j.z, j.length, i.width + j.width, i.height);

	}

	return t;
}

void merge_Block( vector<Block> &list ){

	bool completed = false;

	while(!completed){

		completed = true;
		bool restart = false;

		for(int i = 0; i < list.size(); i++){
			for(int j = 0; j < list.size(); j++){

				if(i == j)
					continue;

				if( find_compatible_x( list[i], list[j] ) ){
					completed = false;
					restart = true;

					list.push_back( find_min_block( list[i], list[j], 1));

					if( i < j)
					{
						list.erase(list.begin() + j);
						list.erase(list.begin() + i);
					}
					else{
						list.erase(list.begin() + i);
						list.erase(list.begin() + j);
					}
					break;
				}
				else if( find_compatible_y( list[i], list[j] ) ){
					completed = false;
					restart = true;

					list.push_back( find_min_block( list[i], list[j], 2) );

					if( i < j)
					{
						list.erase(list.begin() + j);
						list.erase(list.begin() + i);
					}
					else{
						list.erase(list.begin() + i);
						list.erase(list.begin() + j);
					}

					break;
				}
				else if ( find_compatible_z( list[i], list[j] ) ){
					completed = false;
					restart = true;

					list.push_back( find_min_block( list[i], list[j], 3) );

					if( i < j)
					{
						list.erase(list.begin() + j);
						list.erase(list.begin() + i);
					}
					else{
						list.erase(list.begin() + i);
						list.erase(list.begin() + j);
					}

					break;
				}


			}

			if(restart)
				break;

		}

	}

}


