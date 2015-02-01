#ifndef BLOCK_HPP
#define BLOCK_HPP

class Block
{
	public:
	int x;
	int y;
	int z;

	int length;
	int width;
	int height;

	Block(){
		x = y = z = 0;
		length = width = height = 1;
	}

	Block(const Block &b)
	{
		x = b.x;
		y = b.y;
		z = b.z;

		length = b.length;
		width = b.width;
		height = b.height;
	}

	Block(int x_i, int y_i, int z_i, int l_i, int w_i, int h_i )
	{
		x = x_i;
		y = y_i;
		z = z_i;

		length = l_i;
		width = w_i;
		height = h_i;
	}
};

#endif