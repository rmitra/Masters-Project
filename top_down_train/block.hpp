#ifndef BLOCK_HPP
#define BLOCK_HPP

// objects of the Block class represent cuboids in space
// The diagonal points of this cuboid are:
//   (x, y, z)
//   (x + length, y + width, z + height)
class Block
{
	public:
	int x;
	int y;
	int z;

	int length;
	int width;
	int height;

	Block()
	{
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

	void display_extents()
	{
        cout << "block extent x " << this->x << " " << (this->x + this->length) << endl;
        cout << "block extent y " << this->y << " " << (this->y + this->width) << endl;
        cout << "block extent z " << this->z << " " << (this->z + this->height) << endl;
	}

	// returns the center of gravity of a block given its extents
	// this is just the centroid of the block
	void find_center_of_gravity(double &x, double &y, double &z) {
	    x = this->x + (double)this->length/2;
	    y = this->y + (double)this->height/2;
	    z = this->z + (double)this->height/2;
	}

	// volume of the block
	// since we are assuming constant density, we can use this as the mass of the block
	inline long volume() { return length * width * height; }
};

#endif