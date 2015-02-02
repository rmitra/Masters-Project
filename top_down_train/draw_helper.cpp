class Point_3D
{
public:
	double x;
	double y;
	double z;

	Point_3D(double x_i, double y_i, double z_i){
		x = x_i;
		y = y_i;
		z = z_i;
	}

	Point_3D(){
		x = 0.0;
		y = 0.0;
		z = 0.0;
	}

	Point_3D(Point_3D &p){
		x = p.x;
		y = p.y;
		z = p.z;
	}
};


class cuboid
{
public:
	vector< vector<GLdouble> > faces[6];
	GLdouble r,g,b;
	GLdouble al;
	int min_dis;

	cuboid(){
		r = g = b = 1.0;
		al = 1.0;
		min_dis = 999999;
	}
};

vector<cuboid> c_list;
vector<cuboid> c_list_texture;
vector<cuboid> c_list_no_texture;

void set_cuboid_c(cuboid &c, int x, int y, int z, int l, int w, int h)
{
	/*
	c.r = (GLdouble)(rand()%100)/100.0f;
	c.g = (GLdouble)(rand()%100)/100.0f;
	c.b = (GLdouble)(rand()%100)/100.0f;
	c.al = 1;
	*/

	vector<GLdouble> t;
	for(int i = 0; i<6; i++)
		c.faces[i].clear();

	t.push_back(x); t.push_back(y); t.push_back(z);
	c.faces[0].push_back(t);
	t.clear();

	t.push_back(x+l); t.push_back(y); t.push_back(z);
	c.faces[0].push_back(t);
	t.clear();

	t.push_back(x+l); t.push_back(y+w); t.push_back(z);
	c.faces[0].push_back(t);
	t.clear();

	t.push_back(x); t.push_back(y+w); t.push_back(z);
	c.faces[0].push_back(t);
	t.clear();

	//############

	t.push_back(x); t.push_back(y); t.push_back(z+h);
	c.faces[1].push_back(t);
	t.clear();

	t.push_back(x+l); t.push_back(y); t.push_back(z+h);
	c.faces[1].push_back(t);
	t.clear();

	t.push_back(x+l); t.push_back(y+w); t.push_back(z+h);
	c.faces[1].push_back(t);
	t.clear();

	t.push_back(x); t.push_back(y+w); t.push_back(z+h);
	c.faces[1].push_back(t);
	t.clear();

	// ###################

	t.push_back(x); t.push_back(y); t.push_back(z);
	c.faces[2].push_back(t);
	t.clear();

	t.push_back(x); t.push_back(y); t.push_back(z+h);
	c.faces[2].push_back(t);
	t.clear();

	t.push_back(x); t.push_back(y+w); t.push_back(z+h);
	c.faces[2].push_back(t);
	t.clear();

	t.push_back(x); t.push_back(y+w); t.push_back(z);
	c.faces[2].push_back(t);
	t.clear();

	//########

	t.push_back(x+l); t.push_back(y); t.push_back(z);
	c.faces[3].push_back(t);
	t.clear();

	t.push_back(x+l); t.push_back(y); t.push_back(z+h);
	c.faces[3].push_back(t);
	t.clear();

	t.push_back(x+l); t.push_back(y+w); t.push_back(z+h);
	c.faces[3].push_back(t);
	t.clear();

	t.push_back(x+l); t.push_back(y+w); t.push_back(z);
	c.faces[3].push_back(t);
	t.clear();

	//#######

	t.push_back(x); t.push_back(y); t.push_back(z);
	c.faces[4].push_back(t);
	t.clear();

	t.push_back(x+l); t.push_back(y); t.push_back(z);
	c.faces[4].push_back(t);
	t.clear();

	t.push_back(x+l); t.push_back(y); t.push_back(z+h);
	c.faces[4].push_back(t);
	t.clear();

	t.push_back(x); t.push_back(y); t.push_back(z+h);
	c.faces[4].push_back(t);
	t.clear();

	//#######

	t.push_back(x); t.push_back(y+w); t.push_back(z);
	c.faces[5].push_back(t);
	t.clear();

	t.push_back(x+l); t.push_back(y+w); t.push_back(z);
	c.faces[5].push_back(t);
	t.clear();

	t.push_back(x+l); t.push_back(y+w); t.push_back(z+h);
	c.faces[5].push_back(t);
	t.clear();

	t.push_back(x); t.push_back(y+w); t.push_back(z+h);
	c.faces[5].push_back(t);
	t.clear();
}

int get_nearest_plane_index(int i, int j, int k, int x, int y, int z, int length, int width, int height, int &min_dis)
{
	int index = -1;

	if(k - z <= z + height - 1 - k){
		min_dis  = k - z;
		index = 1;
	}
	else{
		min_dis  = z + height - 1 - k;
		index = 2;
	}

	if(j - y < min_dis){
		min_dis  = j - y;
		index = 4;
	}

	if(y + width - 1 - j < min_dis){
		min_dis  = y + width - 1 - j;
		index = 3;
	}

	if(i - x < min_dis){
		min_dis  = i - x;
		index = 5;
	}

	if(x + length - 1 - i < min_dis){
		min_dis  = x + length - 1 - i;
		index = 6;
	}

	return index;
}

cuboid front[150][150];
cuboid back[150][150];
cuboid top[150][150];
cuboid bottom[150][150];
cuboid lt[150][150];
cuboid rt[150][150];

void white_wash(int l, int w, int h)
{
	for (int i = 0; i<= l; i++) {
		for (int j = 0; j<= w; j++) {
			front[i][j].r = front[i][j].g = front[i][j].b = 1.0;
			front[i][j].min_dis = 999999.0;

			back[i][j].r = back[i][j].g = back[i][j].b = 1.0;
			back[i][j].min_dis = 999999.0;
		}
	}

	for (int i = 0; i<= l; i++) {
		for (int j = 0; j<= h; j++) {

			top[i][j].r = top[i][j].g = top[i][j].b = 1.0;
			top[i][j].min_dis = 999999.0;

			bottom[i][j].r = bottom[i][j].g = bottom[i][j].b = 1.0;
			bottom[i][j].min_dis = 999999.0;
		}
	}

	for (int i = 0; i<= w; i++) {
		for (int j = 0; j<= h; j++) {

			lt[i][j].r = lt[i][j].g = lt[i][j].b = 1.0;
			lt[i][j].min_dis = 999999.0;

			rt[i][j].r = rt[i][j].g = rt[i][j].b = 1.0;
			rt[i][j].min_dis = 999999.0;
		}
	}
}

/*

void add_to_clist(grid * g_disp, int x, int y, int z, int l, int w, int h){

	white_wash();

	int hr, vt;
	int min_dis;

	for(int j = x; j < x + l && j < g_disp->length; j++){
		for(int k = y; k < y + w && k < g_disp->width; k++){
			for(int m = z; m < z + h && m < g_disp->height; m++){

				if(g_disp->data[j][k][m].used){

					int nearest_index = get_nearest_plane_index(j, k, m, x, y, z, l, w, h, min_dis);

					//if(nearest_index == 1 || nearest_index == 2)
						//cerr<<"Data Point Found at index "<<nearest_index<<"\n";

					switch(nearest_index){

						case 1: hr = (j - x), vt = (k - y);
								if(front[hr][vt].min_dis > min_dis){
									front[hr][vt].r = g_disp->data[j][k][m].r;
									front[hr][vt].g = g_disp->data[j][k][m].g;
									front[hr][vt].b = g_disp->data[j][k][m].b;
								}
								break;
						case 2: hr = (j - x), vt = (k - y);
								if(back[hr][vt].min_dis > min_dis){
									back[hr][vt].r = g_disp->data[j][k][m].r;
									back[hr][vt].g = g_disp->data[j][k][m].g;
									back[hr][vt].b = g_disp->data[j][k][m].b;
								}
								break;
						case 3: hr = (j - x), vt = (m - z);
								if(top[hr][vt].min_dis > min_dis){
									top[hr][vt].r = g_disp->data[j][k][m].r;
									top[hr][vt].g = g_disp->data[j][k][m].g;
									top[hr][vt].b = g_disp->data[j][k][m].b;
								}
								break;
						case 4: hr = (j - x), vt = (m - z);
								if(bottom[hr][vt].min_dis > min_dis){
									back[hr][vt].r = g_disp->data[j][k][m].r;
									back[hr][vt].g = g_disp->data[j][k][m].g;
									back[hr][vt].b = g_disp->data[j][k][m].b;
								}
								break;
						case 5: hr = (k - y), vt = (m - z);
								if(lt[hr][vt].min_dis > min_dis){
									lt[hr][vt].r = g_disp->data[j][k][m].r;
									lt[hr][vt].g = g_disp->data[j][k][m].g;
									lt[hr][vt].b = g_disp->data[j][k][m].b;
								}
								break;
						case 6: hr = (k - y), vt = (m - z);
								if(rt[hr][vt].min_dis > min_dis){
									rt[hr][vt].r = g_disp->data[j][k][m].r;
									rt[hr][vt].g = g_disp->data[j][k][m].g;
									rt[hr][vt].b = g_disp->data[j][k][m].b;
								}
								break;
					}


				}

			}

		}
	}

	for(int i = x; i < x + l && i < g_disp->length; i++ ){
		for(int j = y; j < y + w && j < g_disp->width; j++ ){
				set_cuboid_c(front[i-x][j-y], i, j, z, 1, 1, 1);
				c_list_texture.push_back(front[i-x][j-y]);
				set_cuboid_c(back[i-x][j-y], i, j, z + h - 1, 1, 1, 1);
				c_list_texture.push_back(back[i-x][j-y]);
				//cerr<<"IIIIIII\n";
		}
	}

	for(int i = x; i < x + l && i < g_disp->length; i++ ){
		for(int j = z; j < z + h && j < g_disp->height; j++ ){
				set_cuboid_c(bottom[i-x][j-z], i, y, j, 1, 1, 1);
				c_list_texture.push_back(bottom[i-x][j-z]);
				set_cuboid_c(top[i-x][j-z], i, y + w -1, j, 1, 1, 1);
				c_list_texture.push_back(top[i-x][j-z]);
		}
	}

	for(int i = y; i < y + w && i < g_disp->width; i++ ){
		for(int j = z; j < z + h && j < g_disp->height; j++ ){
				set_cuboid_c(lt[i-y][j-z], x, i, j, 1, 1, 1);
				c_list_texture.push_back(lt[i-y][j-z]);
				set_cuboid_c(rt[i-y][j-z], x + l -1, i, j, 1, 1, 1);
				c_list_texture.push_back(rt[i-y][j-z]);
		}
	}

}

*/


void get_color(grid * g_disp, cuboid &c, int dir,  int _i, int _j, int start, int finish, bool reverse)
{
	int *x_index, *y_index, *z_index;
	int k;

	if (reverse) {
		int temp = start;
		start = finish;
		finish = temp;
	}

	if (dir == 1) {
		x_index = &_i;
		y_index = &_j;
		z_index = &k;
	}
	else if (dir == 2) {
		x_index = &_i;
		z_index = &_j;
		y_index = &k;
	}
	else {
		y_index = &_i;
		z_index = &_j;
		x_index = &k;
	}

	k = start;

	if (!reverse) {
		for (; k < finish; k++) {
			if (g_disp->data[*x_index][*y_index][*z_index].used) {
				c.r = g_disp->data[*x_index][*y_index][*z_index].r;
				c.g = g_disp->data[*x_index][*y_index][*z_index].g;
				c.b = g_disp->data[*x_index][*y_index][*z_index].b;

				break;
			}
		}
	}
	else {
		k--;
		for (; k >= finish; k--) {
			if (g_disp->data[*x_index][*y_index][*z_index].used) {
				c.r = g_disp->data[*x_index][*y_index][*z_index].r;
				c.g = g_disp->data[*x_index][*y_index][*z_index].g;
				c.b = g_disp->data[*x_index][*y_index][*z_index].b;

				break;
			}
		}
	}
}


void add_to_clist(grid * g_disp, int x, int y, int z, int l, int w, int h)
{
	white_wash(l, w, h);

	for (int i = x; i < x + l && i < g_disp->length; i++) {
		for (int j = y; j < y + w && j < g_disp->width; j++) {
			get_color(g_disp, front[i-x][j-y], 1, i, j, z, z + h, false);
			get_color(g_disp, back[i-x][j-y], 1, i, j, z, z + h, true);

			set_cuboid_c(front[i-x][j-y], i, j, z, 1, 1, 1);
			c_list_texture.push_back(front[i-x][j-y]);
			set_cuboid_c(back[i-x][j-y], i, j, z + h - 1, 1, 1, 1);
			c_list_texture.push_back(back[i-x][j-y]);

		}
	}

	for (int i = x; i < x + l && i < g_disp->length; i++) {
		for (int j = z; j < z + h && j < g_disp->height; j++) {
			get_color(g_disp, bottom[i-x][j-z], 2, i, j, y, y + w, false);
			get_color(g_disp, top[i-x][j-z], 2, i, j, y, y + w, true);

			set_cuboid_c(bottom[i-x][j-z], i, y, j, 1, 1, 1);
			c_list_texture.push_back(bottom[i-x][j-z]);
			set_cuboid_c(top[i-x][j-z], i, y + w -1, j, 1, 1, 1);
			c_list_texture.push_back(top[i-x][j-z]);

		}
	}

	for (int i = y; i < y + w && i < g_disp->width; i++) {
		for (int j = z; j < z + h && j < g_disp->height; j++) {
			get_color(g_disp, lt[i-y][j-z], 3, i, j, x, x + l, false);
			get_color(g_disp, rt[i-y][j-z], 3, i, j, x, x + l, true);

			set_cuboid_c(lt[i-y][j-z], x, i, j, 1, 1, 1);
			c_list_texture.push_back(lt[i-y][j-z]);
			set_cuboid_c(rt[i-y][j-z], x + l -1, i, j, 1, 1, 1);
			c_list_texture.push_back(rt[i-y][j-z]);

		}
	}

}
