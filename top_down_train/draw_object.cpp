//g++ -lglut -lGL -lGLU sample3Dcube2.cpp 

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <cctype>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <cstdlib>
#include <queue>
#include "grid_datastructure.cpp"

#define LIMIT 20000


using namespace std; 

int angy = 0;
int angz = 0;
int angx = 0;

GLdouble scalex, scaley, scalez;
double pt_scx, pt_scy, pt_scz;
double pt_trx, pt_try, pt_trz;    

double	Ex = 0,    // 1717 ,
		Ey = 0, // 50.5 ,
		Ez = 800, // 734.3,
			
		Ax = 0, //-525.2 ,
		Ay = 0, //111.1,
		Az = -1, //-10,
				  				
		Ux = 0.0,
		Uy = 1.0, //-1.0,
		Uz = 0.0;

char toChange = 'e';
char mode = 'n' ;

bool show_cloud = false;	
bool step_wise = false;

int l,w,h;
int x,y,z;

int n, current_n, box_no;

int grid_length, grid_width, grid_height;

GLdouble scale;

class Point_3D{
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


class cuboid{
	
	public:
	vector< vector<GLdouble> > faces[6];
	GLdouble r,g,b;
	GLdouble al;
};

vector<cuboid> c_list;
cuboid gr;

void SpecialKeyPressed(GLint key, GLint x, GLint y) ;
void KeyPressed (unsigned char key, int x, int y);

vector<Block> list_of_blocks;
queue<int> tree_q;

void init() 
{
	glEnable(GL_DEPTH_TEST);
	glClearColor (0.1, 0.1, 0.1, 0.0);
	glShadeModel (GL_SMOOTH);
}

void set_cuboid_color(grid *g, cuboid &c, int x, int y, int z){

	if( (g->data[x][y][z]).used ){
		c.r = g->data[x][y][z].r;
		c.g = g->data[x][y][z].g;
		c.b = g->data[x][y][z].b;
	
		c.al = 1;
	}
	else{
		c.r = c.g = c.b = 0.0;
		c.al = 0;
	}
}



void set_cuboid_c(cuboid &c, int x, int y, int z, int l, int w, int h){
	
		/*
		c.r = (GLdouble)(rand()%100)/100.0f;
		c.g = (GLdouble)(rand()%100)/100.0f;
		c.b = (GLdouble)(rand()%100)/100.0f;
		c.al = 1;
		*/
		
		vector<GLdouble> t;
		
		
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

void draw_points(){
	
	double x, y, z;
	
	glColor4f(1.0,0.0,0.0,1);
	
	glBegin(GL_POINTS);
		for(int i = 0; i < cloud->size(); i++){
		
			x = (cloud->points[i].x - pt_trx) / pt_scx; 
			y = (cloud->points[i].y - pt_try) / pt_scy;
			z = (cloud->points[i].z - pt_trz) / pt_scz;
		
			glVertex3f(x*scalex, y*scaley, z*scalez);
		
		}
	glEnd();			
}


void draw_grid(){
	
	glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
	
	glBegin(GL_QUADS);
		
		glColor4f(0.0,1.0,0.0,1);
		
		for(int j = 0; j<2; j++)
		  {
			glVertex3f( gr.faces[j].at(0).at(0)*scalex, gr.faces[j].at(0).at(1)*scaley, gr.faces[j].at(0).at(2)*scalez );
	        glVertex3f( gr.faces[j].at(1).at(0)*scalex, gr.faces[j].at(1).at(1)*scaley, gr.faces[j].at(1).at(2)*scalez );
	        glVertex3f( gr.faces[j].at(2).at(0)*scalex, gr.faces[j].at(2).at(1)*scaley, gr.faces[j].at(2).at(2)*scalez );
	        glVertex3f( gr.faces[j].at(3).at(0)*scalex, gr.faces[j].at(3).at(1)*scaley, gr.faces[j].at(3).at(2)*scalez );
	      }		
	
		glColor4f(1.0,0.0,0.0,1);
		
		for(int j = 2; j<4; j++)
		  {
			glVertex3f( gr.faces[j].at(0).at(0)*scalex, gr.faces[j].at(0).at(1)*scaley, gr.faces[j].at(0).at(2)*scalez );
	        glVertex3f( gr.faces[j].at(1).at(0)*scalex, gr.faces[j].at(1).at(1)*scaley, gr.faces[j].at(1).at(2)*scalez );
	        glVertex3f( gr.faces[j].at(2).at(0)*scalex, gr.faces[j].at(2).at(1)*scaley, gr.faces[j].at(2).at(2)*scalez );
	        glVertex3f( gr.faces[j].at(3).at(0)*scalex, gr.faces[j].at(3).at(1)*scaley, gr.faces[j].at(3).at(2)*scalez );
	      }
	      
		
		glColor4f(0.0,0.0,1.0,1);	
		for(int j = 4; j<6; j++)
		  {
			glVertex3f( gr.faces[j].at(0).at(0)*scalex, gr.faces[j].at(0).at(1)*scaley, gr.faces[j].at(0).at(2)*scalez );
	        glVertex3f( gr.faces[j].at(1).at(0)*scalex, gr.faces[j].at(1).at(1)*scaley, gr.faces[j].at(1).at(2)*scalez );
	        glVertex3f( gr.faces[j].at(2).at(0)*scalex, gr.faces[j].at(2).at(1)*scaley, gr.faces[j].at(2).at(2)*scalez );
	        glVertex3f( gr.faces[j].at(3).at(0)*scalex, gr.faces[j].at(3).at(1)*scaley, gr.faces[j].at(3).at(2)*scalez );
	      }
	
	glEnd();
	
}


void display(void) 
{

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity ();            /* clear the matrix */
	
	
	gluLookAt (Ex, Ey, Ez,\
			   Ax,Ay,Az,\
			   Ux,Uy,Uz\
			   );
	
	//cout<<"\n"<<Ex<<" "<<Ey<<" "<<Ez<<" "<<Ax<<" "<<Ay<<" "<<Az<<endl;			
	glColor3f(0.4,0.4,0.4);

	glPushMatrix();
  //glScalef(1,1,1);
  
	// glTranslatef(grid_length*scalex/(double)2, grid_width*scaley/(double)2, grid_height*scalez/(double)2);
  
	glRotatef(angx,1,0,0);
	glRotatef(angy,0,1,0);
	glRotatef(angz,0,0,1);
  
	glTranslatef(-grid_length*scalex/(double)2, -grid_width*scaley/(double)2, -grid_height*scalez/(double)2);
  
	draw_grid();
	
	double alpha = 1.0;
	
	if(show_cloud){
		draw_points();
		alpha = 0.5;
	}	
	
	glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
  
	glBegin(GL_QUADS);               
      
		  if(step_wise){
			
			for(int i = 0; i <= box_no; i++){
				if(!show_cloud)
					glColor4f(c_list[i].r, c_list[i].g, c_list[i].b, c_list[i].al * alpha);
				else
					glColor4f(1.0, 1.0, 1.0, alpha);	
				
				for(int j = 0; j<6; j++)
				{
					glVertex3f( c_list[i].faces[j].at(0).at(0)*scalex, c_list[i].faces[j].at(0).at(1)*scaley, c_list[i].faces[j].at(0).at(2)*scalez );
					glVertex3f( c_list[i].faces[j].at(1).at(0)*scalex, c_list[i].faces[j].at(1).at(1)*scaley, c_list[i].faces[j].at(1).at(2)*scalez );
					glVertex3f( c_list[i].faces[j].at(2).at(0)*scalex, c_list[i].faces[j].at(2).at(1)*scaley, c_list[i].faces[j].at(2).at(2)*scalez );
					glVertex3f( c_list[i].faces[j].at(3).at(0)*scalex, c_list[i].faces[j].at(3).at(1)*scaley, c_list[i].faces[j].at(3).at(2)*scalez );
				}
		  
			}
		  }			  	
		  else{	
			for(int i = 0; i < c_list.size(); i++){
				if(!show_cloud)
					glColor4f(c_list[i].r, c_list[i].g, c_list[i].b, c_list[i].al * alpha);
				else
					glColor4f(1.0, 1.0, 1.0, alpha);	
				
				for(int j = 0; j<6; j++)
				{
					glVertex3f( c_list[i].faces[j].at(0).at(0)*scalex, c_list[i].faces[j].at(0).at(1)*scaley, c_list[i].faces[j].at(0).at(2)*scalez );
					glVertex3f( c_list[i].faces[j].at(1).at(0)*scalex, c_list[i].faces[j].at(1).at(1)*scaley, c_list[i].faces[j].at(1).at(2)*scalez );
					glVertex3f( c_list[i].faces[j].at(2).at(0)*scalex, c_list[i].faces[j].at(2).at(1)*scaley, c_list[i].faces[j].at(2).at(2)*scalez );
					glVertex3f( c_list[i].faces[j].at(3).at(0)*scalex, c_list[i].faces[j].at(3).at(1)*scaley, c_list[i].faces[j].at(3).at(2)*scalez );
				}			  	
      
			}
		}	 	
     glEnd();  // End of drawing color-cube
 
   glPopMatrix();

   glutSwapBuffers();
}


void reshape(int w, int h)
{
  if(h == 0) h = 1; 	//divide by zero

  float ratio = 1.0f * w / h;
  
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  
  glViewport(0, 0, w, h);
  gluPerspective(20, ratio, 5, 5000);
  
  glMatrixMode(GL_MODELVIEW);
  
  glLoadIdentity();
}

void set_next_level(){
	
		cuboid c;
		c_list.clear();
		int curr_size = tree_q.size();
		
		for(int i = 0; i < curr_size; i++){
			int t = tree_q.front();
			
			//cerr<<"The value of t: "<<t<<"\n";
			
			if(list_of_blocks[2*t].length != 0)
				tree_q.push(2*t);
				
			if(list_of_blocks[2*t + 1].length != 0)
				tree_q.push(2*t + 1);	
		
			if(list_of_blocks[2*t].length != 0 || list_of_blocks[2*t + 1].length != 0)
				tree_q.pop();
			else{
				int j = tree_q.front();
				tree_q.pop();
				tree_q.push(j);
			}
		
		}
			
		n = box_no = tree_q.size();
				
		for(int i = 0; i < n; i++){	
			
			x = list_of_blocks[ tree_q.front() ].x;  y = list_of_blocks[ tree_q.front() ].y;  z = list_of_blocks[ tree_q.front() ].z;
			l = list_of_blocks[ tree_q.front() ].length;  w = list_of_blocks[ tree_q.front() ].width;  h = list_of_blocks[ tree_q.front() ].height;
	
			c = cuboid();
			set_cuboid_c(c, x, y, z, l, w, h);	
			c_list.push_back(c);
		
			int temp = tree_q.front();
			tree_q.pop();
			
			tree_q.push(temp);
			
		}
	

}

void initialise_parameters(grid *g, double resolution, GLdouble scale_i, vector<Block> &block_list, int length, int width, int height, pcl::PointXYZ min, pcl::PointXYZ max){

	cuboid c;
	scale = scale_i;
	
	scalex = scaley = scalez = 1.0;
	
	scalex = scalex * scale ;
	scaley = scaley * scale ;
	scalez = scalez * scale ;	
	
	grid_length = length;	grid_width = width;	grid_height = height;
	
	set_cuboid_c(gr, 0, 0, 0, grid_length, grid_width, grid_height);
	
	list_of_blocks = block_list;
	
	n = box_no = block_list.size();
		
	for(int i=0; i<n; i++){
		x = block_list[i].x * 3;  y = block_list[i].y * 3;  z = block_list[i].z * 3;
		l = block_list[i].length * 3;  w = block_list[i].width * 3;  h = block_list[i].height * 3;
	
		cerr<<x<<" "<<y<<" "<<z<<"   "<<l<<" "<<w<<" "<<h<<"\n";
		int v_count = 0;		
		
		for(int j = x; j < x + l; j++){
			for(int k = y; k < y + w; k++){
				for(int m = z; m < z + h; m++){
			
					c = cuboid();
					set_cuboid_c(c, j, k, m, 1, 1, 1);	
					set_cuboid_color(g, c, j, k, m);
					c_list.push_back(c);
				
					v_count++;
				}
			}		
		}	
	
		//cerr<<"V Count: "<<v_count<<"\n";
	}
	
	cerr<<"C_LIST size: "<<c_list.size()<<"\n";
	
	
	/*
	tree_q.push(1);
	
	x = block_list[1].x;  y = block_list[1].y;  z = block_list[1].z;
	l = block_list[1].length;  w = block_list[1].width;  h = block_list[1].height;
	
	c = cuboid();
	set_cuboid_c(c, x, y, z, l, w, h);	
	c_list.push_back(c);
		
	n = box_no = 1; 
	*/
	
	double x_diff = max.x - min.x;
	double y_diff = max.y - min.y;
	double z_diff = max.z - min.z;
	
	pt_scx = resolution;
	pt_scy = resolution;
	pt_scz = resolution;

	pt_trx = min.x;
	pt_try = min.y;
	pt_trz = min.z;

}	


int draw_main(int argc,char* argv[])
{
	// Opengl Initialisations //
	
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_RGBA);
	
	glutInitWindowPosition(100,100);

	glutInitWindowSize(1000,1000);
	glutCreateWindow("Draw Cube"); 
    init();
	
	glEnable( GL_DEPTH_TEST );	
	glEnable( GL_BLEND );
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	
	
	
	glutDisplayFunc(display);  
	glutKeyboardFunc(&KeyPressed);
	glutSpecialFunc(&SpecialKeyPressed);
	glutReshapeFunc(reshape);
	
	// End of Initialisations  //
	
	current_n = 0;		
	
	glutMainLoop();
}


char key;

void KeyPressed (unsigned char key, int x, int y)
{
	key = tolower(key);
	if(key == 27)
		exit(0);
	
	switch(key)
	{
			case 'i':
					Ex = -380; Ey = 200; Ez = -262;
					Ax = 121; Ay = -282; Az = 342;
					Ux = 0; Uy = -1; Uz = 0;
					// mode = 'i';
					break;
			case 'e': //eye
			case 'a': //lookat
			case 'u': //up
					  toChange = key;
					  break;
			case 'p': show_cloud = !show_cloud;
					  break;	
			case 'n':
					box_no = (box_no + 1) % n;
					break;
			case 's':
					step_wise = !step_wise;
					box_no = 0;
					break;
			case 'r':
					set_next_level();
					break;
			case ',':
				angy = (angy - 11 + 360)%360 ;	
				glutPostRedisplay();
				break;
			case '.':
				angy = (angy + 11)%360 ;	
				glutPostRedisplay();
				break;
				
			case '[':
				angz = (angz - 11 + 360)%360 ;	
				glutPostRedisplay();
				break;
			case ']':
				angz = (angz + 10)%360 ;	
				glutPostRedisplay();
				break;
				
			case 'l':
				angx = (angx - 11 + 360)%360 ;	
				glutPostRedisplay();
				break;
			case ';':
				angx = (angx + 11)%360 ;	
				glutPostRedisplay();
				break;	
				
			default :
					break;
					
			
		}
   glutPostRedisplay();
}

void SpecialKeyPressed(GLint key, GLint x, GLint y) 
{
 double xpos = 0.0,
	    ypos = 0.0,
	    zpos = 0.0;

 switch (key)
      {   
       case GLUT_KEY_UP:
							ypos += 10.1; 
							break;
       case GLUT_KEY_DOWN:
							ypos -= 10.1; 
							break;
       case GLUT_KEY_LEFT:
							xpos -= 10.1;  
							break;
       case GLUT_KEY_RIGHT:
							xpos += 10.1;                
							break;
       case GLUT_KEY_PAGE_DOWN:
							zpos -= 10.1;
	                    	break;
       case GLUT_KEY_PAGE_UP:
							zpos+=10.1;
	                	    break;
       
      }	
      
      switch(toChange)
      {
		  case 'e':
		  case 'E':
					Ex += xpos;
					Ey += ypos;
					Ez += zpos;
					break;
		   case 'a':
		   case 'A':
					Ax += xpos;
					Ay += ypos;
					Az += zpos;
					break;
		   case 'u':
		   case 'U':
					Ux += xpos;
					Uy += ypos;
					Uz += zpos;
					break;			
		}
      glutPostRedisplay();
}
