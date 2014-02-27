/* Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above notice and this permission notice shall be included in all copies
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/* File for "Terrain" lesson of the OpenGL tutorial on
 * www.videotutorialsrock.com
 */



#include <iostream>
#include <stdlib.h>
#include<cstdio>
#include<cstring>
#include<cmath>
#include<vector>
#ifdef __APPLE__
#include <OpenGL/OpenGL.h>
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "imageloader.h"
#include "vec3f.h"
#define ESC 27
#define PI 3.141592653589
#define DEG2RAD(deg) (deg * PI / 180)
#define RAD2DEG(rad) (rad * 180 / PI)
using namespace std;


//float x = 0.0, y = -5.0; // initially 5 units south of origin
float deltaMove = 0.0; // initially camera doesn't move
float box_x=1.0  , box_y=2.0 , box_z=1.0;
bool jump=0;
bool brake=0;
// Camera direction
//float lx = 1.0, ly = 1.0; // camera points initially along y-axis
float angle = 90.0; // angle of rotation for the camera direction
//float deltaAngle = 0.0; // additional angle change when dragging
int flag=0;
float pitch=0.0f;
float roll=0;
float tyre_angle=0;
int cam_flag=1;
float vel=0;
float deltaRotate=0;
int score=0;
int ifrolled;
float avg_h,h1,h2;
float prevheight=-1;
float prevpitch=0;
float acc=0,ret=0;

vector < pair<int,int> > material;

//Represents a terrain, by storing a set of heights and normals at 2D locations
class Terrain {
	private:
		int w; //Width
		int l; //Length
		float** hs; //Heights
		Vec3f** normals;
		bool computedNormals; //Whether normals is up-to-date
	public:
		Terrain(int w2, int l2) {
			w = w2;
			l = l2;

			hs = new float*[l];
			for(int i = 0; i < l; i++) {
				hs[i] = new float[w];
			}

			normals = new Vec3f*[l];
			for(int i = 0; i < l; i++) {
				normals[i] = new Vec3f[w];
			}

			computedNormals = false;
		}

		~Terrain() {
			for(int i = 0; i < l; i++) {
				delete[] hs[i];
			}
			delete[] hs;

			for(int i = 0; i < l; i++) {
				delete[] normals[i];
			}
			delete[] normals;
		}

		int width() {
			return w;
		}

		int length() {
			return l;
		}

		//Sets the height at (x, z) to y
		void setHeight(int x, int z, float y) {
			hs[z][x] = y;
			computedNormals = false;
		}

		//Returns the height at (x, z)
		float getHeight(int x, int z) {
			return hs[z][x];
		}

		//Computes the normals, if they haven't been computed yet
		void computeNormals() {
			if (computedNormals) {
				return;
			}

			//Compute the rough version of the normals
			Vec3f** normals2 = new Vec3f*[l];
			for(int i = 0; i < l; i++) {
				normals2[i] = new Vec3f[w];
			}

			for(int z = 0; z < l; z++) {
				for(int x = 0; x < w; x++) {
					Vec3f sum(0.0f, 0.0f, 0.0f);

					Vec3f out;
					if (z > 0) {
						out = Vec3f(0.0f, hs[z - 1][x] - hs[z][x], -1.0f);
					}
					Vec3f in;
					if (z < l - 1) {
						in = Vec3f(0.0f, hs[z + 1][x] - hs[z][x], 1.0f);
					}
					Vec3f left;
					if (x > 0) {
						left = Vec3f(-1.0f, hs[z][x - 1] - hs[z][x], 0.0f);
					}
					Vec3f right;
					if (x < w - 1) {
						right = Vec3f(1.0f, hs[z][x + 1] - hs[z][x], 0.0f);
					}

					if (x > 0 && z > 0) {
						sum += out.cross(left).normalize();
					}
					if (x > 0 && z < l - 1) {
						sum += left.cross(in).normalize();
					}
					if (x < w - 1 && z < l - 1) {
						sum += in.cross(right).normalize();
					}
					if (x < w - 1 && z > 0) {
						sum += right.cross(out).normalize();
					}

					normals2[z][x] = sum;
				}
			}

			//Smooth out the normals
			const float FALLOUT_RATIO = 0.5f;
			for(int z = 0; z < l; z++) {
				for(int x = 0; x < w; x++) {
					Vec3f sum = normals2[z][x];

					if (x > 0) {
						sum += normals2[z][x - 1] * FALLOUT_RATIO;
					}
					if (x < w - 1) {
						sum += normals2[z][x + 1] * FALLOUT_RATIO;
					}
					if (z > 0) {
						sum += normals2[z - 1][x] * FALLOUT_RATIO;
					}
					if (z < l - 1) {
						sum += normals2[z + 1][x] * FALLOUT_RATIO;
					}

					if (sum.magnitude() == 0) {
						sum = Vec3f(0.0f, 1.0f, 0.0f);
					}
					normals[z][x] = sum;
				}
			}

			for(int i = 0; i < l; i++) {
				delete[] normals2[i];
			}
			delete[] normals2;

			computedNormals = true;
		}

		//Returns the normal at (x, z)
		Vec3f getNormal(int x, int z) {
			if (!computedNormals) {
				computeNormals();
			}
			return normals[z][x];
		}
};

//Loads a terrain from a heightmap.  The heights of the terrain range from
//-height / 2 to height / 2.
Terrain* loadTerrain(const char* filename, float height) {
	Image* image = loadBMP(filename);
	Terrain* t = new Terrain(image->width, image->height);
	for(int y = 0; y < image->height; y++) {
		for(int x = 0; x < image->width; x++) {
			unsigned char color =
				(unsigned char)image->pixels[3 * (y * image->width + x)];
			float h = height * ((color / 255.0f) - 0.5f);
			if(h>0)
				t->setHeight(x, y, h);
			else

				t->setHeight(x, y, 0);
		}
	}

	delete image;
	t->computeNormals();
	return t;
}

float _angle = 60.0f;
Terrain* _terrain;

void cleanup() {
	delete _terrain;
}

void handleKeypress(unsigned char key, int x, int y) {
	if (key == ESC || key == 'q' || key == 'Q') exit(0);
	if(key == '1') cam_flag=1;
	if(key == '2') cam_flag=2;
	if(key == '3') cam_flag=3;
	if(key == '4') cam_flag=4;
	if(key == '5') cam_flag=5;
}

void initRendering() {
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_NORMALIZE);
	glShadeModel(GL_SMOOTH);
}

float mag(float a,float b,float c)
{
    return sqrt((a*a)+(b*b)+(c*c));
}


void handleResize(int w, int h) {
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (double)w / (double)h, 1.0, 200.0);
}

float height(Terrain* terrain, float x, float z) {
    //Make (x, z) lie within the bounds of the terrain
    if (x < 1) {
        x = 1;
    }
    else if (x > terrain->width() - 2) {
        x = terrain->width() - 2;
    }
    if (z < 1) {
        z = 1;
    }
    else if (z > terrain->length() - 2) {
        z = terrain->length() - 2;
    }

    //Compute the grid cell in which (x, z) lies and how close we are to the
    //left and outward edges
    int left_X = (int)x;
    if (left_X == terrain->width() - 1) {
        left_X--;
    }
    float frac_X = x - left_X;

    int out_Z = (int)z;
    if (out_Z == terrain->width() - 1) {
        out_Z--;
    }
    float frac_Z = z - out_Z;

    //Compute the four heights for the grid cell
    float h11 = terrain->getHeight(left_X, out_Z);
    float h12 = terrain->getHeight(left_X, out_Z + 1);
    float h21 = terrain->getHeight(left_X + 1, out_Z);
    float h22 = terrain->getHeight(left_X + 1, out_Z + 1);

    //Take a weighted average of the four heights
    return (1 - frac_X) * ((1 - frac_Z) * h11 + frac_Z * h12) +
        frac_X * ((1 - frac_Z) * h21 + frac_Z * h22);
}
void drawScene() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0.0,0.0,0.0);
	if(brake)
    {
        vel=0;
        ret=0;
        acc=0;
    }
    if(vel>=0 || pitch!=0) 
    {
        box_z+=vel*cosf(DEG2RAD(-(angle-90.0f)));
        box_x+=vel*sinf(DEG2RAD(-(angle-90.0f)));
    }
    else if(vel<0)
    {
       vel=0;
        acc=0;
        ret=0;
   }
    if(box_z<0)
        box_z=0;
    if(box_x<0)
        box_x=0;
    if(box_z>117)
        box_z=117;
    if(box_x>117)
        box_x=117;

    h1=height(_terrain,box_x,box_z);
    h2=h1;
    //cout<<prevheight<<" "<<h2<<" "<<jump<<"\n";
    if(jump==1)
        h1=prevheight-0.1;
    if(prevheight-(h2)>0.2)
        jump=1;
    else if(h1<0)
        jump=0;
    prevheight=h1;
    /*_terrain->getHeight((int)bike_x,(int)bike_z);
    h2=_terrain->getHeight(((int)bike_x)+1,(int)bike_z);
    h3=_terrain->getHeight((int)bike_x,((int)bike_z)+1);
    h4=_terrain->getHeight(((int)bike_x)+1,((int)bike_z)+1);
    h5=_terrain->getHeight(((int)bike_x)-1,((int)bike_z)-1);
    h6=_terrain->getHeight(((int)bike_x)-1,((int)bike_z));
    h7=_terrain->getHeight(((int)bike_x),((int)bike_z)-1);
    h8=_terrain->getHeight(((int)bike_x)-1,((int)bike_z)+1);
    h9=_terrain->getHeight(((int)bike_x)+1,((int)bike_z)-1);
    avg_h=((h2/8)+(h3/8)+(h4/8)+(h5/8)+(h6/8)+(h7/8)+(h8/8)+(h9/8));
    avg_h=(avg_h/2)+(h1/2);
    pitch=RAD2DEG(atan((avg_h-bike_y)/1.414));
    pitch+=10.0265;
    */
    Vec3f tnormal = _terrain->getNormal(box_x, box_z);
    //cout<<"normal\n";
   // cout<<tnormal[0]<<" "<<tnormal[1]<<" "<<tnormal[2]<<"\n";
    if(!jump)
    {
    float tryc,ab,ac,ad;//,ac,ab,ac;
   // mag=getmagnitude(bike_x,bike_y,bike_z);
    ab=sinf(DEG2RAD(angle));
    ac=0;//sinf(DEG2RAD(pitch));
    ad=cosf(DEG2RAD(angle));
    //ac=bike_x/mag;
    //ac=bike_x/mag;
    tryc=((tnormal[0]*ab+tnormal[2]*ad+tnormal[1]*ac)/(mag(tnormal[0],tnormal[1],tnormal[2])*mag(ab,ac,ad)));
    //cout<<(getmagnitude(tnormal[0],tnormal[1],tnormal[2]))<<"\n";
    //cout<<"bike look vector\n";
    //cout<<ab<<" "<<ad<<"\n";
    tryc=RAD2DEG(acos(tryc));
    pitch=-(90-tryc);
    }
    //if(tnormal[0]>0)
      //  pitch=-pitch;
   // else if(tnorn
   // cout<<pitch<<"\n";
   // cout<<"sinpitch "<<-sinf(DEG2RAD(pitch))<<"\n";
    box_y=h1+0.25;
	if(cam_flag==1)
	{
		gluLookAt(
				box_x-5*cos(DEG2RAD(angle)),      box_y+1.0,      box_z-5*sin(DEG2RAD(angle)),
				box_x, box_y, box_z,
				0.0,    1.0,    0.0);
	}
	if(cam_flag==2)
	{
		gluLookAt(
				box_x-7*cos(DEG2RAD(angle)),      box_y+5.0,      box_z-7*sin(DEG2RAD(angle)),
				box_x, box_y, box_z,
				0.0,    1.0,    0.0);
	}
	if(cam_flag==3)
	{
		gluLookAt(
				box_x+1*cos(DEG2RAD(angle)),      box_y,      box_z+1.0*sin(DEG2RAD(angle)),
				box_x+3*cos(DEG2RAD(angle)), box_y, box_z+3.0*sin(DEG2RAD(angle)),
				0.0,    1.0,    0.0);
	}
	if(cam_flag==4)
{
        
        gluLookAt(
                box_x,      box_y+10,      box_z,
                box_x+cos(DEG2RAD(angle)), box_y,box_z+sin(DEG2RAD(angle)),
                0.0,    1.0,    0.0
                );
    }
	if(cam_flag==5)
{
        
        gluLookAt(
                box_x-1.5,      box_y+0.25,      box_z-1.5,
                box_x, box_y+0.25,box_z,
                0.0,    1.0,    0.0
                );

    }
	//glTranslatef(0.0f, 0.0f, -10.0f);
	//glRotatef(30.0f, 1.0f, 0.0f, 0.0f);
	//glRotatef(-_angle, 0.0f, 1.0f, 0.0f);

	GLfloat ambientColor[] = {0.4f, 0.4f, 0.4f, 1.0f};
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientColor);

	GLfloat lightColor0[] = {0.6f, 0.6f, 0.6f, 1.0f};
	GLfloat lightPos0[] = {-0.5f, 0.8f, 0.1f, 0.0f};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor0);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos0);
	glPushMatrix();
	float scale = 100.0f / max(_terrain->width() - 1, _terrain->length() - 1);
	glScalef(scale, scale, scale);
	//glTranslatef(-(float)(_terrain->width() - 1) / 2,
	//		0.0f,
	//		-(float)(_terrain->length() - 1) / 2);
	glPopMatrix();
	glColor3f(0.3f, 0.6f, 0.0f);
	for(int z = 0; z < _terrain->length() - 1; z++) {
		//Makes OpenGL draw a triangle at every three consecutive vertices
		glBegin(GL_TRIANGLE_STRIP);
		for(int x = 0; x < _terrain->width(); x++) {
			Vec3f normal = _terrain->getNormal(x, z);
			glNormal3f(normal[0], normal[1], normal[2]);
			glVertex3f(x, _terrain->getHeight(x, z), z);
			normal = _terrain->getNormal(x, z + 1);
			glNormal3f(normal[0], normal[1], normal[2]);
			glVertex3f(x, _terrain->getHeight(x, z + 1), z + 1);
		}
		glEnd();
	}

	



	/*glColor3f(0.5, 0.5, 0.5);
	glPushMatrix();
	float y1,y2,y3,y4;
	//cout<<box_x<<" "<<box_z<<endl;*/
	if(box_x<=0) box_x=0;
	if(box_z<=0) box_z=0;
	if(box_z>=_terrain->length()-2) box_z=_terrain->length()-2;
	if(box_x>=_terrain->width()-2) box_x=_terrain->width()-2;
	/*y1=_terrain->getHeight(box_x, box_z);
	y2=_terrain->getHeight(box_x+0.4, box_z+0.4);
	y3=_terrain->getHeight(box_x+0.4, box_z);
	y4=_terrain->getHeight(box_x, box_z+0.4);
	box_y=((y1+y2+y3+y4)/8);
	glTranslatef(box_x,box_y,box_z);
	glRotatef(-(angle-90.0f), 0.0f, 1.0f, 0.0f);
	glutSolidCube(0.4);
	glPopMatrix();*/
	
    glColor3f(1.0,1.0,0.0); 
    glPushMatrix();
    glTranslatef(box_x,0,box_z);
    glRotatef(-(angle-90.0f),0,1,0);
    glRotatef(-pitch,1,0,0);
    glRotatef(roll,0,0,1);
    glTranslatef(0,box_y+0.05,0);//to keep it till ground
    glPushMatrix();
    glTranslatef(0.0,0.0,-0.125);
    glutSolidCube(0.25);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(0.0,0.0,0.125);
    glutSolidCube(0.25);
    glTranslatef(0.0,0.0,0.125);
    
    //headlight
    glPushMatrix();
    glColor3f(1.0,0.8f,1.0); 
    glTranslatef(0.0,0.175,0.0);
    glutSolidCone(0.05,0.1,20,20);
    glPopMatrix();
    
    //handle
    glPushMatrix();
    glColor3f(1.0,1.0,0.8); 
    glLineWidth(3);
    glBegin(GL_LINES);
    glVertex3f(0.125,0.125,0.0);
    glVertex3f(0.375,0.375,0.0);
    glVertex3f(-0.125,0.125,0.0);
    glVertex3f(-0.375,0.375,0.0);
    glEnd();
    glPopMatrix();
    
    glPopMatrix();


    tyre_angle+=vel/0.06;
   
    //back wheel
    glPushMatrix();
    glColor3f(1.0,1.0,1.0); 
    glTranslatef(0.0,-0.125,-0.375);
    glRotatef(-90,1,0,0);
    glRotatef(90,0,1,0);
    glRotatef(RAD2DEG(tyre_angle),0,0,1);//wheel rotation
    glutWireTorus(0.06,0.12,10,10);
    glPopMatrix();

    //front wheel
    glPushMatrix();
    glColor3f(1.0,1.0,1.0);
    glTranslatef(0.0,-0.125,0.375);
    glRotatef(-90,1,0,0);
    glRotatef(90,0,1,0);
    glRotatef(RAD2DEG(tyre_angle),0,0,1);//wheel rotation
    glutWireTorus(0.06,0.12,10,10);
    glPopMatrix();

    glPopMatrix();

	glPushMatrix(); 
	char text[11]; 
	sprintf(text,"Score: %d",score); 
	glTranslatef(box_x,0.6f,box_z ); 
	glColor3f(1.0f,0.0f,1.0f);
	glRasterPos2f(2.5f,0.0f); 
	for(unsigned int i=0;i<strlen(text);i++) 
	{
	glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24,text[i]); 
	}
	glPopMatrix();

    for(size_t k=0;k<material.size();k++)
    {
        glPushMatrix();
        glTranslatef(material[k].first,height(_terrain,material[k].first,material[k].second),material[k].second);
        glColor3f(0.4f, 0.0f, 0.8f);
        glutSolidCube(0.5f);
        glPopMatrix();
    }
  


	glutSwapBuffers();
}

float distance(float x,float y,float a,float b)
{
return sqrt((x-a)*(x-a)+(y-b)*(y-b));
} 
void update() {
	/*if (deltaMove) { // update camera position
		box_x += deltaMove * cos(DEG2RAD(angle))* 0.15;
		box_z += deltaMove * sin(DEG2RAD(angle)) * 0.15;
	}*/
	if(flag)
	{
		angle=angle+flag*5.0;
	}
 if(jump)
    {
    glutPostRedisplay(); // redisplay everything
    return;
    }
    if(prevpitch<0 && pitch==0)
    {
        ret=-0.005;
    }
    prevpitch=pitch;
    if(vel>0)
    vel=vel+acc+ret-(0.001*sinf(DEG2RAD(pitch)));
    else //if(vel==0)
    {
        ret=0;
       // 
        //if(pitch<0)
       // {
            vel=vel+acc-(0.001*sinf(DEG2RAD(pitch)));
      //  }
  //  else
    //vel=vel+acc;
    }
    if(deltaRotate==-1)
        angle+=5;
    else if(deltaRotate==1)
        angle-=5;
    if( -45< (roll+(5*ifrolled)) && roll+(5*ifrolled)<45)
        roll=roll+(5*ifrolled);
    if(ifrolled)
        if(vel!=0 && vel>0.05)
        {
            angle+=-0.7*tanf(DEG2RAD(roll))/vel;

        }

	glutPostRedisplay(); // redisplay everything
	for(size_t k=0;k<material.size();k++)
 	{
	//cout<<material[k].first<<" "<<material[k].second<<" "<<box_x<<" "<<box_z<<endl;
	//if((box_x<=material[k].first+0.1f)&&(box_x>=material[k].first-0.1f)&&(box_z<=material[k].second+0.1f)&&(box_x>=material[k].first-0.1f))
	if(distance(material[k].first,material[k].second,box_x,box_z)<0.5f)
	{material.erase(material.begin()+k);
	score++;
}
	
        }
}

void pressSpecialKey(int key, int xx, int yy)
{
	switch (key) {
		case GLUT_KEY_UP : deltaMove = 1.0; acc=0.002; break;
		case GLUT_KEY_DOWN : deltaMove = -1.0; brake=1; break;
		case GLUT_KEY_LEFT : flag=-1; ifrolled=-1; break;
		case GLUT_KEY_RIGHT : flag=1; ifrolled=1; break;
	}
} 

void releaseSpecialKey(int key, int x, int y) 
{
	switch (key) {

		case GLUT_KEY_UP : deltaMove = 0.0; ret=-0.005;
            acc=0; break;
		case GLUT_KEY_DOWN : deltaMove = 0.0; acc=0;
            brake=0; break;
		case GLUT_KEY_LEFT : flag=0; deltaRotate = 0.0; 
            roll=0;
            ifrolled=0;break;
		case GLUT_KEY_RIGHT : flag=0; deltaRotate = 0.0; 
            roll=0;
            ifrolled=0; break;
	}
} 


int main(int argc, char** argv) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(400, 400);
	
	glutCreateWindow("Terrain");
	initRendering();

	_terrain = loadTerrain("heightmap.bmp", 20);
	glutIgnoreKeyRepeat(1);
	glutDisplayFunc(drawScene);
	glutIdleFunc(update);
	glutKeyboardFunc(handleKeypress);
	glutReshapeFunc(handleResize);
	glutSpecialFunc(pressSpecialKey); // process special key pressed
	// Warning: Nonstandard function! Delete if desired.
	glutSpecialUpFunc(releaseSpecialKey);
	//glutTimerFunc(25, update, 0);
	glEnable(GL_DEPTH_TEST);
	
        for(int j=0;j<36;j++)
        {
            material.push_back(make_pair((rand()%58 + 1),(rand()%58 + 1)));
        }
	glutMainLoop();
	return 0;
}









