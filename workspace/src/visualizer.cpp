// Note: Only one tree required, and no graph search required.
// e.g., efficiency isn't important here, only the requirement that
// the path to the goal will actually happen, eventually. We can assure
// this by drawing a path directly to the goal every so often and seeing what happens.

// CURRENT ISSUES:
// -No ROS/Stage integration. This was large on its own, I didn't have time over the 6 days of work. I have an algorithm written for it on paper, and suspect it wouldn't take long,  but ran out of time to implement it.
// -The drawPartialLine() function sometimes (infrequently in my testing?) inverts and draws the wrong portions of the line. I'm unsure how/why.
// This is not an issue with the RRT algorithm, which does grow the tree correctly, just the visualization -- you'll notice they all converge on the correct point. I suspect this has something
// to do with a line colliding with more than one wall, and my infamiliarity with SDL meant I spent hours on this, but couldn't find the cause prior to submission.
// -The first line of a run draws much faster than the other 5, since it's just an attempted straight shot to the goal -- this isn't a glitch, it just updates instantly, by comparison to the others. 

#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <string>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <vector>

#define RAND_MAX 2147483647
using namespace std;

//Screen dimensions
const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;

//Window and rendering surfae
SDL_Surface* gScreenSurface = NULL;
SDL_Window* gWindow = NULL;

// Forward declaration
bool sdlInit(string filename);
bool sdlEnd();
void putPixel(int startx, int starty, Uint32 pixel);
Uint32 getPixel(int x, int y);
void bhm_line(int x1, int y1, int x2, int y2);
void printVector();
void drawLine();
void drawPartialLine();
bool loadSurface(string filePath);
bool checkLine();

vector< pair<int, int> > current;
vector< pair<int, int> > total;
vector< pair<int, int> > tryPoints;
pair<int, int> currcoord;

int main()
{
	int startx, starty, goalx, goaly;
	string filename;
	srand(time(NULL));
	cout << "What pixel (x y) should represent the start location?" << endl;
	cin >> startx >> starty;
	cout << "What pixel (x y) should represent the finish location?" << endl;
	cin >> goalx >> goaly;
	cout << "Okay! What is the file path of the map you want to use?" << endl;
	cin >> filename;
	cout << "Trying to open image..." << endl;
	if (sdlInit(filename) == false)
	{
		return 1;
	}
	Uint32 blue = SDL_MapRGB(gScreenSurface->format, 0, 0, 255);
	// Mark start pixel on image
	putPixel(startx, starty, blue); 
	SDL_UpdateWindowSurface(gWindow);
	cout << "Alright! Pixel " << startx << ", " << starty << " has been marked as the robot's starting point..." << endl;
	// Mark finish pixel on image
	putPixel(goalx, goaly, blue);
	SDL_UpdateWindowSurface(gWindow);
	cout << "...and " << goalx << ", " << goaly << " has been marked as the robot's intended destination!" << endl;
	cout << "Simulating..." << endl;

	// main event loop starts here!

	// Set current coordinate to starting position
	currcoord.first = startx;
	currcoord.second = starty;
	// Create placeholders for random x/y, an internal index, and distances
	int x, y, index, d1, d2;

	// Add starting coordinate to total path
	total.push_back(currcoord);

	while(true)
	{	
		cout << "Top of while loop!" << endl;
		// Check for line to finish. This will populate the current vector.
		bhm_line(currcoord.first, currcoord.second, goalx, goaly);

		if (checkLine())
		{
			// Got line to goal. Works.
			cout << "Whoop! Found goal!" << endl;
			drawLine();
			currcoord.first = goalx;
			currcoord.second = goaly;
			total.push_back(currcoord);
			break;
		}
		// Couldn't get there, Need to clear current but keep current position...
		drawPartialLine(); // Should draw green line to closest wall to final
		usleep(400000); // Temporarily pause simulation to see current cycle
		current.clear(); // Clear out current points

		// Generate 5 random points
		for (int i = 0; i < 5; i++)
		{
			pair <int, int> tmp;
			x = rand() % SCREEN_WIDTH;
			y = rand() % SCREEN_HEIGHT;

			while (x == currcoord.first && y == currcoord.second)
			{
				x = rand() % SCREEN_WIDTH;
				y = rand() % SCREEN_HEIGHT;
			}

			tmp.first = x;
			tmp.second = y;

			tryPoints.push_back(tmp);
		}

		// Difference 1: Total distance (x and y) from the goal of the 0th element of tryPoints
		d1 = abs(goalx - tryPoints[0].first) + abs(goaly - tryPoints[0].second);

		//Check to see which gets you closest
		for (int i = 1; i < 5; i++)
		{
			cout << "At beginning of for loop" << endl;
			// Get highest remaining one...
			for (int j = 0; j < i; j++)
			{
				//d2: difference of x/y wherever j is currently!
				d2 = abs(goalx - tryPoints[j].first) + abs(goalx - tryPoints[j].second);
				// If the current point is closer...
				if (d2 < d1)
				{
					// Then the current point should be preferred.
					index = j;
					// Update d1 to match!
					d1 = d2;
				}
			}
			
			// Check to see if the current one works!
			bhm_line(currcoord.first, currcoord.second, tryPoints[index].first, tryPoints[index].second);
			
			// If we could draw a line to the point...
			if (checkLine())
			{
				cout << "Whoop! Start while loop again!" << "Destination X: " << tryPoints[index].first << " Y: " << tryPoints[index].second << endl;
				// Draw the full line, since we can get to the point!
				drawLine();
				// Update current coordinates to reflect the new position...
				currcoord.first = tryPoints[index].first;
				currcoord.second = tryPoints[index].second;
				// And add that coordinate to the vector of total path taken to end.
				total.push_back(currcoord);
				// Break out of the loop, don't test outdated points
				break;
			}
			cout << "Back to the beginning of for loop. current: " << currcoord.first << ", " << currcoord.second << ". index: " << index << " Current size: " << current.size() << endl;
			// We couldn't draw a line to the point. Draw green line up to black pixels.
			drawPartialLine();
			// Remove current index from vector
			tryPoints.erase(tryPoints.begin() + index);
			current.clear();
			// spin
		}

		cout << "Hit end of while loop." << endl;
	
		// Current iteration is done. Clear out the trial points...
		tryPoints.clear();
		// ...and  
		current.clear();
		index = 0;
		d1 = 0;
		d2 = 0;
	}

	usleep(900000);

	sdlEnd();

	return 0;
}

bool sdlInit(string filename){
	//Init flag
	bool success = true;

	//Initialize SDL
	if (SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		printf("SDL Initialization Error: %s\n", SDL_GetError());
	}
	else
	{
		//Create window
		gWindow = SDL_CreateWindow("Visualizer", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
		if (gWindow == NULL)
		{
			printf("Error initializing window: %s\n", SDL_GetError());
		}
		else
		{
			//Begin loading PNG
			int imgFlags = IMG_INIT_PNG;
			if ( !(IMG_Init(imgFlags) & imgFlags))
			{
				printf("SDL_Image didn't initialize! Error: %s \n", IMG_GetError());
			}
			else
			{
				//Get surface
				gScreenSurface = SDL_GetWindowSurface(gWindow);

				if(!loadSurface(filename))
				{
					return 1;
				}

				//Update surface
				SDL_UpdateWindowSurface(gWindow);

				//Wait 2 secs
				SDL_Delay(2000);
			}
		}
	}

	return success;
}			

bool sdlEnd()
{
	SDL_DestroyWindow(gWindow);
	SDL_Quit();
}

void printVector()
{
	for (int i = 0; i < current.size(); i++)
	{
		cout << "(" << current[i].first << "," << current[i].second << ")" << endl;
	}
}

bool checkLine(){
	Uint32 black = SDL_MapRGB(gScreenSurface->format, 0, 0, 0);
	for (int i = 0; i < current.size(); i++)
	{	
		if (black == getPixel(current[i].first, current[i].second))
		{
			return false;
		}
	}
	return true;
}

void drawLine(){
	Uint32 blue = SDL_MapRGB(gScreenSurface->format, 0, 0, 255);
	for (int i = 0; i < current.size(); i++)
	{
		putPixel(current[i].first, current[i].second, blue); 
	}
	SDL_UpdateWindowSurface(gWindow);

}

void drawPartialLine(){
	Uint32 green = SDL_MapRGB(gScreenSurface->format, 0, 255, 0);
	Uint32 black = SDL_MapRGB(gScreenSurface->format, 0, 0, 0);
	for (int i = 0; i < current.size(); i++)
	{
		if (black == getPixel(current[i].first, current[i].second))
		{
			SDL_UpdateWindowSurface(gWindow);
			return;
		}
		putPixel(current[i].first, current[i].second, green); 
	}
	SDL_UpdateWindowSurface(gWindow);
}

bool loadSurface(string filePath)
{
	//Final optimized image surface
	SDL_Surface* optimizedSurface = NULL;

	//Load image at specified path
	SDL_Surface* loadedSurface = IMG_Load(filePath.c_str());
	if(loadedSurface == NULL)
	{
		printf( "Unable to load image %s! SDL_image Error: %s\n", filePath.c_str(), IMG_GetError());
		return false;
	}
	//Convert surface to screen format
	optimizedSurface = SDL_ConvertSurface(loadedSurface, gScreenSurface->format, NULL);
	if(optimizedSurface == NULL)
	{
		printf("Couldn't optimize image %s! Error: %s\n", filePath.c_str(), SDL_GetError());
	}

	SDL_FreeSurface(loadedSurface);

	SDL_BlitSurface(optimizedSurface, NULL, gScreenSurface, NULL);

	SDL_FreeSurface(optimizedSurface);

	return true;
}

Uint32 getPixel(int x, int y)
{
	 if(SDL_MUSTLOCK(gScreenSurface))
	 {
		 SDL_LockSurface(gScreenSurface);
	 }

	// 32-bit pixel conversion
	Uint32 *pixels = (Uint32 *)gScreenSurface->pixels;
	Uint32 desiredPixel = pixels[(y * gScreenSurface->w ) + x];

	if(SDL_MUSTLOCK(gScreenSurface))
	{
		SDL_UnlockSurface(gScreenSurface);
	}

	return desiredPixel; // Note: 0 is black, 255 is white
}

void putPixel(int x, int y, Uint32 pixel)
{
	int bpp = gScreenSurface->format->BytesPerPixel;

	// Address to pixel we want to set
	Uint8 *p = (Uint8 *)gScreenSurface->pixels + y * gScreenSurface->pitch + x * bpp;

	if(SDL_MUSTLOCK(gScreenSurface))
	{
		SDL_LockSurface(gScreenSurface);
 	}

	switch (bpp) {
	case 1:
		*p = pixel;
		break;

	case 2:
		*(Uint16 *)p = pixel;
		break;

	case 3:
		if (SDL_BYTEORDER == SDL_BIG_ENDIAN)
		{
			p[0] = (pixel >> 16) & 0xff;
            		p[1] = (pixel >> 8) & 0xff;
            		p[2] = pixel & 0xff;
        	} else {
            		p[0] = pixel & 0xff;
            		p[1] = (pixel >> 8) & 0xff;
            		p[2] = (pixel >> 16) & 0xff;
        	}
        	break;

    	case 4:
        	*(Uint32 *)p = pixel;
        	break;
    	}

	if(SDL_MUSTLOCK(gScreenSurface))
	{
		SDL_UnlockSurface(gScreenSurface);
	}
}

// Note: some of this taken from etchplanet
void bhm_line(int x1,int y1,int x2,int y2)
{
	 int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;
	 pair<int, int> xy;
	 dx=x2-x1;
 	 dy=y2-y1;
 	 dx1=abs(dx);
	 dy1=abs(dy);
	 px=2*dy1-dx1;
	 py=2*dx1-dy1;
	 if(dy1<=dx1)
	 {
		  if(dx>=0)
		  {
			   x=x1;
			   y=y1;
			   xe=x2;
		  }
		  else
		  {
			   x=x2;
			   y=y2;
			   xe=x1;
		  }
		  xy.first = x;
		  xy.second = y;
		  current.push_back(xy);
		  for(i=0;x<xe;i++)
		  {
			   x=x+1;
			   if(px<0)
			   {
				px=px+2*dy1;
			   }
			   else
			   {
				if((dx<0 && dy<0) || (dx>0 && dy>0))
				{
					 y=y+1;
				}
				else
				{
					 y=y-1;
				}
				px=px+2*(dy1-dx1);
			   }
		  	   xy.first = x;
		  	   xy.second = y;
		  	   current.push_back(xy);
		  }
	 }
	 else
	 {
		  if(dy>=0)
		  {
			   x=x1;
			   y=y1;
			   ye=y2;
		  }
		  else
		  {
			   x=x2;
			   y=y2;
			   ye=y1;
		  }
		  xy.first = x;
		  xy.second = y;
		  current.push_back(xy);
		  for(i=0;y<ye;i++)
		  {
			   y=y+1;
			   if(py<=0)
			   {
				 py=py+2*dx1;
			   }
		   	   else
		   	   {
			   	    if((dx<0 && dy<0) || (dx>0 && dy>0))
			    	    {
				    	 x=x+1;
				    }
				    else
				    {
					 x=x-1;
				    }
			   	    py=py+2*(dx1-dy1);
   	           }	
		   xy.first = x;
		   xy.second = y;
		   current.push_back(xy);
   		   }
	 }
}
