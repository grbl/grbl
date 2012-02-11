#include "SDL/SDL.h"
#include "SDL/SDL_ttf.h"
#include "SDL/SDL_image.h"
#include <pthread.h>

#include "stdbool.h"

#define FPS 20
#define PlayerPaddleY (SCREEN_HEIGHT - (SCREEN_HEIGHT / 10))
#define CompPaddleY (SCREEN_HEIGHT / 10)
/*SDL Includes. If your include paths are different, change these accordingly.*/

#define SCREEN_WIDTH 640     
#define SCREEN_HEIGHT 480 
#define COLOR_DEPTH 16
#define POINTSIZE 20
/*Defines that make it easy for us to change the screen properties without even changing the functions.*/

#define COLORKEY 255, 0, 255 
/*The colorkey that will not be included in sprites when they are blitted onto the screen.*/ 

/*******************************DECLARATIONS*************************************************/
enum textquality {solid=1, shaded, blended};

void drawimage(SDL_Surface *srcimg, int sx, int sy, int sw, int sh, SDL_Surface *dstimg, int dx, int dy, int alpha);
SDL_Surface* loadimage(char *file);
SDL_Surface* drawtext(TTF_Font *fonttodraw, char fgR, char fgG, char fgB, char fgA, 
		char bgR, char bgG, char bgB, char bgA, char text[], enum textquality quality);
TTF_Font* loadfont(char *file, int ptsize);

SDL_Surface* screen;

SDL_Surface* loadimage(char *file) {
	/*This function takes a path to an image file and returns a pointer to the loaded SDL_Surface to be saved into 
	  a variable.*/
	SDL_Surface *tmp;                        
	tmp = IMG_Load(file);            

	if (tmp == NULL) {                   
		printf("Error: File could not be opened: %s %s", file, SDL_GetError());
		exit(1);
	} 
	else {                               
		if(SDL_SetColorKey(tmp, SDL_SRCCOLORKEY | SDL_RLEACCEL, SDL_MapRGB(tmp->format, COLORKEY)) == -1)
			printf("Warning: Colorkey will not be used, reason: %s \n", SDL_GetError());
		/*If there are no errors loading the image into tmp, set the color key of the surface, so that our chosen 
		  transparent color will be transparent.*/
	}
	return tmp;
}

void drawimage(SDL_Surface *srcimg, int sx, int sy, int sw, int sh, SDL_Surface *dstimg, int dx, int dy, int alpha ) {

	if ((!srcimg) || (alpha == 0)) return; 
	/*If there's no image, or its 100% transparent.*/

	SDL_Rect src, dst;                     
	/*The two rectangles are filled with the information passed to the function.*/   
	src.x = sx;  src.y = sy;  src.w = sw;  src.h = sh;
	dst.x = dx;  dst.y = dy;  dst.w = src.w;  dst.h = src.h;

	if (alpha != 255) SDL_SetAlpha(srcimg, SDL_SRCALPHA, alpha); 
	/*Make SDL aware of the desired level of Alpha transparency in the source image.*/

	SDL_BlitSurface(srcimg, &src, dstimg, &dst);      
	/*Finally Blit the source on to the destination surface.*/
}

TTF_Font* loadfont(char *file, int ptsize)
{
	/*This function returns a loaded font to be saved into a variable. If the font cannot be loaded, the error is reported
	  and the application exits.
	  File is a string to the file
	  Ptsize is the size of the font in points*/
	TTF_Font* tmpfont;
	tmpfont = TTF_OpenFont(file, ptsize);
	if (tmpfont == NULL) {
		printf("Unable to load font: %s \n", TTF_GetError());
		exit(1);
	}
	return tmpfont;
}

SDL_Surface* drawtext(TTF_Font *fonttodraw, char fgR, char fgG, char fgB, char fgA, 
		char bgR, char bgG, char bgB, char bgA, char text[], enum textquality quality)
{
	/*This takes lots of parameters and uses them to render in the desired manner a string in the desired font to the desired
	  surface in the desired color .*/
	SDL_Color tmpfontcolor = {fgR,fgG,fgB,fgA};
	SDL_Color tmpfontbgcolor = {bgR, bgG, bgB, bgA};
	if (quality == solid){return TTF_RenderText_Solid(fonttodraw, text, tmpfontcolor);}
	if (quality == shaded){return TTF_RenderText_Shaded(fonttodraw, text, tmpfontcolor, tmpfontbgcolor);}
	if (quality == blended){return TTF_RenderText_Blended(fonttodraw, text, tmpfontcolor);}
	return 0;
}


void loadmedia();
void * mainloop(void *nop);
void GetInput();
void do_thinking();
void Draw();
void cleanup();

bool running;
bool gameovermode;

int PlayerPaddleX;
int PlayerPaddleVel;
int CompPaddleX;
int CompPaddleVel;

double cnc_head_x = 0;
double cnc_head_y = 0;

struct coord{
	int x;
	int y;
	int z;
}curr;

bool rightdown;
bool leftdown;
bool comprightdown;
bool compleftdown;
int widthoftext;
int heightoftext;

SDL_Surface* cnc_head;
TTF_Font* font;

int init_graphics(int argc, char *argv[])
{
	if (SDL_Init(SDL_INIT_VIDEO) != 0) {   
		printf("Unable to initialize SDL: %s \n", SDL_GetError());
		exit(1);
	}

	if (TTF_Init() == -1)
	{
		printf("Unable to initialize SDL_ttf:%s \n", TTF_GetError());
		exit(1);
	}  

	screen = SDL_SetVideoMode(SCREEN_WIDTH, SCREEN_HEIGHT, COLOR_DEPTH, SDL_DOUBLEBUF); //| SDL_FULLSCREEN);
	if (screen == NULL) {
		printf("Unable to set video mode: %s \n", SDL_GetError());
		exit(1);
	}

	SDL_ShowCursor(SDL_DISABLE); //remove cursor

	/*When the application exits, make sure everything is shutdown.*/
	atexit(cleanup);

	/*Load our media, like images and fonts*/
	loadmedia();

	/*Set some variables so our mainloop does the right thing*/
	running = true;
	gameovermode = true;

	/*Enter the main loop, and when we return, exit*/
	pthread_t graphics_thread;
	pthread_create( &graphics_thread, NULL, mainloop, NULL);
	return 0;
}

void loadmedia()
{
	cnc_head = loadimage("arch/simulator/media/cnc_head.bmp");
	font = loadfont("arch/simulator/media/FreeSans.ttf", POINTSIZE);
} 

void * mainloop(void *nop)
{
	curr.y = 0;
	curr.x = 0;

	int looptime = SDL_GetTicks();
	while (running) {
		GetInput();
		do_thinking();
		Draw();

		/*Delay the game according to the gamespeed constant so we don't just eat up all the CPU.*/
		while (SDL_GetTicks() - looptime < FPS) {
			SDL_Delay(10);
		}
		looptime = SDL_GetTicks();
	}
	return NULL;
}

void GetInput()
{
	SDL_Event event;
	while (SDL_PollEvent(&event)) {
		switch (event.type) {
			case SDL_QUIT:                 
				running = false;
				break;
			case SDL_KEYDOWN:
				switch (event.key.keysym.sym) {
					case SDLK_ESCAPE:
						running = false;
						break;
					case SDLK_RIGHT:
						rightdown = true;
						break;
					case SDLK_LEFT:
						leftdown = true;  
						break;
					default: 
						break;
				}
				break;
			case SDL_KEYUP:
				switch (event.key.keysym.sym) {
					case SDLK_RIGHT:
						rightdown = false;
						break;
					case SDLK_LEFT:
						leftdown = false;  
						break;        
					default:
						break;
				}
				break;
		} 
	}
}

void do_thinking()
{
	curr.x = (int)(cnc_head_x + 40.0);
	curr.y = (int)(cnc_head_y + 40.0);
//	printf("updated to x: %i, y: %i", curr.x,curr.y);
}

void Draw()
{
	SDL_Rect table;
	table.x = 20;
	table.y = 20;
	table.w = SCREEN_WIDTH-40;
	table.h= SCREEN_HEIGHT-40;
	SDL_FillRect(screen, 0, SDL_MapRGB(screen->format, 0, 0, 0));
	SDL_FillRect(screen, &table, SDL_MapRGB(screen->format, 255, 255, 255));

	drawimage(cnc_head, 0, 0, 4, 4, screen, curr.x, curr.y, 255);
	SDL_Flip(screen);  
}

void cleanup()
{
	/*Destroy our libraries and media so we don't leave bunches of junk in memory.*/
	SDL_FreeSurface(screen);
	SDL_FreeSurface(cnc_head);

	TTF_CloseFont(font);

	TTF_Quit();
	SDL_Quit();
}  

