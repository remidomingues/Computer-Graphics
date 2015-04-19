// TODO: Documentation!

#ifndef SDL_AUXILIARY_H
#define SDL_AUXILIARY_H

#include "SDL.h"
#include <iostream>
#include <glm/glm.hpp>

// Initializes SDL (video and timer). SDL creates a window where you can draw.
// A pointer to this SDL_Surface is returned. After calling this function
// you can use the function PutPixelSDL to do the actual drawing.
SDL_Surface* InitializeSDL( int width, int height, bool fullscreen = false );

// Checks all events/messages sent to the SDL program and returns true as long
// as no quit event has been received.
bool NoQuitMessageSDL();

// Draw a pixel on a SDL_Surface. The color is represented by a glm:vec3 which
// specifies the red, green and blue component with numbers between zero and
// one. Before calling this function you should call:
// if( SDL_MUSTLOCK( surface ) )
//     SDL_LockSurface( surface );
// After calling this function you should call:
// if( SDL_MUSTLOCK( surface ) )
//     SDL_UnlockSurface( surface );
// SDL_UpdateRect( surface, 0, 0, 0, 0 );
void PutPixelSDL( SDL_Surface* surface, int x, int y, glm::vec3 color );

SDL_Surface* InitializeSDL( int width, int height, bool fullscreen )
{
	if( SDL_Init( SDL_INIT_VIDEO | SDL_INIT_TIMER ) < 0 )
	{
		std::cout << "Could not init SDL: " << SDL_GetError() << std::endl;
		exit(1);
	}
	atexit( SDL_Quit );

	Uint32 flags = SDL_SWSURFACE;
	if( fullscreen )
		flags |= SDL_FULLSCREEN;

	SDL_Surface* surface = 0;
	surface = SDL_SetVideoMode( width, height, 32, flags );
	if( surface == 0 )
	{
		std::cout << "Could not set video mode: "
				  << SDL_GetError() << std::endl;
		exit(1);
	}
	return surface;
}

bool NoQuitMessageSDL()
{
	SDL_Event e;
	while( SDL_PollEvent(&e) )
	{
		if( e.type == SDL_QUIT )
			return false;
		if( e.type == SDL_KEYDOWN )
			if( e.key.keysym.sym == SDLK_ESCAPE)
				return false;
	}
	return true;
}

// TODO: Does this work on all platforms?
void PutPixelSDL( SDL_Surface* surface, int x, int y, glm::vec3 color )
{
	if( x < 0 || surface->w <= x || y < 0 || surface->h <= y )
		return;

	Uint8 r = Uint8( glm::clamp( 255*color.r, 0.f, 255.f ) );
	Uint8 g = Uint8( glm::clamp( 255*color.g, 0.f, 255.f ) );
	Uint8 b = Uint8( glm::clamp( 255*color.b, 0.f, 255.f ) );

	Uint32* p = (Uint32*)surface->pixels + y*surface->pitch/4 + x;
	*p = SDL_MapRGB( surface->format, r, g, b );
}

#endif