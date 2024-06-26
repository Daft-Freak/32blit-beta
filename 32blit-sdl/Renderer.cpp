#include <algorithm>
#include <cstdio>
#include <cmath>
#include <iostream>
#include "SDL.h"

#include "graphics/surface.hpp"

#include "Renderer.hpp"
#include "System.hpp"

Renderer::Renderer(SDL_Window *window, int width, int height) : sys_width(width), sys_height(height) {
	//SDL_SetHint(SDL_HINT_RENDER_DRIVER, "openGL");
	renderer = SDL_CreateRenderer(window, -1, 0);
	if (renderer == nullptr) {
		std::cerr << "could not create renderer: " << SDL_GetError() << std::endl;
	}

  current = fb_texture;

	int w, h;
	SDL_GetWindowSize(window, &w, &h);
	resize(w, h);
  set_mode(mode);

	// Clear the window.
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
	SDL_RenderClear(renderer);
	SDL_RenderPresent(renderer);

  fb_texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, sys_width, sys_height);
  fb_565_texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_BGR565, SDL_TEXTUREACCESS_STREAMING, sys_width, sys_height);
}

Renderer::~Renderer() {
	SDL_DestroyTexture(fb_texture);
	SDL_DestroyTexture(fb_565_texture);
	SDL_DestroyRenderer(renderer);
}

void Renderer::set_mode(Mode new_mode) {
	mode = new_mode;

  int w = is_lores ? sys_width / 2 : sys_width;
  int h = is_lores ? sys_height / 2 : sys_height;

  SDL_RenderSetLogicalSize(renderer, w, h);
  SDL_RenderSetIntegerScale(renderer, (mode == KeepPixels) ? SDL_TRUE : SDL_FALSE);

  if (mode == Stretch) {
    // override the automatic scaling to not preserve the ratio
    SDL_RenderSetViewport(renderer, nullptr);

    SDL_RenderSetScale(renderer, (float)win_width / w, (float)win_height / h);
	}
}

void Renderer::resize(int width, int height) {
	win_width = width;
	win_height = height;

  if (mode == Stretch)
	  set_mode(mode);
}

void Renderer::update(System *sys) {
  auto format = blit::PixelFormat(sys->format());

	current = format == blit::PixelFormat::RGB565 ? fb_565_texture : fb_texture;

  if(is_lores != (sys->mode() == 0)) {
    is_lores = sys->mode() == 0;
    set_mode(mode);
  }

	sys->update_texture(current);
}

void Renderer::_render(SDL_Texture *target, SDL_Rect *destination) {
	SDL_SetRenderTarget(renderer, target);
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
	SDL_RenderClear(renderer);
	SDL_RenderCopy(renderer, current, nullptr, destination);
}

void Renderer::present() {
  SDL_Rect dest;
  dest.x = 0;
  dest.y = 0;
  dest.w = sys_width;
  dest.h = sys_height;

	_render(nullptr, &dest);
	SDL_RenderPresent(renderer);
}

void Renderer::read_pixels(int width, int height, Uint32 format, Uint8 *buffer) {
	SDL_Texture *record_target = SDL_CreateTexture(renderer, format, SDL_TEXTUREACCESS_TARGET, width, height);
	_render(record_target, nullptr);
	SDL_RenderReadPixels(renderer, nullptr, format, buffer, width*SDL_BYTESPERPIXEL(format));
	SDL_DestroyTexture(record_target);
}

