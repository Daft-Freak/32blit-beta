#pragma once
#include <cstdint>

#include "engine/api_private.hpp"
#include "config.h"

int get_display_page_size();
void init_display();
void update_display(uint32_t time);

bool display_render_needed();

bool display_mode_supported(blit::ScreenMode new_mode, const blit::SurfaceTemplate &new_surf_template);

void display_mode_changed(blit::ScreenMode new_mode, blit::SurfaceTemplate &new_surf_template);
