#pragma once
#include <cstdint>

#include "engine/api_private.hpp"

void blit_api_init(const char *file_base_dir, const char *save_dir);

// handlers in main
bool screen_mode_changed(blit::SurfaceTemplate &info);
uint32_t get_current_time();
void debug_message(const char *message);

// user code
void init();
void update(uint32_t time);
void render(uint32_t time);

extern const char *metadata_title;
extern const char *metadata_author;
extern const char *metadata_description;
extern const char *metadata_version;
extern const char *metadata_url;
extern const char *metadata_category;
