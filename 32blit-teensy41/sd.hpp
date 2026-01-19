#pragma once
#include <cstdint>

void sd_init();

// returns true if a new card was detected
bool sd_update();

bool sd_get_initialised();
uint32_t sd_get_num_blocks();

bool sd_read_blocks(uint32_t block, uint8_t *buf, int count);
bool sd_write_blocks(uint32_t block, const uint8_t *buf, int count);

