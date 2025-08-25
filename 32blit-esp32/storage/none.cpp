#include "storage.hpp"

bool storage_init() {
  return false;
}

bool is_storage_available() {
  return false;
}

bool has_storage_changed() {
  return false;
}

void get_storage_size(uint16_t &block_size, uint32_t &num_blocks) {
  block_size = 0;
  num_blocks = 0;
}

int32_t storage_read(uint32_t sector, uint32_t offset, void *buffer, uint32_t size_bytes) {
  return -1;
}

int32_t storage_write(uint32_t sector, uint32_t offset, const uint8_t *buffer, uint32_t size_bytes) {
  return -1;
}
