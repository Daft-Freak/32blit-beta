#include <cstdint>

#include "hardware/flash.h"
#include "hardware/irq.h"
#include "hardware/structs/qmi.h"

#include "32blit.hpp"

struct BlitGameHeader {
  uint32_t magic;

  // don't care about these
  uint32_t render;
  uint32_t tick;
  uint32_t init;

  uint32_t end;

  // rest isn't important
};

void blit_fast_code(launch)(uint32_t offset, uint32_t binary_size) {
  // disable all irqs
  irq_set_mask_n_enabled(0, ~0u, false);
  irq_set_mask_n_enabled(1, ~0u, false);

  auto addr = XIP_BASE;

  const uint32_t atrans_pane_size = 4 * 1024 * 1024;

  // setup translation
  uint32_t total_size = binary_size;
  uint32_t size = std::min(atrans_pane_size, total_size);
  qmi_hw->atrans[0] = (size >> 12) << QMI_ATRANS0_SIZE_LSB
                    | (offset >> 12) << QMI_ATRANS0_BASE_LSB;

  // second pane (this is riskier as this loader is running from atrans1)
  if(total_size > atrans_pane_size) {
    offset += atrans_pane_size;
    total_size -= atrans_pane_size;
    uint32_t size = std::min(atrans_pane_size, total_size);
    qmi_hw->atrans[1] = (size >> 12) << QMI_ATRANS1_SIZE_LSB
                      | (offset >> 12) << QMI_ATRANS1_BASE_LSB;
  }

  // invalidate cache
  for(uint32_t off = 0; off < binary_size; off += 8)
    *(volatile uint8_t *)(XIP_MAINTENANCE_BASE + off + 2/*invalidate by addr*/) = 0;

  // jump to it
  scb_hw->vtor = addr;

  asm volatile(
    "ldr r0, [%0]\n"
    "ldr r1, [%0, #4]\n"
    "msr msp, r0\n" // set SP
    "bx r1" // branch to reset
    :
    : "r" (addr)
    : "r0", "r1"
  );
}

void init() {
  // find aligned end
  extern char __flash_binary_start, __flash_binary_end;
  auto end_ptr = &__flash_binary_end;
  end_ptr = (char *)((((uintptr_t)end_ptr) + 0xFFF) & ~0xFFF); // round up to 4k boundary

  auto header = (BlitGameHeader *)&__flash_binary_start;
  auto size = header->end;

  // convert back to offset
  auto offset = end_ptr - (char *)XIP_BASE;
  // blits are launched using atrans1, map back to raw offset
  auto trans_offset = (qmi_hw->atrans[1] & QMI_ATRANS1_BASE_BITS) >> QMI_ATRANS1_BASE_LSB << 12;
  offset += trans_offset - (4 * 1024 * 1024);

  launch(offset, size);
}

void update(uint32_t) {

}

void render(uint32_t) {

}
