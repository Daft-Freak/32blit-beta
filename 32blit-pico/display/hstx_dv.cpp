#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/resets.h"
#include "hardware/structs/hstx_ctrl.h"
#include "hardware/structs/hstx_fifo.h"

#include "display.hpp"

#include "config.h"

// DVI constants

#define TMDS_CTRL_00 0x354u
#define TMDS_CTRL_01 0x0abu
#define TMDS_CTRL_10 0x154u
#define TMDS_CTRL_11 0x2abu

#define SYNC_V0_H0 (TMDS_CTRL_00 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V0_H1 (TMDS_CTRL_01 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V1_H0 (TMDS_CTRL_10 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V1_H1 (TMDS_CTRL_11 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))

// mode
// active area needs to be consistent with (2x) DISPLAY_WIDTH/_HEIGHT
//#define MODE_H_SYNC_POLARITY 0 // unused, assumed to be active-low
#define MODE_H_FRONT_PORCH   16
#define MODE_H_SYNC_WIDTH    96
#define MODE_H_BACK_PORCH    48
#define MODE_H_ACTIVE_PIXELS 640

//#define MODE_V_SYNC_POLARITY 0
#define MODE_V_FRONT_PORCH   10
#define MODE_V_SYNC_WIDTH    2
#define MODE_V_BACK_PORCH    33
#define MODE_V_ACTIVE_LINES  480

/*
#define MODE_H_TOTAL_PIXELS ( \
  MODE_H_FRONT_PORCH + MODE_H_SYNC_WIDTH + \
  MODE_H_BACK_PORCH  + MODE_H_ACTIVE_PIXELS \
)*/
#define MODE_V_TOTAL_LINES  ( \
  MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH + \
  MODE_V_BACK_PORCH  + MODE_V_ACTIVE_LINES \
)

#define HSTX_CMD_RAW         (0x0u << 12)
#define HSTX_CMD_RAW_REPEAT  (0x1u << 12)
#define HSTX_CMD_TMDS        (0x2u << 12)
#define HSTX_CMD_TMDS_REPEAT (0x3u << 12)
#define HSTX_CMD_NOP         (0xfu << 12)

// HSTX command lists

// Lists are padded with NOPs to be >= HSTX FIFO size, to avoid DMA rapidly
// pingponging and tripping up the IRQs.

static uint32_t vblank_line_vsync_off[] = {
  HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH,
  SYNC_V1_H1,
  HSTX_CMD_RAW_REPEAT | MODE_H_SYNC_WIDTH,
  SYNC_V1_H0,
  HSTX_CMD_RAW_REPEAT | (MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS),
  SYNC_V1_H1,
  HSTX_CMD_NOP
};

static uint32_t vblank_line_vsync_on[] = {
  HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH,
  SYNC_V0_H1,
  HSTX_CMD_RAW_REPEAT | MODE_H_SYNC_WIDTH,
  SYNC_V0_H0,
  HSTX_CMD_RAW_REPEAT | (MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS),
  SYNC_V0_H1,
  HSTX_CMD_NOP,
};

static const uint32_t vactive_line[] = {
  HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH,
  SYNC_V1_H1,
  HSTX_CMD_RAW_REPEAT | MODE_H_SYNC_WIDTH,
  SYNC_V1_H0,
  HSTX_CMD_RAW_REPEAT | MODE_H_BACK_PORCH,
  SYNC_V1_H1,
  HSTX_CMD_TMDS       | MODE_H_ACTIVE_PIXELS
};

// DMA logic

#define HSTX_DMA_CH_BASE 0

static uint8_t cur_dma_ch = HSTX_DMA_CH_BASE;

// pixel/line repeat
static uint8_t h_shift = 0;
static uint8_t v_shift = 0;
static uint8_t new_h_shift = 0;
static uint8_t new_v_shift = 0;

// A ping and a pong are cued up initially, so the first time we enter this
// handler it is to cue up the second ping after the first ping has completed.
// This is the third scanline overall (-> =2 because zero-based).
static uint v_scanline = 2;

static bool started = false;
static volatile bool do_render = true;
static volatile bool need_mode_change = false;
static uint8_t *cur_display_buffer = nullptr;

// temp buffer for expanding lines (pixel double)
// two scanlines + include the cmdlist(s) so we can avoid an irq
static uint32_t scanline_buffer[(MODE_H_ACTIVE_PIXELS * sizeof(uint16_t) + sizeof(vactive_line)) / sizeof(uint32_t) * 2];

void __scratch_x("") dma_irq_handler() {
  // dma_pong indicates the channel that just finished, which is the one
  // we're about to reload.
  dma_channel_hw_t *ch = &dma_hw->ch[cur_dma_ch];
  dma_hw->intr = 1u << cur_dma_ch;
  cur_dma_ch ^= 1;

  if (v_scanline >= MODE_V_FRONT_PORCH && v_scanline < (MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH)) {
    ch->read_addr = (uintptr_t)vblank_line_vsync_on;
  } else if (v_scanline < MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH + MODE_V_BACK_PORCH) {
    ch->read_addr = (uintptr_t)vblank_line_vsync_off;
    ch->transfer_count = std::size(vblank_line_vsync_off);
  } else {
    int display_line = v_scanline - (MODE_V_TOTAL_LINES - MODE_V_ACTIVE_LINES);
    bool first = !(display_line & ((1 << v_shift) - 1));
    display_line >>= v_shift;

    const auto line_buf_size_words = sizeof(scanline_buffer) / sizeof(uint32_t) / 2;
    auto temp_ptr = scanline_buffer + (display_line & 1) * line_buf_size_words;
    ch->read_addr = (uintptr_t)temp_ptr;

    ch->transfer_count = std::size(vactive_line) + (MODE_H_ACTIVE_PIXELS * 2) / sizeof(uint32_t);

    // expand line if needed
    if(first) {
      auto w = MODE_H_ACTIVE_PIXELS >> h_shift;
      auto fb_line_ptr = reinterpret_cast<uint16_t *>(cur_display_buffer) + display_line * w;

      temp_ptr += std::size(vactive_line);
      // expand to words
      // we configures the expander to not shift, so we don't need to repeat the value
      for(int i = 0; i < w; i++) {
        auto px = *fb_line_ptr++;
        *temp_ptr++ = px;
        if(h_shift == 2)
          *temp_ptr++ = px;
      }
    }
  }

  v_scanline++;

  if(v_scanline == MODE_V_TOTAL_LINES) {
    v_scanline = 0;
  } else if(v_scanline == 2) {
    // new frame, swap buffers and trigger render
    // wait until scanline 2 so that there are no active lines in progress

    if(!do_render) {
      if(fb_double_buffer)
        std::swap(blit::screen.data, cur_display_buffer);
      do_render = true;
    }

    // set h/v shift
    if(need_mode_change) {
      h_shift = new_h_shift;
      v_shift = new_v_shift;

      need_mode_change = false;
      new_h_shift = new_v_shift = false;
    }
  }
}

void init_display() {
  // reset HSTX to make sure it's in a good state
  reset_unreset_block_num_wait_blocking(RESET_HSTX);

  // divide down if we're overclocking (we probably are)
#if OVERCLOCK_250
  clock_configure(clk_hstx, 0, CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLK_SYS, 250000000, 125000000);
#endif

  // Configure HSTX's TMDS encoder for RGB565
  // (it starts from bit 7)
  hstx_ctrl_hw->expand_tmds =
    4  << HSTX_CTRL_EXPAND_TMDS_L2_NBITS_LSB | // R
    29 << HSTX_CTRL_EXPAND_TMDS_L2_ROT_LSB   | //
    5  << HSTX_CTRL_EXPAND_TMDS_L1_NBITS_LSB | // G
    3  << HSTX_CTRL_EXPAND_TMDS_L1_ROT_LSB   | //
    4  << HSTX_CTRL_EXPAND_TMDS_L0_NBITS_LSB | // B
    8  << HSTX_CTRL_EXPAND_TMDS_L0_ROT_LSB;    //

  // Pixels (TMDS) come in 2 16-bit chunks. Control symbols (RAW) are an
  // entire 32-bit word.
  hstx_ctrl_hw->expand_shift =
    2  << HSTX_CTRL_EXPAND_SHIFT_ENC_N_SHIFTS_LSB |
    //16 << HSTX_CTRL_EXPAND_SHIFT_ENC_SHIFT_LSB | // don't shift pixels, output the same thing twice
    1  << HSTX_CTRL_EXPAND_SHIFT_RAW_N_SHIFTS_LSB |
    0  << HSTX_CTRL_EXPAND_SHIFT_RAW_SHIFT_LSB;

  // Serial output config: clock period of 5 cycles, pop from command
  // expander every 5 cycles, shift the output shiftreg by 2 every cycle.
  hstx_ctrl_hw->csr = 0;
  hstx_ctrl_hw->csr =
    HSTX_CTRL_CSR_EXPAND_EN_BITS |
    5u << HSTX_CTRL_CSR_CLKDIV_LSB |
    5u << HSTX_CTRL_CSR_N_SHIFTS_LSB |
    2u << HSTX_CTRL_CSR_SHIFT_LSB |
    HSTX_CTRL_CSR_EN_BITS;


  // HSTX outputs 0 through 7 appear on GPIO 12 through 19.
  // Pinout on Pico DVI sock:
  //
  //   GP12 D0+  GP13 D0-
  //   GP14 CK+  GP15 CK-
  //   GP16 D2+  GP17 D2-
  //   GP18 D1+  GP19 D1-

  // Assign clock pair to two neighbouring pins:
  hstx_ctrl_hw->bit[2] = HSTX_CTRL_BIT0_CLK_BITS;
  hstx_ctrl_hw->bit[3] = HSTX_CTRL_BIT0_CLK_BITS | HSTX_CTRL_BIT0_INV_BITS;
  for(uint lane = 0; lane < 3; ++lane) {
    // For each TMDS lane, assign it to the correct GPIO pair based on the
    // desired pinout:
    static const int lane_to_output_bit[3] = {0, 6, 4};
    int bit = lane_to_output_bit[lane];
    // Output even bits during first half of each HSTX cycle, and odd bits
    // during second half. The shifter advances by two bits each cycle.
    uint32_t lane_data_sel_bits =
      (lane * 10    ) << HSTX_CTRL_BIT0_SEL_P_LSB |
      (lane * 10 + 1) << HSTX_CTRL_BIT0_SEL_N_LSB;
    // The two halves of each pair get identical data, but one pin is inverted.
    hstx_ctrl_hw->bit[bit    ] = lane_data_sel_bits;
    hstx_ctrl_hw->bit[bit + 1] = lane_data_sel_bits | HSTX_CTRL_BIT0_INV_BITS;
  }

  for(int i = 12; i <= 19; ++i)
    gpio_set_function(i, GPIO_FUNC_HSTX);

  // Both channels are set up identically, to transfer a whole scanline and
  // then chain to the opposite channel. Each time a channel finishes, we
  // reconfigure the one that just finished, meanwhile the opposite channel
  // is already making progress.
  dma_channel_claim(HSTX_DMA_CH_BASE + 0);
  dma_channel_claim(HSTX_DMA_CH_BASE + 1);

  for(int i = 0; i < 2; i++) {
    dma_channel_config c;
    c = dma_channel_get_default_config(HSTX_DMA_CH_BASE + i);
    channel_config_set_chain_to(&c, (HSTX_DMA_CH_BASE + i) ^ 1);
    channel_config_set_dreq(&c, DREQ_HSTX);
    dma_channel_configure(
      HSTX_DMA_CH_BASE + i,
      &c,
      &hstx_fifo_hw->fifo,
      vblank_line_vsync_off,
      count_of(vblank_line_vsync_off),
      false
    );
  }

  dma_hw->ints0 = (3u << HSTX_DMA_CH_BASE);
  dma_hw->inte0 = (3u << HSTX_DMA_CH_BASE);
  irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
  irq_set_enabled(DMA_IRQ_0, true);

  // fill line headers
  const auto line_buf_size = sizeof(scanline_buffer) / 2;
  auto line_buf0 = scanline_buffer;
  auto line_buf1 = scanline_buffer + line_buf_size / sizeof(uint32_t);

  for(size_t i = 0; i < std::size(vactive_line); i++)
    *line_buf0++ = *line_buf1++ = vactive_line[i];

  // set irq to highest priority
  irq_set_priority(DMA_IRQ_0, PICO_HIGHEST_IRQ_PRIORITY);
}

void update_display(uint32_t time) {
  if(do_render) {
    blit::render(time);

    // start dma after first render
    if(!started) {
      started = true;
      dma_channel_start(HSTX_DMA_CH_BASE);
    } else if(new_h_shift) {
      need_mode_change = true;
    }
    do_render = false;
  }
}

void init_display_core1() {
}

void update_display_core1() {
}

bool display_render_needed() {
  return do_render;
}

bool display_mode_supported(blit::ScreenMode new_mode, const blit::SurfaceTemplate &new_surf_template) {
  if(new_surf_template.format != blit::PixelFormat::RGB565)
    return false;

  blit::Size expected_bounds(DISPLAY_WIDTH, DISPLAY_HEIGHT);

  auto w = new_surf_template.bounds.w;
  auto h = new_surf_template.bounds.h;

  if(w != MODE_H_ACTIVE_PIXELS / 2 && w != MODE_H_ACTIVE_PIXELS / 4)
    return false;

  if(h != MODE_V_ACTIVE_LINES && h != MODE_V_ACTIVE_LINES / 2 && h != MODE_V_ACTIVE_LINES / 4)
    return false;

  return true;
}

void display_mode_changed(blit::ScreenMode new_mode, blit::SurfaceTemplate &new_surf_template) {
  auto display_buf_base = (uint8_t *)screen_fb;

  // prevent buffer swap while we're doing this
  do_render = true;

  bool use_second_buf = fb_double_buffer && (!blit::screen.data || blit::screen.data == display_buf_base);
  cur_display_buffer = use_second_buf ? display_buf_base + get_display_page_size() : display_buf_base;

  // avoid resetting screen.data to first buffer, causing both buffers to be the same
  if(fb_double_buffer && !use_second_buf)
    new_surf_template.data = display_buf_base + get_display_page_size();

  // set h/v shift
  new_h_shift = 0;
  new_v_shift = 0;

  while(MODE_H_ACTIVE_PIXELS >> new_h_shift > new_surf_template.bounds.w)
    new_h_shift++;

  while(MODE_V_ACTIVE_LINES >> new_v_shift > new_surf_template.bounds.h)
    new_v_shift++;

  // check if we're actually changing scale
  if(new_v_shift == v_shift && new_h_shift == h_shift) {
    new_h_shift = new_v_shift = 0;
    return;
  }

  // don't do it yet if already started
  // (will set need_mode_change after next render)
  if(started)
    return;

  h_shift = new_h_shift;
  v_shift = new_v_shift;
  new_h_shift = new_v_shift = 0;
}
