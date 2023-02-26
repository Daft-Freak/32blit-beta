#include <algorithm>
#include <cstdio>

#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/binary_info.h"
#include "pico/time.h"

#include "storage.hpp"

#include "sd_card.pio.h"

// vga board pins
#define SD_CLK   5
#define SD_CMD  18
#define SD_DAT0 19

#define SD_TIMEOUT 100

#define SD_MAX_READ_BLOCKS 32

#define jmp_cmd_dat(state) pio_encode_jmp(sd_cmd_or_dat_offset_ ## state)

static PIO sd_pio = pio1;
static int sd_clock_sm;
static int sd_cmd_sm;
static int sd_data_sm;

static int dma_channel;
static int dma_ctrl_channel;

static uint32_t card_size_blocks = 0;
static bool is_hcs = false;
static uint16_t card_rca;
static uint8_t data_width = 1;

static bool sd_io_initialised = false;

static uint8_t sd_crc7(const uint8_t *data, uint32_t len) {
  uint8_t v = 0;

  for(uint32_t i = 0; i < len; i++) {
    v = (v << 1) ^ data[i];

    if(v & 0x80)
        v ^= 0x89;

    for(int b = 0; b < 7; b++) {
      v <<= 1;
      if(v & 0x80)
        v ^= 0x89;
    }
  }

  return v & 0x7F;
}

static uint16_t crc16(const uint8_t *data, uint32_t len) {
  uint16_t v = 0;

  for(uint32_t i = 0; i < len; i++) {
    v = v ^ data[i] << 8;

    for(int b = 0; b < 8; b++) {
      if(v & 0x8000)
        v = (v << 1) ^ 0x1021;
      else
        v <<= 1;
    }
  }

  return v;
}

static void set_clkdiv(uint16_t div) {
  pio_sm_set_clkdiv_int_frac(sd_pio, sd_clock_sm, div, 0);
  pio_sm_set_clkdiv_int_frac(sd_pio, sd_cmd_sm, div, 0);
  pio_sm_set_clkdiv_int_frac(sd_pio, sd_data_sm, div, 0);
  pio_clkdiv_restart_sm_mask(sd_pio, sd_clock_sm | sd_cmd_sm | sd_data_sm);
}

static void wait_done(int sm) {
  sd_pio->fdebug |= 1 << (PIO_FDEBUG_TXSTALL_LSB + sm); // clear stall flag
  while(!(sd_pio->fdebug & (1 << (PIO_FDEBUG_TXSTALL_LSB + sm)))); // wait
}

static void wait_not_busy(int sm) {
  pio_sm_put_blocking(sd_pio, sm, jmp_cmd_dat(state_inline_instruction) << 16 | pio_encode_set(pio_pindirs, 0)); // set input
  pio_sm_put_blocking(sd_pio, sm, jmp_cmd_dat(state_inline_instruction) << 16 | pio_encode_wait_pin(true, 0)); // wait for high
  pio_sm_put_blocking(sd_pio, sm, jmp_cmd_dat(state_inline_instruction) << 16 | jmp_cmd_dat(no_arg_state_wait_high)); // return to wait

  wait_done(sm);
}

static bool check_res_status(uint8_t *res_data) {
  uint32_t status = res_data[1] << 24 | res_data[2] << 16 | res_data[3] << 8 | res_data[4];

  if(status & 0xFDF98008) // any error bits
    return false;

  return true;
}

// res_data should be padded to word size
static bool sd_command(uint8_t cmd, uint32_t param, int res_size = 0, uint8_t *res_data = nullptr) {
  uint8_t buf[]{
    uint8_t(0x40 | cmd),
    uint8_t(param >> 24),
    uint8_t(param >> 16),
    uint8_t(param >> 8),
    uint8_t(param),
  };

  uint8_t crc = sd_crc7(buf, 5);

  // stop the sm so we can fill the fifo
  pio_sm_set_enabled(sd_pio, sd_cmd_sm, false);

  // jmp to send state + len
  pio_sm_put_blocking(sd_pio, sd_cmd_sm, jmp_cmd_dat(state_send_bits) << 16 | (48 - 1));

  // cmd + param
  pio_sm_put_blocking(sd_pio, sd_cmd_sm, buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3]);
  // param + crc + jmp to wait
  pio_sm_put_blocking(sd_pio, sd_cmd_sm, buf[4] << 24 | crc << 17 | 1 << 16 | jmp_cmd_dat(no_arg_state_wait_high));

  if(res_size) {
    pio_sm_put_blocking(sd_pio, sd_cmd_sm, jmp_cmd_dat(state_receive_bits) << 16 | (res_size - 1)); // receive command

    // prepare dma
    auto ctrl = dma_channel_hw_addr(dma_channel)->al1_ctrl & ~(DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS | DMA_CH0_CTRL_TRIG_BSWAP_BITS | DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS | DMA_CH0_CTRL_TRIG_INCR_READ_BITS);
    auto dreq = pio_get_dreq(sd_pio, sd_cmd_sm, false);
    ctrl |= (dreq << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB);
    ctrl |= (dma_channel << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB); // reset chain
    ctrl |= DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS;
    dma_channel_hw_addr(dma_channel)->al1_ctrl = ctrl;

    dma_channel_set_trans_count(dma_channel, (res_size + 31) / 32, false);
    dma_channel_set_read_addr(dma_channel, &sd_pio->rxf[sd_cmd_sm], false);
    dma_channel_set_write_addr(dma_channel, res_data, true);
  }

  pio_sm_set_enabled(sd_pio, sd_cmd_sm, true); // start

  if(res_size) {
    // pad response data to 32 bit
    if(res_size % 32)
      pio_sm_put_blocking(sd_pio, sd_cmd_sm, jmp_cmd_dat(state_inline_instruction) << 16 | pio_encode_in(pio_null, 32 - (res_size % 32)));

    // double jump to wait state
    pio_sm_put_blocking(sd_pio, sd_cmd_sm, jmp_cmd_dat(state_inline_instruction) << 16 | jmp_cmd_dat(no_arg_state_wait_high));

    absolute_time_t timeout_time = make_timeout_time_ms(SD_TIMEOUT);

    while(dma_channel_is_busy(dma_channel)) {
        // timeout
        if(absolute_time_diff_us(get_absolute_time(), timeout_time) <= 0) {
          dma_channel_abort(dma_channel);
          pio_sm_clear_fifos(sd_pio, sd_cmd_sm);
          pio_sm_exec(sd_pio, sd_cmd_sm, jmp_cmd_dat(no_arg_state_wait_high));
          return false;
        }
    }

    // swap and shift for missing msb (it's 0)
    int num_words = (res_size + 31) / 32;
    auto data32 = reinterpret_cast<uint32_t *>(res_data);
    for(int i = num_words - 1; i > 0; i--)
      data32[i] = __builtin_bswap32(data32[i] >> 1 | data32[i - 1] << 31);

    data32[0] = __builtin_bswap32(data32[0] >> 1);
  }

  wait_done(sd_cmd_sm);

  return true;
}

static bool sd_command_read_block(uint8_t cmd, uint32_t addr, uint8_t *buffer, int count = 1, uint16_t *crc = nullptr) {
  int block_len = 512;
  if(cmd == 6) // SWITCH
    block_len = 64;

  if(count > SD_MAX_READ_BLOCKS)
    return false;

  // setup config for chaining
  uint32_t chain_words[(SD_MAX_READ_BLOCKS * 2 + 1) * 2];

  // TODO: multiple blocks
  uint32_t crc_words[2];

  for(int i = 0; i < count; i++) {
    // data dest/count
    chain_words[i * 4 + 0] = uintptr_t(buffer + i * block_len);
    chain_words[i * 4 + 1] = block_len / 4;

    // crc dest/count
    chain_words[i * 4 + 2] = uintptr_t(crc_words);
    chain_words[i * 4 + 3] = data_width == 4 ? 2 : 1;
  }

  // end
  chain_words[count * 4 + 0] = 0;
  chain_words[count * 4 + 1] = 0;

  // setup data sm
  // fifo should be empty
  pio_sm_set_enabled(sd_pio, sd_data_sm, false);

  // set wrap for width
  if(data_width == 4)
    pio_sm_set_wrap(sd_pio, sd_data_sm, sd_cmd_or_dat_offset_wrap_target_for_4bit_receive, sd_cmd_or_dat_offset_wrap_for_4bit_receive - 1);
  else
    pio_sm_set_wrap(sd_pio, sd_data_sm, sd_cmd_or_dat_wrap_target, sd_cmd_or_dat_wrap);

  int crc_size = data_width * 16; // 1 or 4 16bit crcs

  // PIO cmds for data read
  uint32_t read_commands[2];
  int num_read_commands = 0;
  read_commands[num_read_commands++] = jmp_cmd_dat(state_receive_bits) << 16 | ((block_len * 8 + crc_size) / data_width - 1);

  if(data_width == 1)
    read_commands[num_read_commands++] = jmp_cmd_dat(state_inline_instruction) << 16 | pio_encode_in(pio_null, 16); // pad to word

  for(int i = 0; i < num_read_commands; i++)
    pio_sm_put(sd_pio, sd_data_sm, read_commands[i]);

  pio_sm_set_enabled(sd_pio, sd_data_sm, true);

  // prepare dma control value
  auto dma_ctrl = dma_channel_hw_addr(dma_channel)->al1_ctrl & ~(DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS | DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS);
  auto dreq = pio_get_dreq(sd_pio, sd_data_sm, false);
  dma_ctrl |= (dreq << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | DMA_CH1_CTRL_TRIG_BSWAP_BITS;
  dma_ctrl |= dma_ctrl_channel << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB;

  uint8_t res_data[8];

  if(!sd_command(cmd, addr, 48, res_data) || !check_res_status(res_data))
    return false;

  // get data
  dma_channel_hw_addr(dma_channel)->al1_ctrl = dma_ctrl;
  dma_channel_set_read_addr(dma_channel, &sd_pio->rxf[sd_data_sm], false);

  // start the chain
  dma_channel_set_read_addr(dma_ctrl_channel, &chain_words, true);

  // write the commands for the rest of the blocks
  while(--count) {
    for(int i = 0; i < num_read_commands; i++)
      pio_sm_put_blocking(sd_pio, sd_data_sm, read_commands[i]);
  }

  while(dma_channel_is_busy(dma_channel) || dma_channel_is_busy(dma_ctrl_channel));

  // return to wait
  pio_sm_put_blocking(sd_pio, sd_data_sm, jmp_cmd_dat(state_inline_instruction) << 16 | jmp_cmd_dat(no_arg_state_waiting_for_cmd));

  // TODO: 4 bit crcs
  // TODO: multiple blocks
  if(crc)
    *crc = crc_words[0] >> 16;

  wait_done(sd_data_sm);

  if(cmd == 18) {
    // end multi block
    sd_command(12, 0, 48, res_data); // STOP_TRANSMISSION
    wait_not_busy(sd_cmd_sm);
  }

  return true;
}

static bool sd_command_write_block(uint8_t cmd, uint32_t addr, uint8_t *buffer) {
  int block_len = 512;

  uint16_t crc = crc16(buffer, block_len);

  uint32_t start_bit = ~1;

  // setup data sm
  pio_sm_set_enabled(sd_pio, sd_data_sm, false);

  // make sure data pins are set to output
  pio_sm_put(sd_pio, sd_data_sm, jmp_cmd_dat(state_inline_instruction) << 16 | jmp_cmd_dat(no_arg_state_wait_high));
  // TODO: 4bit?
  pio_sm_put(sd_pio, sd_data_sm, jmp_cmd_dat(state_send_bits) << 16 | (block_len * 8 + 32 + 16 /*crc*/ + 16 /*padding*/ - 1));

  uint8_t res_data[8];

  if(!sd_command(cmd, addr, 48, res_data) || !check_res_status(res_data))
    return false;

  pio_sm_set_enabled(sd_pio, sd_data_sm, true);

  // data dma
  // (set bswap, change dreq, swap read/write incr)
  auto ctrl = dma_channel_hw_addr(dma_channel)->al1_ctrl & ~(DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS | DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS);
  auto dreq = pio_get_dreq(sd_pio, sd_data_sm, true);
  ctrl |= (dreq << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB);
  ctrl |= DMA_CH0_CTRL_TRIG_BSWAP_BITS | DMA_CH0_CTRL_TRIG_INCR_READ_BITS;
  dma_channel_hw_addr(dma_channel)->al1_ctrl = ctrl;

  dma_channel_set_trans_count(dma_channel, block_len / 4, false);
  dma_channel_set_write_addr(dma_channel, &sd_pio->txf[sd_data_sm], false);

  // start bit (1 -> 0 transition)
  pio_sm_put(sd_pio, sd_data_sm, start_bit);

  dma_channel_set_read_addr(dma_channel, buffer, true);

  while(dma_channel_is_busy(dma_channel));

  pio_sm_put_blocking(sd_pio, sd_data_sm, crc << 16);
  pio_sm_put_blocking(sd_pio, sd_data_sm, jmp_cmd_dat(state_inline_instruction) << 16 | jmp_cmd_dat(no_arg_state_waiting_for_cmd));

  wait_not_busy(sd_data_sm);

  return true;
}

static void sd_set_width(uint8_t width) {
  if(data_width == width)
    return;

  // ACMD6
  uint8_t res_data[8];
  bool cmd_res = sd_command(55, card_rca << 16, 48, res_data); // APP_CMD
  if(!cmd_res || !check_res_status(res_data))
      return;

  cmd_res = sd_command(6, width == 4 ? 2 : 0, 48, res_data); // SET_BUS_WIDTH

  if(!cmd_res ||!check_res_status(res_data))
      return;

  data_width = width;
}

void sd_init_io() {
  bi_decl_if_func_used(bi_2pins_with_names(SD_CMD, "SD CMD", SD_CLK, "SD SCK"));
  bi_decl_if_func_used(bi_pin_mask_with_name(0xF << SD_DAT0, "SD Data"));

  uint cmd_dat_offset = pio_add_program(sd_pio, &sd_cmd_or_dat_program); // offset should be 0
  uint clock_offset = pio_add_program(sd_pio, &sd_clk_program);

  // init clock
  sd_clock_sm = pio_claim_unused_sm(sd_pio, true);

  pio_sm_config c = sd_clk_program_get_default_config(clock_offset);
  sm_config_set_sideset_pins(&c, SD_CLK);
  pio_sm_init(sd_pio, sd_clock_sm, clock_offset, &c);

  // init cmd
  sd_cmd_sm = pio_claim_unused_sm(sd_pio, true);

  c = sd_cmd_or_dat_program_get_default_config(cmd_dat_offset);

  sm_config_set_out_pins(&c, SD_CMD, 1);
  sm_config_set_in_pins(&c, SD_CMD);
  sm_config_set_set_pins(&c, SD_CMD, 1);

  sm_config_set_out_shift(&c, false, true, 32);
  sm_config_set_in_shift(&c, false, true, 32);

  pio_sm_init(sd_pio, sd_cmd_sm, cmd_dat_offset, &c);

  // init data
  sd_data_sm = pio_claim_unused_sm(sd_pio, true);

  c = sd_cmd_or_dat_program_get_default_config(cmd_dat_offset);

  sm_config_set_out_pins(&c, SD_DAT0, 1); // or 4?
  sm_config_set_in_pins(&c, SD_DAT0);
  sm_config_set_set_pins(&c, SD_DAT0, 1);

  sm_config_set_out_shift(&c, false, true, 32);
  sm_config_set_in_shift(&c, false, true, 32);

  pio_sm_init(sd_pio, sd_data_sm, cmd_dat_offset, &c);

  // io setup
  pio_sm_set_pins_with_mask(sd_pio, sd_cmd_sm, 0, 1 << SD_CLK);
  pio_sm_set_pindirs_with_mask(sd_pio, sd_cmd_sm, 1 << SD_CLK, 1 << SD_CLK | 1 << SD_CMD | 0xF << SD_DAT0);

  pio_gpio_init(sd_pio, SD_CMD);
  pio_gpio_init(sd_pio, SD_CLK);

  for(int i = 0; i < 4; i++) {
    pio_gpio_init(sd_pio, SD_DAT0 + i);
    gpio_pull_up(SD_DAT0 + i);
  }

  gpio_pull_up(SD_CMD);

  hw_set_bits(&sd_pio->input_sync_bypass, 0xF << SD_DAT0 | 1 << SD_CMD);

  // DMA
  // setup with some defaults
  dma_channel = dma_claim_unused_channel(true);

  auto config = dma_channel_get_default_config(dma_channel);
  channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
  channel_config_set_read_increment(&config, false);
  channel_config_set_write_increment(&config, true);

  dma_channel_set_config(dma_channel, &config, false);

  // second channel for chaining
  dma_ctrl_channel = dma_claim_unused_channel(true);

  config = dma_channel_get_default_config(dma_ctrl_channel);
  channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
  channel_config_set_read_increment(&config, true);
  channel_config_set_write_increment(&config, true);
  channel_config_set_ring(&config, true, 3); // wrap write addr after two words

  dma_channel_set_write_addr(dma_ctrl_channel, &dma_channel_hw_addr(dma_channel)->al1_write_addr, false);
  dma_channel_set_trans_count(dma_ctrl_channel, 2, false);
  dma_channel_set_config(dma_ctrl_channel, &config, false);

  sd_io_initialised = true;
}

bool storage_init() {
  if(!sd_io_initialised)
    sd_init_io();

  // go slow for init
  set_clkdiv(250);

  // start clock
  pio_sm_set_enabled(sd_pio, sd_clock_sm, true);

  // send cmd0
  sd_command(0, 0);  // GO_IDLE_STATE

  // check voltage range / check if SDv2
  bool is_v2 = true;

  uint8_t data[8];
  if(!sd_command(8, 0x1AA, 48, data)) // SEND_IF_COND
    return false;

  uint32_t data_arg = data[1] << 24 | data[2] << 16 | data[3] << 8 | data[4];

  // check supported? (timeout)

  if(data_arg != 0x1AA) {
    printf("CMD8 returned unexpected %lX!\n", data_arg);
    return false;
  }

  uint32_t ocr;

  // init
  while(true) {
    bool cmd_res = sd_command(55, 0, 48, data); // APP_CMD

    if(!cmd_res || !check_res_status(data))
      continue;

    if(!sd_command(41, 1 << 20 /*3.2-3.3v*/ | (is_v2 ? 0x40000000 : 0), 48, data)) // APP_SEND_OP_COND
      continue;

    ocr = data[1] << 24 | data[2] << 16 | data[3] << 8 | data[4];

    // check if ready
    if(ocr & (1 << 31))
      break;
  }

  // check OCR
  is_hcs = ocr & (1 << 30);

  // address setup
  uint8_t cid_res[20];
  if(!sd_command(2, 0, 136, cid_res)) // ALL_SEND_CID
    return false;

  if(!sd_command(3, 0, 48, data)) // SEND_RELATIVE_ADDR
    return false;

  card_rca = data[1] << 8 | data[2];

  // csd r2
  // read CSD
  uint8_t csd_res[20];
  if(!sd_command(9, card_rca << 16, 136, csd_res)) // SEND_CSD
    return false;

  auto csd = csd_res + 1;

  // v1
  if((csd[0] >> 6) == 0) {
    int c_size = ((csd[6] & 0x3) << 10) | (csd[7] << 2) | (csd[8] >> 6);
    int c_size_mult =  ((csd[9] & 0x3) << 1) | (csd[10] >> 7);
    int readBlLen = csd[5] & 0xF;

    uint32_t num_blocks = uint32_t(c_size + 1) * (2 << (c_size_mult + 1));
    uint32_t size_bytes = num_blocks * (2 << (readBlLen - 1));
    card_size_blocks = size_bytes / 512;
  } else { // v2
    // measured in 512k blocks
    card_size_blocks = (((int32_t(csd[7] & 0x3F) << 16) | (uint32_t(csd[8]) << 8) | csd[9]) + 1) * 1024;
  }

  printf("Detected %s card, size %lu blocks\n", is_v2 ? (is_hcs ? "SDHC" : "SDv2") : "SDv1", card_size_blocks);

  // select the card
  if(!sd_command(7, card_rca << 16, 48, data)) // SELECT_CARD
    return false;

  wait_not_busy(sd_cmd_sm);

  // attempt high speed
  bool high_speed = false;
  uint8_t switch_res[64];

  if(sd_command_read_block(6, 0x80FFFFF1, switch_res)) { // SWITCH
    if((switch_res[16] & 0xF) == 1)
      high_speed = true;
  }

#ifdef OVERCLOCK_250
  if(high_speed)
    set_clkdiv(3); // 41.6...
  else
    set_clkdiv(5); // 25
#else
  if(high_speed)
    set_clkdiv(2); // 31.25
  else
    set_clkdiv(3); // 20.83...
#endif

  return true;
}

void get_storage_size(uint16_t &block_size, uint32_t &num_blocks) {
  block_size = 512;
  num_blocks = card_size_blocks;
}

int32_t storage_read(uint32_t block, uint32_t offset, void *buffer, uint32_t size_bytes) {
  // offset should be 0 (block size == msc buffer size)

  if(!is_hcs)
    block *= 512;

  auto blocks = size_bytes / 512;

  sd_set_width(4);

  if(blocks == 1) {
    if(!sd_command_read_block(17, block, (uint8_t *)buffer)) // READ_SINGLE_BLOCK
      return 0;

    return size_bytes;
  } else {
    uint32_t read = 0;
    while(blocks) {
      int num = std::min(UINT32_C(SD_MAX_READ_BLOCKS), blocks);
      blocks -= num;
      if(!sd_command_read_block(18, block, (uint8_t *)buffer + read, num))
        break;
      read += num * 512;
      block += num * (is_hcs ? 1 : 512);
    }

    return read;
  }
}

int32_t storage_write(uint32_t block, uint32_t offset, const uint8_t *buffer, uint32_t size_bytes) {
  if(!is_hcs)
    block *= 512;

  auto blocks = size_bytes / 512;

  sd_set_width(1);

  int32_t written = 0;

  // TODO: multi block writes
  while(blocks--) {
    if(!sd_command_write_block(24, block, (uint8_t *)buffer + written)) // WRITE_SINGLE_BLOCK
      break;

    written += 512;
    if(!is_hcs)
      block += 512;
    else
      block++;
  }

  return written;
}
