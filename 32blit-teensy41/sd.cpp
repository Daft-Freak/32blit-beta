#include "core_pins.h"
#include "imxrt.h"
#include "usb_serial.h"

#include "sd.hpp"

// reg defs
#define IOMUXC_SW_PAD_CTL_PAD_PKE    (1 << 12)
#define IOMUXC_SW_PAD_CTL_PAD_PUE    (1 << 13)
#define IOMUXC_SW_PAD_CTL_PAD_PUS(n) (((n) & 3) << 14)
#define IOMUXC_SW_PAD_CTL_PAD_DSE(n) (((n) & 7) <<  3)

#define USDHC_BLK_ATT_BLKSIZE_SHIFT 0
#define USDHC_BLK_ATT_BLKSIZE_MASK  (0x1FFF << USDHC_BLK_ATT_BLKSIZE_SHIFT)
#define USDHC_BLK_ATT_BLKSIZE(n)    (n << USDHC_BLK_ATT_BLKSIZE_SHIFT)
#define USDHC_BLK_ATT_BLKCNT_SHIFT  16
#define USDHC_BLK_ATT_BLKCNT_MASK   (0xFFFF << USDHC_BLK_ATT_BLKCNT_SHIFT)
#define USDHC_BLK_ATT_BLKCNT(n)     (n << USDHC_BLK_ATT_BLKCNT_SHIFT)

#define USDHC_CMD_XFR_TYP_RSPTYP_SHIFT      16
#define USDHC_CMD_XFR_TYP_RSPTYP_MASK       (3 << USDHC_CMD_XFR_TYP_RSPTYP_SHIFT)
#define USDHC_CMD_XFR_TYP_RSPTYP_NONE       0
#define USDHC_CMD_XFR_TYP_RSPTYP_136BIT     (1 << USDHC_CMD_XFR_TYP_RSPTYP_SHIFT) // R2
#define USDHC_CMD_XFR_TYP_RSPTYP_48BIT      (2 << USDHC_CMD_XFR_TYP_RSPTYP_SHIFT) // R1,3,4,5,6
#define USDHC_CMD_XFR_TYP_RSPTYP_48BIT_BUSY (3 << USDHC_CMD_XFR_TYP_RSPTYP_SHIFT) // R1b
#define USDHC_CMD_XFR_TYP_CCCEN             (1 << 19) // CRC check
#define USDHC_CMD_XFR_TYP_CICEN             (1 << 20) // index check
#define USDHC_CMD_XFR_TYP_DPSEL             (1 << 21) // data present
#define USDHC_CMD_XFR_TYP_CMDTYP_SHIFT      22
#define USDHC_CMD_XFR_TYP_CMDTYP_MASK       (3 << USDHC_CMD_XFR_TYP_CMDTYP_SHIFT)
#define USDHC_CMD_XFR_TYP_CMDINX_SHIFT      24
#define USDHC_CMD_XFR_TYP_CMDINX_MASK       (0x3F << USDHC_CMD_XFR_TYP_CMDINX_SHIFT)
#define USDHC_CMD_XFR_TYP_CMDINX(n)         (n << USDHC_CMD_XFR_TYP_CMDINX_SHIFT)

#define USDHC_PRES_STATE_CIHB       (1 << 0) // command inhibit (cmd)
#define USDHC_PRES_STATE_CDIHB      (1 << 1) // command inhibit (data)
#define USDHC_PRES_STATE_DLA        (1 << 2) // data line active
#define USDHC_PRES_STATE_SDSTB      (1 << 3) // sd clock stable
#define USDHC_PRES_STATE_IPGOFF     (1 << 4)
#define USDHC_PRES_STATE_HCKOFF     (1 << 5)
#define USDHC_PRES_STATE_PEROFF     (1 << 6)
#define USDHC_PRES_STATE_SDOFF      (1 << 7)
#define USDHC_PRES_STATE_WTA        (1 << 8) // write transfer active
#define USDHC_PRES_STATE_RTA        (1 << 9) // read transfer active
#define USDHC_PRES_STATE_BWEN       (1 << 10) // buffer write enable
#define USDHC_PRES_STATE_BREN       (1 << 11) // buffer read enable
#define USDHC_PRES_STATE_RTR        (1 << 12)
#define USDHC_PRES_STATE_TSCD       (1 << 15)
#define USDHC_PRES_STATE_CINST      (1 << 16) // card inserted
#define USDHC_PRES_STATE_CLSL       (1 << 23)
#define USDHC_PRES_STATE_DLSL_SHIFT 24
#define USDHC_PRES_STATE_DLSL_MASK  (0xFF << USDHC_PRES_STATE_DLSL_SHIFT)

#define USDHC_PROT_CTRL_DTW_SHIFT          1
#define USDHC_PROT_CTRL_DTW_MASK           (3 << USDHC_PROT_CTRL_DTW_SHIFT)
#define USDHC_PROT_CTRL_DTW_1BIT           0
#define USDHC_PROT_CTRL_DTW_4BIT           (1 << USDHC_PROT_CTRL_DTW_SHIFT)
#define USDHC_PROT_CTRL_DTW_8BIT           (2 << USDHC_PROT_CTRL_DTW_SHIFT)
#define USDHC_PROT_CTRL_D3CD               (1 << 3) // DATA3 as card detect
#define USDHC_PROT_CTRL_EMODE_SHIFT        4
#define USDHC_PROT_CTRL_EMODE_MASK         (3 << USDHC_PROT_CTRL_EMODE_SHIFT)
#define USDHC_PROT_CTRL_EMODE_BIG          0
#define USDHC_PROT_CTRL_EMODE_HALF_BIG     (1 << USDHC_PROT_CTRL_EMODE_SHIFT)
#define USDHC_PROT_CTRL_EMODE_LITTLE       (2 << USDHC_PROT_CTRL_EMODE_SHIFT)
#define USDHC_PROT_CTRL_DMASEL_SHIFT       8
#define USDHC_PROT_CTRL_DMASEL_MASK        (3 << USDHC_PROT_CTRL_DMASEL_SHIFT)
#define USDHC_PROT_CTRL_DMASEL_NONE_SIMPLE 0
#define USDHC_PROT_CTRL_DMASEL_ADMA1       (1 << USDHC_PROT_CTRL_DMASEL_SHIFT)
#define USDHC_PROT_CTRL_DMASEL_ADMA2       (2 << USDHC_PROT_CTRL_DMASEL_SHIFT)
#define USDHC_PROT_CTRL_SABGREQ            (1 << 16)
#define USDHC_PROT_CTRL_CREQ               (1 << 17)
#define USDHC_PROT_CTRL_RWCTL              (1 << 18)
#define USDHC_PROT_CTRL_IABG               (1 << 19)
#define USDHC_PROT_CTRL_RD_DONE_NO_8CLK    (1 << 20)
#define USDHC_PROT_CTRL_WECINT             (1 << 24)
#define USDHC_PROT_CTRL_WECINS             (1 << 25)
#define USDHC_PROT_CTRL_WECRM              (1 << 26)
#define USDHC_PROT_CTRL_BURST_LEN_EN_SHIFT 27
#define USDHC_PROT_CTRL_BURST_LEN_EN_MASK  (7 << USDHC_PROT_CTRL_BURST_LEN_EN_SHIFT)
#define USDHC_PROT_CTRL_NON_EXACT_BLK_RD   (1 << 26)

#define USDHC_SYS_CTRL_DVS_SHIFT     4
#define USDHC_SYS_CTRL_DVS_MASK      (0xF << USDHC_SYS_CTRL_DVS_SHIFT)
#define USDHC_SYS_CTRL_DVS(n)        ((n - 1) << USDHC_SYS_CTRL_DVS_SHIFT)
#define USDHC_SYS_CTRL_SDCLKFS_SHIFT 8
#define USDHC_SYS_CTRL_SDCLKFS_MASK  (0xFF << USDHC_SYS_CTRL_SDCLKFS_SHIFT)
#define USDHC_SYS_CTRL_SDCLKFS(n)    ((n >> 1) << USDHC_SYS_CTRL_SDCLKFS_SHIFT)
#define USDHC_SYS_CTRL_DTOCV_SHIFT   16
#define USDHC_SYS_CTRL_DTOCV_MASK    (0xF << USDHC_SYS_CTRL_DTOCV_SHIFT)
#define USDHC_SYS_CTRL_DTOCV(n)      (n << USDHC_SYS_CTRL_DTOCV_SHIFT)
#define USDHC_SYS_CTRL_IPP_RST_N     (1 << 23)
#define USDHC_SYS_CTRL_RSTA          (1 << 24) // reset all
#define USDHC_SYS_CTRL_RSTC          (1 << 25) // reset cmd
#define USDHC_SYS_CTRL_RSTD          (1 << 26) // reset data
#define USDHC_SYS_CTRL_INITA         (1 << 27) // initialisation active
#define USDHC_SYS_CTRL_RSTT          (1 << 28) // reset tuning

#define USDHC_INT_STATUS_CC             (1 << 0) // command complete
#define USDHC_INT_STATUS_TC             (1 << 1) // transfer complete
#define USDHC_INT_STATUS_BGE            (1 << 2) // block gap event
#define USDHC_INT_STATUS_DINT           (1 << 3) // DMA interrupt
#define USDHC_INT_STATUS_BWR            (1 << 4) // buffer write ready
#define USDHC_INT_STATUS_BRR            (1 << 5) // buffer read ready
#define USDHC_INT_STATUS_CINS           (1 << 6) // card insertion
#define USDHC_INT_STATUS_CRM            (1 << 7) // card removal
#define USDHC_INT_STATUS_CINT           (1 << 8) // card interrupt
#define USDHC_INT_STATUS_RTE            (1 << 12) // re-tuning event
#define USDHC_INT_STATUS_TP             (1 << 14) // tuning pass
#define USDHC_INT_STATUS_ERR_INT_STATUS (1 << 15)
#define USDHC_INT_STATUS_CTOE           (1 << 16) // command timeout error
#define USDHC_INT_STATUS_CCE            (1 << 17) // command CRC error
#define USDHC_INT_STATUS_CEBE           (1 << 18) // command end bit error
#define USDHC_INT_STATUS_CIE            (1 << 19) // command index error
#define USDHC_INT_STATUS_DTOE           (1 << 20) // data timeout error
#define USDHC_INT_STATUS_DCE            (1 << 21) // data CRC error
#define USDHC_INT_STATUS_DEBE           (1 << 22) // data end bit error
#define USDHC_INT_STATUS_AC12E          (1 << 24) // auto CMD12 error
#define USDHC_INT_STATUS_TNE            (1 << 26) // tuning error
#define USDHC_INT_STATUS_DMAE           (1 << 28) // DMA error

#define USDHC_INT_STATUS_EN_CCSEN    USDHC_INT_STATUS_CC    // command complete
#define USDHC_INT_STATUS_EN_TCSEN    USDHC_INT_STATUS_TC    // transfer complete
#define USDHC_INT_STATUS_EN_BGESEN   USDHC_INT_STATUS_BGE   // block gap event
#define USDHC_INT_STATUS_EN_DINTSEN  USDHC_INT_STATUS_DINT  // DMA interrupt
#define USDHC_INT_STATUS_EN_BWRSEN   USDHC_INT_STATUS_BWR   // buffer read ready
#define USDHC_INT_STATUS_EN_BRRSEN   USDHC_INT_STATUS_BRR   // buffer write ready
#define USDHC_INT_STATUS_EN_CINSSEN  USDHC_INT_STATUS_CINS  // card insertion
#define USDHC_INT_STATUS_EN_CRMSEN   USDHC_INT_STATUS_CRM   // card removal
#define USDHC_INT_STATUS_EN_CINTSEN  USDHC_INT_STATUS_CINT  // card interrupt
#define USDHC_INT_STATUS_EN_RTESEN   USDHC_INT_STATUS_RTE   // re-tuning event
#define USDHC_INT_STATUS_EN_TPSEN    USDHC_INT_STATUS_TP    // tuning pass
#define USDHC_INT_STATUS_EN_CTOESEN  USDHC_INT_STATUS_CTOE  // command timeout error
#define USDHC_INT_STATUS_EN_CCESEN   USDHC_INT_STATUS_CCE   // command CRC error
#define USDHC_INT_STATUS_EN_CEBESEN  USDHC_INT_STATUS_CEBE  // command end bit error
#define USDHC_INT_STATUS_EN_CIESEN   USDHC_INT_STATUS_CIE   // command index error
#define USDHC_INT_STATUS_EN_DTOESEN  USDHC_INT_STATUS_DTOE  // data timeout error
#define USDHC_INT_STATUS_EN_DCESEN   USDHC_INT_STATUS_DCE   // data CRC error
#define USDHC_INT_STATUS_EN_DEBESEN  USDHC_INT_STATUS_DEBE  // data end bit error
#define USDHC_INT_STATUS_EN_AC12ESEN USDHC_INT_STATUS_AC12E // auto CMD12 error
#define USDHC_INT_STATUS_EN_TNESEN   USDHC_INT_STATUS_TNE   // tuning error
#define USDHC_INT_STATUS_EN_DMAESEN  USDHC_INT_STATUS_DMAE  // DMA error

#define USDHC_INT_SIGNAL_EN_CCIEN    USDHC_INT_STATUS_CC    // command complete
#define USDHC_INT_SIGNAL_EN_TCIEN    USDHC_INT_STATUS_TC    // transfer complete
#define USDHC_INT_SIGNAL_EN_BGEIEN   USDHC_INT_STATUS_BGE   // block gap event
#define USDHC_INT_SIGNAL_EN_DINTIEN  USDHC_INT_STATUS_DINT  // DMA interrupt
#define USDHC_INT_SIGNAL_EN_BWRIEN   USDHC_INT_STATUS_BWR   // buffer read ready
#define USDHC_INT_SIGNAL_EN_BRRIEN   USDHC_INT_STATUS_BRR   // buffer write ready
#define USDHC_INT_SIGNAL_EN_CINSIEN  USDHC_INT_STATUS_CINS  // card insertion
#define USDHC_INT_SIGNAL_EN_CRMIEN   USDHC_INT_STATUS_CRM   // card removal
#define USDHC_INT_SIGNAL_EN_CINTIEN  USDHC_INT_STATUS_CINT  // card interrupt
#define USDHC_INT_SIGNAL_EN_RTEIEN   USDHC_INT_STATUS_RTE   // re-tuning event
#define USDHC_INT_SIGNAL_EN_TPIEN    USDHC_INT_STATUS_TP    // tuning pass
#define USDHC_INT_SIGNAL_EN_CTOEIEN  USDHC_INT_STATUS_CTOE  // command timeout error
#define USDHC_INT_SIGNAL_EN_CCEIEN   USDHC_INT_STATUS_CCE   // command CRC error
#define USDHC_INT_SIGNAL_EN_CEBEIEN  USDHC_INT_STATUS_CEBE  // command end bit error
#define USDHC_INT_SIGNAL_EN_CIEIEN   USDHC_INT_STATUS_CIE   // command index error
#define USDHC_INT_SIGNAL_EN_DTOEIEN  USDHC_INT_STATUS_DTOE  // data timeout error
#define USDHC_INT_SIGNAL_EN_DCEIEN   USDHC_INT_STATUS_DCE   // data CRC error
#define USDHC_INT_SIGNAL_EN_DEBEIEN  USDHC_INT_STATUS_DEBE  // data end bit error
#define USDHC_INT_SIGNAL_EN_AC12EIEN USDHC_INT_STATUS_AC12E // auto CMD12 error
#define USDHC_INT_SIGNAL_EN_TNEIEN   USDHC_INT_STATUS_TNE   // tuning error
#define USDHC_INT_SIGNAL_EN_DMAEIEN  USDHC_INT_STATUS_DMAE  // DMA error

#define USDHC_MIX_CTRL_DMAEN        (1 << 0) // DMA enable
#define USDHC_MIX_CTRL_BCEN         (1 << 1) // block count enable
#define USDHC_MIX_CTRL_AC12EN       (1 << 2) // auto CMD12 enable
#define USDHC_MIX_CTRL_DDR_EN       (1 << 3)
#define USDHC_MIX_CTRL_DTDSEL       (1 << 4) // data transfer direction select
#define USDHC_MIX_CTRL_MSBSEL       (1 << 5) // multi/single block select
#define USDHC_MIX_CTRL_NIBBLE_POS   (1 << 6)
#define USDHC_MIX_CTRL_AC23EN       (1 << 7) // auto CMD23 enable
#define USDHC_MIX_CTRL_EXE_TUNE     (1 << 22) // execute tuning
#define USDHC_MIX_CTRL_SMP_CLK_SEL  (1 << 23)
#define USDHC_MIX_CTRL_AUTO_TUNE_EN (1 << 24)
#define USDHC_MIX_CTRL_FBCLK_SEL    (1 << 24)

#define SD_FIFO_WATERMARK 16 // the default

enum class SDCmd : uint8_t
{
  GoIdleState         = 0,
  AllSendCID          = 2,
  SendRelativeAddr    = 3,
  Switch              = 6,
  SelectCard          = 7,
  SendIfCond          = 8,
  SendCSD             = 9,
  SendCID             = 10,
  SendStatus          = 13,
  SetBlockLen         = 16,
  ReadSingleBlock     = 17,
  ReadMultipleBlocks  = 18,
  WriteSingleBlock    = 24,
  WriteMultipleBlocks = 25,
  AppCmd              = 55,
  ReadOCR             = 58,
  CRCOnOff            = 59,

  AppSetBusWidth      = 6,
  AppSendOpCond       = 41
};

enum SDCommandFlags
{
  SDResponse_None = 0,
  SDResponse_R1   = USDHC_CMD_XFR_TYP_RSPTYP_48BIT | USDHC_CMD_XFR_TYP_CCCEN | USDHC_CMD_XFR_TYP_CICEN,
  SDResponse_R1b  = USDHC_CMD_XFR_TYP_RSPTYP_48BIT_BUSY | USDHC_CMD_XFR_TYP_CCCEN | USDHC_CMD_XFR_TYP_CICEN,
  SDResponse_R2   = USDHC_CMD_XFR_TYP_RSPTYP_136BIT | USDHC_CMD_XFR_TYP_CCCEN,
  SDResponse_R3   = 2 << 16, // no CRC/index check
  SDResponse_R7   = SDResponse_R1,

  SD_HasData      = USDHC_CMD_XFR_TYP_DPSEL,

  // these are actually MIX_CTRL bits, but they don't conflict with XFER_TYPE so...
  SD_ReadData     = SD_HasData | USDHC_MIX_CTRL_DTDSEL,
  SD_WriteData    = SD_HasData,
  SD_MultiBlock   = USDHC_MIX_CTRL_BCEN | USDHC_MIX_CTRL_AC12EN | USDHC_MIX_CTRL_MSBSEL,

  // bit 0 = DMA
};

enum class CardState {
  Uninitialised,
  Initialised,
  Error
} card_state = CardState::Uninitialised;

static bool card_inserted = false;
static uint32_t card_size_blocks = 0;
static bool is_hcs = false;
static uint32_t card_rca = 0; // really only the top 16 bits

static bool sd_command(SDCmd cmd, uint32_t param, uint32_t type = SDResponse_None)
{
  // make sure we can send commands
  // TODO: don't always check data bit?
  while(USDHC1_PRES_STATE & (USDHC_PRES_STATE_CDIHB | USDHC_PRES_STATE_CIHB));

  uint32_t xfer_type = static_cast<int>(cmd) << 24 | type;

  USDHC1_CMD_ARG = param;

  // split the "type" bits back into the two registers
  if(type & SD_HasData)
    USDHC1_MIX_CTRL = xfer_type & 0xFFFF;

  USDHC1_CMD_XFR_TYP = xfer_type & 0xFFFF0000;

  //Serial.printf("cmd %i", cmd);

  // wait
  while(!(USDHC1_INT_STATUS & (USDHC_INT_STATUS_CC | USDHC_INT_STATUS_ERR_INT_STATUS)));

  // get/clear status
  uint32_t status = USDHC1_INT_STATUS;
  USDHC1_INT_STATUS = status;

  //Serial.printf(" status %x res %08X\n", status, USDHC1_CMD_RSP0);

  // check done and no errors
  return (status & USDHC_INT_STATUS_CC) && !(status & USDHC_INT_STATUS_ERR_INT_STATUS);
}

// might need to disable card detection when transferring data...
static void sd_read_data(uint8_t *data, int length)
{
  auto data32 = reinterpret_cast<uint32_t *>(data);
  auto remaining = length / sizeof(uint32_t);

  while(remaining)
  {
    while(!(USDHC1_PRES_STATE & USDHC_PRES_STATE_BREN)); // wait

    for(int i = 0; i < SD_FIFO_WATERMARK; i++)
      *data32++ = USDHC1_DATA_BUFF_ACC_PORT;

    remaining -= SD_FIFO_WATERMARK;
  }
}

static void sd_write_data(const uint8_t *data, int length)
{
  auto data32 = reinterpret_cast<const uint32_t *>(data);
  auto remaining = length / sizeof(uint32_t);

  while(remaining)
  {
    while(!(USDHC1_PRES_STATE & USDHC_PRES_STATE_BWEN)); // wait

    for(int i = 0; i < SD_FIFO_WATERMARK; i++)
      USDHC1_DATA_BUFF_ACC_PORT = *data32++;

    remaining -= SD_FIFO_WATERMARK;
  }
}

static void sd_interrupt()
{
  // may need some debouncing on detection
  if(USDHC1_INT_STATUS & USDHC_INT_STATUS_CINS)
  {
    //Serial.println("cins");

    USDHC1_INT_STATUS = USDHC_INT_STATUS_CINS; // clear

    if(!card_inserted)
    {
      card_inserted = true;
      card_state = CardState::Uninitialised;
    }
  }
}

void sd_init()
{
  // configure clocks
  CCM_CCGR6 |= CCM_CCGR6_USDHC1(CCM_CCGR_ON);

  CCM_CSCMR1 &= ~CCM_CSCMR1_USDHC1_CLK_SEL; // PFD2 (396MHz)
  CCM_CSCDR1 = (CCM_CSCDR1 & ~CCM_CSCDR1_USDHC1_PODF(7)) | CCM_CSCDR1_USDHC1_PODF(1); // /2 (198MHz)

  // pins (ALT0)
  CORE_PIN42_CONFIG = 0; // DATA1
  CORE_PIN43_CONFIG = 0; // DATA0
  CORE_PIN44_CONFIG = 0; // CLK
  CORE_PIN45_CONFIG = 0; // CMD
  CORE_PIN46_CONFIG = 0; // DATA3
  CORE_PIN47_CONFIG = 0; // DATA2

  auto pull_up = IOMUXC_SW_PAD_CTL_PAD_PUS(1) | IOMUXC_SW_PAD_CTL_PAD_PUE | IOMUXC_SW_PAD_CTL_PAD_PKE;
  auto pull_down = IOMUXC_SW_PAD_CTL_PAD_PUS(0) | IOMUXC_SW_PAD_CTL_PAD_PUE | IOMUXC_SW_PAD_CTL_PAD_PKE;
  auto strength = IOMUXC_SW_PAD_CTL_PAD_DSE(7); // full strength

  CORE_PIN42_PADCONFIG = pull_up | strength; // DATA1
  CORE_PIN43_PADCONFIG = pull_up | strength; // DATA0
  CORE_PIN44_PADCONFIG = strength; // CLK
  CORE_PIN45_PADCONFIG = pull_up | strength; // CMD
  CORE_PIN46_PADCONFIG = pull_down | strength; // DATA3, needs a pull-down for card detection
  CORE_PIN47_PADCONFIG = pull_up | strength; // DATA2

  // card detection
  USDHC1_PROT_CTRL |= USDHC_PROT_CTRL_D3CD;

  // enable interrupt
  _VectorsRam[IRQ_SDHC1 + 16] = sd_interrupt;
  NVIC_ENABLE_IRQ(IRQ_SDHC1);
  USDHC1_INT_STATUS_EN = 0x157F51FF; // enable all status bits
  USDHC1_INT_SIGNAL_EN = USDHC_INT_SIGNAL_EN_CINSIEN; // signal card insertion
}

static bool init_card()
{
  // reset
  // and set clock to ~386kHz
  USDHC1_SYS_CTRL = USDHC_SYS_CTRL_RSTA | USDHC_SYS_CTRL_SDCLKFS(256) | USDHC_SYS_CTRL_DVS(2) | 0xF/*reserved bits*/;
  while(USDHC1_SYS_CTRL & USDHC_SYS_CTRL_RSTA);

  // init clocks
  USDHC1_SYS_CTRL |= USDHC_SYS_CTRL_INITA;
  while(USDHC1_SYS_CTRL & USDHC_SYS_CTRL_INITA);

  // send CMD0
  if(!sd_command(SDCmd::GoIdleState, 0))
    return false;

  // check version
  bool is_v2 = false;

  if(sd_command(SDCmd::SendIfCond, 0x1AA, SDResponse_R7))
  {
    is_v2 = true;
    if(USDHC1_CMD_RSP0 != 0x1AA)
      return false; // bad response
  }

  // init/voltage range check
  uint32_t param = 3 << 20 | (is_v2 ? (1 << 30) : 0); // ~3.3v, HCS bit if v2
  while(true)
  {
    if(!sd_command(SDCmd::AppCmd, 0, SDResponse_R1))
      return false; // appcmd failed

    if(!sd_command(SDCmd::AppSendOpCond, param, SDResponse_R3))
      return false; // sendoppcond failed

    // check if ready
    if(USDHC1_CMD_RSP0 & (1 << 31))
      break;
  }

  //... and get the OCR
  uint32_t ocr = USDHC1_CMD_RSP0;

  is_hcs = ocr & (1 << 30);

  // address setup (this is new)
  if(!sd_command(SDCmd::AllSendCID, 0, SDResponse_R2))
    return false;

  if(!sd_command(SDCmd::SendRelativeAddr, 0, SDResponse_R1))
    return false;

  card_rca = USDHC1_CMD_RSP0 & 0xFFFF0000;

  // get CSD (need to do before CMD7)
  if(!sd_command(SDCmd::SendCSD, card_rca, SDResponse_R2))
    return false;

  // get data... and reverse the bytes
  uint32_t csd_words[]{
    __builtin_bswap32(USDHC1_CMD_RSP3),
    __builtin_bswap32(USDHC1_CMD_RSP2),
    __builtin_bswap32(USDHC1_CMD_RSP1),
    __builtin_bswap32(USDHC1_CMD_RSP0)
  };

  auto csd = reinterpret_cast<uint8_t *>(csd_words) + 1; // ignore last (first) byte

  // calculate size

  // v1
  if((csd[0] >> 6) == 0)
  {
    int c_size = ((csd[6] & 0x3) << 10) | (csd[7] << 2) | (csd[8] >> 6);
    int c_size_mult =  ((csd[9] & 0x3) << 1) | (csd[10] >> 7);
    int readBlLen = csd[5] & 0xF;

    uint32_t num_blocks = uint32_t(c_size + 1) * (2 << (c_size_mult + 1));
    uint32_t size_bytes = num_blocks * (2 << (readBlLen - 1));
    card_size_blocks = size_bytes / 512;
  }
  else // v2
  {
    // measured in 512k blocks
    card_size_blocks = (((int32_t(csd[7] & 0x3F) << 16) | (uint32_t(csd[8]) << 8) | csd[9]) + 1) * 1024;
  }

  Serial.printf("Detected %s card, size %i blocks (~%iGiB)\n", is_v2 ? (is_hcs ? "SDHC" : "SDv2") : "SDv1", card_size_blocks, card_size_blocks / (1024 * 1024 * 2));

  // select the card
  if(!sd_command(SDCmd::SelectCard, card_rca, SDResponse_R1b))
    return false;

  // switch width (doing this first to see if the read for the speed switch works)
  if(!sd_command(SDCmd::AppCmd, card_rca, SDResponse_R1) || !sd_command(SDCmd::AppSetBusWidth, 2, SDResponse_R1b))
    return false;

  USDHC1_PROT_CTRL |= USDHC_PROT_CTRL_DTW_4BIT;

  // attempt speed change
  USDHC1_BLK_ATT = 64; // set size to 64, count doesn't matter
  // TODO: set FIFO watermark instead of relying on default?

  bool high_speed = false;

  // TODO: check first? (send 0x00FFFFFF, check data[12] & (1 << 1))
  if(sd_command(SDCmd::Switch, 0x80FFFFF1, SDResponse_R1 | SD_ReadData))
  {
    uint8_t switch_res[64];
    sd_read_data(switch_res, 64);

    if((switch_res[16] & 0xF) == 1)
      high_speed = true;
  }

  // reconfigure clocks
  uint32_t clock_mask = USDHC_SYS_CTRL_SDCLKFS_MASK | USDHC_SYS_CTRL_DVS_MASK;
  if(high_speed)
  {
    // 198 / (4 * 1) = 49.5MHz
    USDHC1_SYS_CTRL = (USDHC1_SYS_CTRL & ~clock_mask) | USDHC_SYS_CTRL_SDCLKFS(4);
  }
  else
  {
    // 198 / (8 * 1) = 24.75MHz
    USDHC1_SYS_CTRL = (USDHC1_SYS_CTRL & ~clock_mask) | USDHC_SYS_CTRL_SDCLKFS(8);
  }

  // wait for stable clock
  while(!(USDHC1_PRES_STATE & USDHC_PRES_STATE_SDSTB));

  return true;
}

bool sd_update()
{
  if(card_inserted && card_state == CardState::Uninitialised)
  {
    card_state = init_card() ? CardState::Initialised : CardState::Error;
    return card_state == CardState::Initialised;
  }
  else if(card_inserted)
  {
    if(!(USDHC1_PRES_STATE & USDHC_PRES_STATE_CINST))
    {
      // possibly removed, try a command
      if(!sd_command(SDCmd::SendStatus, card_rca, SDResponse_R1))
        card_inserted = false;
    }
  }

  return false;
}

bool sd_get_initialised()
{
  return card_state == CardState::Initialised;
}

uint32_t sd_get_num_blocks()
{
  return card_size_blocks;
}

static bool wait_ready_for_data()
{
  // wait for card to not be busy
  while(true)
  {
    if(!sd_command(SDCmd::SendStatus, card_rca, SDResponse_R1))
      return false;

    if(USDHC1_CMD_RSP0 & (1 << 8) /*READY_FOR_DATA*/)
      return true;
  }
}

bool sd_read_blocks(uint32_t block, uint8_t *buf, int count)
{
  if(!wait_ready_for_data())
    return false;

  // doesn't matter what we set the count to for single-block
  USDHC1_BLK_ATT = USDHC_BLK_ATT_BLKCNT(count) | USDHC_BLK_ATT_BLKSIZE(512);

  if(count == 1)
  {
    if(!sd_command(SDCmd::ReadSingleBlock, is_hcs ? block : block * 512, SDResponse_R1 | SD_ReadData))
      return false;

    sd_read_data(buf, 512);
  }
  else
  {
    if(!sd_command(SDCmd::ReadMultipleBlocks, is_hcs ? block : block * 512, SDResponse_R1 | SD_ReadData | SD_MultiBlock))
      return false;

    sd_read_data(buf, count * 512);
  }

  return true;
}

bool sd_write_blocks(uint32_t block, const uint8_t *buf, int count)
{
  if(!wait_ready_for_data())
    return false;

  USDHC1_BLK_ATT = USDHC_BLK_ATT_BLKCNT(count) | USDHC_BLK_ATT_BLKSIZE(512);

  if(count == 1)
  {
    if(!sd_command(SDCmd::WriteSingleBlock, is_hcs ? block : block * 512, SDResponse_R1 | SD_WriteData))
      return false;

    sd_write_data(buf, 512);
  }
  else
  {
    if(!sd_command(SDCmd::WriteMultipleBlocks, is_hcs ? block : block * 512, SDResponse_R1 | SD_WriteData | SD_MultiBlock))
      return false;

    sd_write_data(buf, count * 512);
  }

  return true;
}
