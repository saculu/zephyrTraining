#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(spi_enc);

/* Thread stack size & priority */
#define STACK_SIZE   4096
#define PRIORITY     5

/* encSPI Volatile Register Addresses (14-bit) */
#define ENCSPI_NOP      0x0000U
#define ENCSPI_ERRFL    0x0001U
#define ENCSPI_PROG     0x0003U
#define ENCSPI_DIAAGC   0x3FFCU
#define ENCSPI_MAG      0x3FFDU
#define ENCSPI_ANGLEUNC 0x3FFEU
#define ENCSPI_ANGLECOM 0x3FFFU

/*encSPI Non Volatile Register Addresses (14-bit) */
#define ENCSPI_ZPOSM     0x0016U
#define ENCSPI_ZPOSL     0x0017U
#define ENCSPI_SETTINGS1 0x0018U
#define ENCSPI_SETTINGS2 0x0019U
#define ENCSPI_RED       0x001AU

/* encSPI uses 16-bit SPI frames */
/* encspi uses SPI Mode 1: CPOL=0, CPHA=1 */
#define SPIOP      (SPI_WORD_SET(16) | SPI_TRANSFER_MSB | SPI_MODE_CPHA)

struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(encspi), SPIOP, 0);

/* Calculate even parity for encSPI command frame */
static uint8_t calc_even_parity(uint16_t value)
{
    uint8_t cnt = 0;
    while (value) {
        cnt += value & 1;
        value >>= 1;
    }
    return cnt & 1;
}

/* Build encSPI command frame: bit15=parity, bit14=R/W (1=read), bits13-0=address */
static uint16_t encspi_cmd(uint16_t reg_addr, bool read)
{
    uint16_t cmd = (reg_addr & 0x3FFF);  /* 14-bit address */
    if (read) {
        cmd |= (1 << 14);  /* Set read bit */
    }
    /* Calculate parity over bits 0-14 and set bit 15 */
    if (calc_even_parity(cmd)) {
        cmd |= (1 << 15);
    }
    return cmd;
}

int err;

/* Thread function prototypes */
void th_encSpi(void *, void *, void *);

K_THREAD_DEFINE(encSpi, STACK_SIZE, th_encSpi, NULL, NULL, NULL,
		    PRIORITY, 0, 0);

/* encSPI read register - sends command in one frame, reads response in next */
static int encspi_read_reg(uint16_t reg_addr, uint16_t *data)
{
	int err;
	uint16_t tx_cmd = encspi_cmd(reg_addr, true);  /* Build read command */
	uint16_t rx_data = 0;

	struct spi_buf tx_buf = {.buf = &tx_cmd, .len = sizeof(tx_cmd)};
	struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};
	struct spi_buf rx_buf = {.buf = &rx_data, .len = sizeof(rx_data)};
	struct spi_buf_set rx_buf_set = {.buffers = &rx_buf, .count = 1};

	/* First transaction: send command, ignore response (it's from previous command) */
	err = spi_transceive_dt(&spispec, &tx_buf_set, &rx_buf_set);
	if (err < 0) {
		LOG_ERR("spi_transceive_dt() failed, err: %d", err);
		return err;
	}

	/* Second transaction: send NOP to read the actual response */
	tx_cmd = encspi_cmd(ENCSPI_NOP, true);
	err = spi_transceive_dt(&spispec, &tx_buf_set, &rx_buf_set);
	if (err < 0) {
		LOG_ERR("spi_transceive_dt() failed, err: %d", err);
		return err;
	}

	/* Response format: bit15=parity, bit14=error flag, bits13-0=data */
	*data = rx_data & 0x3FFF;  /* Extract 14-bit data */

	return 0;
}

/* Read all non-volatile registers using individual reads */
static int encspi_read_nv_registers(void)   
{
	uint16_t zposm, zposl, settings1, settings2, red;

	if (encspi_read_reg(ENCSPI_ZPOSM, &zposm) != 0) return -1;
	if (encspi_read_reg(ENCSPI_ZPOSL, &zposl) != 0) return -1;
	if (encspi_read_reg(ENCSPI_SETTINGS1, &settings1) != 0) return -1;
	if (encspi_read_reg(ENCSPI_SETTINGS2, &settings2) != 0) return -1;
	if (encspi_read_reg(ENCSPI_RED, &red) != 0) return -1;

	LOG_INF("NV Registers:");
	LOG_INF("ZPOSM: 0x%04X", zposm);
	LOG_INF("ZPOSL: 0x%04X", zposl);
	LOG_INF("SETTINGS1: 0x%04X", settings1);
	LOG_INF("SETTINGS2: 0x%04X", settings2);
	LOG_INF("RED: 0x%04X", red);

	return 0;
}

void th_encSpi(void *arg1, void *arg2, void *arg3)
{   
    LOG_DBG("SPI Encoder Thread Started");
    err = spi_is_ready_dt(&spispec);
    if (!err) {
        LOG_ERR("Error: SPI device is not ready, err: %d", err);
    }

    uint16_t reg_value;

    // SPI encoder handling code goes here
    while (1) {

        if (encspi_read_reg(ENCSPI_ERRFL, &reg_value) == 0) {
            LOG_INF("ERRFL: 0x%04X", reg_value);
        }

        if (encspi_read_reg(ENCSPI_PROG, &reg_value) == 0) {
            LOG_INF("PROG: 0x%04X", reg_value);
        }

        if (encspi_read_reg(ENCSPI_DIAAGC, &reg_value) == 0) {
            LOG_INF("DIAAGC: 0x%04X", reg_value);
        }

        if (encspi_read_reg(ENCSPI_MAG, &reg_value) == 0) {
			LOG_INF("MAG: 0x%04X", reg_value);
		}

        if (encspi_read_reg(ENCSPI_ANGLECOM, &reg_value) == 0) {
            /* Convert to degrees: (value * 360) / 16384 */
            uint32_t degrees_x100 = ((uint32_t)reg_value * 36000) / 16384;
            LOG_INF("ANGLE: 0x%04X (degrees: %u.%02u)", 
                    reg_value, degrees_x100 / 100, degrees_x100 % 100);
        }

        /* Read all non-volatile registers in sequence */

         encspi_read_nv_registers();


        // Implement SPI communication with the encoder
        k_msleep(1000); // Adjust sleep time as necessary
    }
}