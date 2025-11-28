#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(spi_enc);

/* Thread stack size & priority */
#define STACK_SIZE   4096
#define PRIORITY     5

#define NOP     0x00U
#define ERRFL   0x01U
#define PROG     0x03U

#define SPIOP      SPI_WORD_SET(8) | SPI_TRANSFER_MSB

struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(encspi), SPIOP, 0);

int err;

/* Thread function prototypes */
void th_encSpi(void *, void *, void *);

K_THREAD_DEFINE(encSpi, STACK_SIZE, th_encSpi, NULL, NULL, NULL,
		    PRIORITY, 0, 0);

static int bme_read_reg(uint8_t reg, uint8_t *data, uint8_t size)
{
	int err;

	/* STEP 4.1 - Set the transmit and receive buffers */
	uint8_t tx_buffer = reg;
	struct spi_buf tx_spi_buf			= {.buf = (void *)&tx_buffer, .len = 1};
	struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};
	struct spi_buf rx_spi_bufs 			= {.buf = data, .len = size};
	struct spi_buf_set rx_spi_buf_set	= {.buffers = &rx_spi_bufs, .count = 1};

	/* STEP 4.2 - Call the transceive function */
	err = spi_transceive_dt(&spispec, &tx_spi_buf_set, &rx_spi_buf_set);
	if (err < 0) {
		LOG_ERR("spi_transceive_dt() failed, err: %d", err);
		return err;
	}

	return 0;
}

void th_encSpi(void *arg1, void *arg2, void *arg3)
{   
    LOG_DBG("SPI Encoder Thread Started");
    err = spi_is_ready_dt(&spispec);
    if (!err) {
        LOG_ERR("Error: SPI device is not ready, err: %d", err);
    }

    uint8_t values[2];
    uint8_t size = 2;

    // SPI encoder handling code goes here
    while (1) {

        bme_read_reg(ERRFL, values, size);
        printk("SPI ERRFL Register: 0x%02X%02X\n", values[0], values[1]);
        // Implement SPI communication with the encoder
        k_msleep(100); // Adjust sleep time as necessary
    }
}