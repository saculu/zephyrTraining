#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

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

/* ============ Register Unions with Bitfields ============ */

/* ERRFL Register (0x0001) - Error flags */
typedef union {
    uint16_t raw;
    struct {
        uint16_t frerr   : 1;  /* Bit 0: Framing error is set to 1 when a non-compliant SPI
frame is detected */
        uint16_t invcomm : 1;  /* Bit 1: Invalid command */
        uint16_t parerr  : 1;  /* Bit 2: Parity error */
        uint16_t reserved: 13; /* Bits 3-15: Reserved */
    } bits;
} errfl_reg_t;

/* PROG Register (0x0003) - Programming control */
typedef union {
    uint16_t raw;
    struct {
        uint16_t progen    : 1;  /* Bit 0: Program OTP enable: enables reading / writing the OTP memory */
        uint16_t reserved1 : 1;  /* Bit 1: Reserved */
        uint16_t otpref    : 1;  /* Bit 2: Refreshes the non-volatile memory content with the OTP programmed content */
        uint16_t progotp   : 1;  /* Bit 3: Start OTP programming cyclee */
        uint16_t reserved2 : 2;  /* Bits 4-5: Reserved */
        uint16_t progver   : 1;  /* Bit 6: Verify OTP programming */
        uint16_t reserved3 : 9;  /* Bits 7-15: Reserved */
    } bits;
} prog_reg_t;

/* DIAAGC Register (0x3FFC) - Diagnostics and AGC */
typedef union {
    uint16_t raw;
    struct {
        uint16_t agc  : 8;  /* Bits 7:0: Automatic gain control value */
        uint16_t lf   : 1;  /* Bit 8: Loops finished (0=not ready, 1=finished) */
        uint16_t cof  : 1;  /* Bit 9: CORDIC overflow */
        uint16_t magh : 1;  /* Bit 10: Magnetic field too high (AGC=0x00) */
        uint16_t magl : 1;  /* Bit 11: Magnetic field too low (AGC=0xFF) */
        uint16_t reserved : 4; /* Bits 12-15: Reserved/unused */
    } bits;
} diaagc_reg_t;

/* MAG Register (0x3FFD) - CORDIC magnitude */
typedef union {
    uint16_t raw;
    struct {
        uint16_t cmag : 14; /* Bits 0-13: CORDIC magnitude */
        uint16_t reserved : 2; /* Bits 14-15: Reserved */
    } bits;
} mag_reg_t;

/* ANGLE Register (0x3FFE/0x3FFF) - Angle value */
typedef union {
    uint16_t raw;
    struct {
        uint16_t angle : 14; /* Bits 0-13: Angle value (14-bit) */
        uint16_t reserved : 2; /* Bits 14-15: Reserved */
    } bits;
} angle_reg_t;

/* SETTINGS1 Register (0x0018) - Configuration settings 1 */
typedef union {
    uint16_t raw;
    struct {
        uint16_t iwidth    : 1;  /* Bit 0: Index pulse width (0=3LSB, 1=1LSB) */
        uint16_t noiseset  : 1;  /* Bit 1: Noise setting */
        uint16_t dir       : 1;  /* Bit 2: Rotation direction */
        uint16_t uvw_abi   : 1;  /* Bit 3: PWM output (0=ABI+W as PWM, 1=UVW+I as PWM) */
        uint16_t daecdis   : 1;  /* Bit 4: Disable DAE compensation (0=ON, 1=OFF) */
        uint16_t reserved1 : 1;  /* Bit 5: Reserved */
        uint16_t dataselect: 1;  /* Bit 6: Data select (0=DAECANG, 1=CORDICANG) */
        uint16_t pwmon     : 1;  /* Bit 7: Enable PWM (requires UVW_ABI setting) */
        uint16_t reserved2 : 8;  /* Bits 8-15: Reserved */
    } bits;
} settings1_reg_t;



static void print_settings1(settings1_reg_t reg)
{
    LOG_INF("SETTINGS1: 0x%04X", reg.raw);
    LOG_INF("  IWIDTH     [0]: %u - Index pulse width (%s)", 
            reg.bits.iwidth, reg.bits.iwidth ? "1 LSB" : "3 LSB");
    LOG_INF("  NOISESET   [1]: %u - Noise setting", reg.bits.noiseset);
    LOG_INF("  DIR        [2]: %u - Rotation direction", reg.bits.dir);
    LOG_INF("  UVW_ABI    [3]: %u - PWM output (%s)", 
            reg.bits.uvw_abi, reg.bits.uvw_abi ? "UVW, I=PWM" : "ABI, W=PWM");
    LOG_INF("  DAECDIS    [4]: %u - DAE compensation (%s)", 
            reg.bits.daecdis, reg.bits.daecdis ? "OFF" : "ON");
    LOG_INF("  DATASELECT [6]: %u - Data at 0x3FFF (%s)", 
            reg.bits.dataselect, reg.bits.dataselect ? "CORDICANG" : "DAECANG");
    LOG_INF("  PWMON      [7]: %u - PWM (%s)", 
            reg.bits.pwmon, reg.bits.pwmon ? "enabled" : "disabled");
}

/* ======================================================= */



/* encSPI uses 16-bit SPI frames */
/* encspi uses SPI Mode 1: CPOL=0, CPHA=1 */
#define SPIOP      (SPI_WORD_SET(16) | SPI_TRANSFER_MSB | SPI_MODE_CPHA)

struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(encspi), SPIOP);

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
int th_encSpi(void *, void *, void *);

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

/* encSPI write register - sends write command with data */
static int encspi_write_reg(uint16_t reg_addr, uint16_t data)
{
	int err;
	uint16_t rx_data = 0;

	/* Build write command: bit15=parity, bit14=0 (write), bits13-0=address */
	uint16_t tx_cmd = encspi_cmd(reg_addr, false);  /* false = write */

	struct spi_buf tx_buf = {.buf = &tx_cmd, .len = sizeof(tx_cmd)};
	struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};
	struct spi_buf rx_buf = {.buf = &rx_data, .len = sizeof(rx_data)};
	struct spi_buf_set rx_buf_set = {.buffers = &rx_buf, .count = 1};

	/* First transaction: send write command (address) */
	err = spi_transceive_dt(&spispec, &tx_buf_set, &rx_buf_set);
	if (err < 0) {
		LOG_ERR("spi_transceive_dt() failed, err: %d", err);
		return err;
	}

	/* Second transaction: send data with parity */
	/* Data frame: bit15=parity, bit14=0, bits13-0=data */
	tx_cmd = data & 0x3FFF;  /* 14-bit data */
	if (calc_even_parity(tx_cmd)) {
		tx_cmd |= (1 << 15);  /* Set parity bit */
	}

	err = spi_transceive_dt(&spispec, &tx_buf_set, &rx_buf_set);
	if (err < 0) {
		LOG_ERR("spi_transceive_dt() failed, err: %d", err);
		return err;
	}

	LOG_DBG("Write reg 0x%04X = 0x%04X", reg_addr, data);

	return 0;
}

/* Read all non-volatile registers using individual reads */
static int encspi_read_nv_registers_internal(void)
{
	uint16_t zposm;
	uint16_t zposl;
	uint16_t settings1;
	uint16_t settings2;
	uint16_t red;

    LOG_INF( "if using nucleo power the board with 3.3V");
    
	if (encspi_read_reg(ENCSPI_ZPOSM, &zposm) != 0) return -1;
	if (encspi_read_reg(ENCSPI_ZPOSL, &zposl) != 0) return -1;
	if (encspi_read_reg(ENCSPI_SETTINGS1, &settings1) != 0) return -1;
	if (encspi_read_reg(ENCSPI_SETTINGS2, &settings2) != 0) return -1;
	if (encspi_read_reg(ENCSPI_RED, &red) != 0) return -1;

	LOG_INF("=== NV Registers ===");
    LOG_INF("ZPOSM:     0x%04X", zposm);
    LOG_INF("ZPOSL:     0x%04X", zposl);
    LOG_INF("SETTINGS1: 0x%04X", settings1);
    LOG_INF("SETTINGS2: 0x%04X", settings2);
    LOG_INF("RED:       0x%04X", red);

	return 0;
}

/* Shell wrapper for encspi_read_nv_registers */
static int encspi_read_nv_registers(const struct shell *sh, size_t argc, char **argv)
{
	return encspi_read_nv_registers_internal();
}


/* Set I/PWM pin to Index pulse mode (pulse at zero position) */
static int encspi_set_index(const struct shell *sh, size_t argc, char **argv)
{
    shell_print(sh, "if using nucleo power the board with 3.3V");
    settings1_reg_t settings1;
    if (encspi_read_reg(ENCSPI_SETTINGS1, &settings1.raw) != 0) {
        LOG_ERR("Failed to read SETTINGS1 register");
        return -1;
    }
    settings1.bits.uvw_abi = 0; // abi operating , w used as pwm
    settings1.bits.pwmon = 1;   // enable pwm
    if (encspi_write_reg(ENCSPI_SETTINGS1, settings1.raw) != 0) {
        LOG_ERR("Failed to write SETTINGS1 register");
        return -1;
    }
    LOG_INF("I/PWM pin set to Index pulse mode");
    print_settings1(settings1);
    return 0;
}

/*Program OTP*/
static int encspi_program_otp(const struct shell *sh, size_t argc, char **argv)
{
    prog_reg_t prog;
    // Enable programming
    shell_print(sh, "Starting OTP programming...");
    if (encspi_read_reg(ENCSPI_PROG, &prog.raw) != 0) {
        shell_print(sh, "Failed to read PROG register");
        return -1;
    }
    shell_print(sh, "PROG before: 0x%04X", prog.raw);
    prog.raw = 0; // Clear PROG register
    shell_print(sh, "Enabling programming...");
    prog.bits.progen = 1; // Enable programming
    if (encspi_write_reg(ENCSPI_PROG, prog.raw) != 0) {
        shell_print(sh, "Failed to write PROG register");
        return -1;
    }

    // Start OTP programming
    prog.raw = 0; // Clear PROG register
    prog.bits.progotp = 1; // Start programming
    if (encspi_write_reg(ENCSPI_PROG, prog.raw) != 0) {
        shell_print(sh, "Failed to write PROG register for OTP programming");
        return -1;
    }

    shell_print(sh, "OTP programming started");
    // Wait for programming to complete (poll PROG register)
    do {
        k_msleep(100); // Delay between polls
        if (encspi_read_reg(ENCSPI_PROG, &prog.raw) != 0) {
            shell_print(sh, "Failed to read PROG register during OTP programming");
            return -1;
        }
        shell_print(sh, "PROG status: 0x%04X", prog.raw);
    } while (prog.raw != 0x01);

    shell_print(sh, "OTP programming completed - run encspi_otp_finish to finalize");
    return 0;
}

/* Finish OTP - disable PROGEN and refresh OTP content */
static int encspi_otp_finish(const struct shell *sh, size_t argc, char **argv)
{
    prog_reg_t prog;
    
    // Read current PROG state
    if (encspi_read_reg(ENCSPI_PROG, &prog.raw) != 0) {
        shell_print(sh, "Failed to read PROG register");
        return -1;
    }
    shell_print(sh, "PROG before: 0x%04X", prog.raw);

    // clear the the non-volatile memory content with the OTP programmed content
    shell_print(sh, "Clearing non-volatile memory content with OTP programmed content...");
    if (encspi_write_reg(ENCSPI_ZPOSM, 0x00) != 0) {
        shell_print(sh, "Failed to clear ZPOSM register");
        return -1;
    }
    if (encspi_write_reg(ENCSPI_ZPOSL, 0x00) != 0) {
        shell_print(sh, "Failed to clear ZPOSL register");
        return -1;
    }
    if (encspi_write_reg(ENCSPI_SETTINGS1, 0x00) != 0) {
        shell_print(sh, "Failed to clear SETTINGS1 register");
        return -1;
    }
    if (encspi_write_reg(ENCSPI_SETTINGS2, 0x00) != 0) {
        shell_print(sh, "Failed to clear SETTINGS2 register");
        return -1;
    }

    // Set Guarband to disable PROGEN
    shell_print(sh, "setguarband.");
    if (encspi_write_reg(ENCSPI_PROG, 0x40) != 0) {
        shell_print(sh, "Failed to write PROG register");
        return -1;
    }

    //set otpref to refresh
    shell_print(sh, "Triggering OTP refresh...");
    prog.bits.otpref = 1;
    if (encspi_write_reg(ENCSPI_PROG, prog.raw) != 0) {
        shell_print(sh, "Failed to trigger OTP refresh");
        return -1;
    }
    k_msleep(10);



    shell_print(sh, "OTP refresh complete. Power off the board");
    shell_print(sh, "Power on the board and check settings parameters to verify OTP programming was successful.");

    return 0;
}


/* thread */
int th_encSpi(void *arg1, void *arg2, void *arg3)
{   
    LOG_DBG("SPI Encoder Thread Started");
    err = spi_is_ready_dt(&spispec);
    if (!err) {
        LOG_ERR("Error: SPI device is not ready, err: %d", err);
    }

    /* Read all non-volatile registers at startup */
    encspi_read_nv_registers_internal();

    // Main loop can be used for periodic tasks or left empty if using shell commands for interaction
    // shell thread has by default has priority 0, which is higher than this thread, so shell commands will preempt this thread when invoked
    
    // while (1) {
    //     // Implement SPI communication with the encoder
    //     k_msleep(1000); // Adjust sleep time as necessary
    // }
    // finish this thread since all interactions are done via shell commands
    return 0;
}




/*SHELL*/

SHELL_CMD_REGISTER(encspi_read_nv, NULL, "Read encSPI NV registers", encspi_read_nv_registers);
SHELL_CMD_REGISTER(encspi_set_index, NULL, "Set I/PWM pin to Index pulse mode", encspi_set_index);
SHELL_CMD_REGISTER(encspi_program_otp, NULL, "Program encSPI OTP", encspi_program_otp);
SHELL_CMD_REGISTER(encspi_otp_finish, NULL, "Finish OTP (disable PROGEN, refresh)", encspi_otp_finish);