/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include "sensor_zssc3224.h"
#include "hal_data.h"

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
#define SENSOR_ZSSC3224_TIMEOUT 1000uL

/***********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Global function prototypes
 **********************************************************************************************************************/
void sensor_zssc3224_callback(spi_callback_args_t *p_args);
/***********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/
static fsp_err_t sensor_zssc3224_delay_us (uint32_t const delay_us);
/***********************************************************************************************************************
 * Private global variables
 **********************************************************************************************************************/
fsp_err_t sensorZSSC3224_write(uint8_t *cmd, uint32_t cmd_length);
fsp_err_t sensorZSSC3224_read(uint8_t *res, uint32_t res_length);
/***********************************************************************************************************************
 * Global variables
 **********************************************************************************************************************/
#if (BSP_CFG_RTOS == 0)
//SPI
volatile bool g_comm_spi_flag;
volatile bool g_transfer_spi_complete;
volatile spi_event_t g_comm_spi_event;
#endif
/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/
fsp_err_t SensorZSSC3224_init(void)
{
	fsp_err_t err = FSP_SUCCESS;
	uint8_t start_CM[] = {0xA9, 0x00, 0x00, 0x00}; // command mode
	uint8_t response[4] = {0};
	uint8_t cmd_config[] = {0x00, 0x02, 0x02};
	uint8_t start_NOM[] = {0xA8, 0x00, 0x00, 0x00}; // normal mode
	uint8_t status_mode;

	err = g_comms_spi_sensor_zssc3224.p_api->open(g_comms_spi_sensor_zssc3224.p_ctrl,
					g_comms_spi_sensor_zssc3224.p_cfg);
	FSP_ERROR_RETURN(FSP_SUCCESS == err, err);

	/*start command mode*/
	err = sensorZSSC3224_write(start_CM, sizeof(start_CM));
	FSP_ERROR_RETURN(FSP_SUCCESS == err, err);

	/*check command mode bit[4:3] table 6.3 data sheet*/
	err = sensorZSSC3224_read(response, sizeof(response));
	FSP_ERROR_RETURN(FSP_SUCCESS == err, err);

	status_mode = (response[0] >> 3) & 0x03;
	if(status_mode != 0x01) {
		return FSP_ERR_MODE_FAULT;
	}
	/*Configuration sensor*/
	err = sensorZSSC3224_write(cmd_config, sizeof(cmd_config));
	FSP_ERROR_RETURN(FSP_SUCCESS == err, err);

	/*Exit command mode and transition to normal mode*/
	err = sensorZSSC3224_write(start_NOM, sizeof(start_NOM));
	FSP_ERROR_RETURN(FSP_SUCCESS == err, err);

	/*check command mode bit[4:3] table 6.3 data sheet*/
	err = sensorZSSC3224_read(response, sizeof(response));
	FSP_ERROR_RETURN(FSP_SUCCESS == err, err);

	status_mode = (response[0] >> 3) & 0x03;
	if(status_mode != 0x00) {
		return FSP_ERR_MODE_FAULT;
	}
	return FSP_SUCCESS;
}
fsp_err_t SensorZSSC3224_MeasurementData(void)
{
	fsp_err_t err;
	uint8_t cmd_measure[] = {0xAA, 0x00, 0x00};
	uint8_t respond_measure[3] = {0};

	/*WRITE command measurement */
	err = sensorZSSC3224_write(cmd_measure, sizeof(cmd_measure));
	FSP_ERROR_RETURN(FSP_SUCCESS == err, err);

	/*Check respond measurement*/
	err = sensorZSSC3224_read(respond_measure, sizeof(respond_measure));
	FSP_ERROR_RETURN(FSP_SUCCESS == err, err);
	//uint8_t status = respond_measure[0];
	//uint32_t data = (respond_measure[1] << 8) | respond_measure[2];

	return FSP_SUCCESS;
}
fsp_err_t SensorZSSC3224_Get_RawData(uint8_t *p_data, uint32_t data_length)
{
	fsp_err_t err;
	uint8_t cmd_NOP[] = {0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // NOP command full measurement

	/*Command = NOP order to READ data*/
	err = sensorZSSC3224_write(cmd_NOP, sizeof(cmd_NOP));
	FSP_ERROR_RETURN(FSP_SUCCESS == err, err);

	/*READ measurement*/
	err = sensorZSSC3224_read(p_data, data_length);
	FSP_ERROR_RETURN(FSP_SUCCESS == err, err);

	return FSP_SUCCESS;
}

/***********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/

/*******************************************************************************************************************//**
 * @brief Write data from Sensor ZSSC3224 device.
 *
 * @retval FSP_SUCCESS              Successfully started.
 * @retval FSP_ERR_TIMEOUT          communication is timeout.
 * @retval FSP_ERR_ABORTED          communication is aborted.
 **********************************************************************************************************************/
fsp_err_t sensorZSSC3224_write(uint8_t *cmd, uint32_t cmd_length)
{
	fsp_err_t err = FSP_SUCCESS;

	err = g_comms_spi_sensor_zssc3224.p_api->write(g_comms_spi_sensor_zssc3224.p_ctrl, cmd, cmd_length, SPI_BIT_WIDTH_8_BITS);
	FSP_ERROR_RETURN(FSP_SUCCESS == err, err);
#if (BSP_CFG_RTOS == 0)
	uint16_t counter = 0;
	/* Start a read transfer */
	g_transfer_spi_complete = false;
  /* Wait callback event. */
	while (false == g_transfer_spi_complete)
	{
		sensor_zssc3224_delay_us(1000);
		counter++;
		FSP_ERROR_RETURN(SENSOR_ZSSC3224_TIMEOUT >= counter, FSP_ERR_TIMEOUT);
	}
    /* Check callback event */
    FSP_ERROR_RETURN(SPI_EVENT_TRANSFER_COMPLETE == g_comm_spi_event, FSP_ERR_ABORTED);
#endif
	return FSP_SUCCESS;

}
/*******************************************************************************************************************//**
 * @brief Read data from Sensor ZSSC3224 device.
 *
 * @retval FSP_SUCCESS              Successfully started.
 * @retval FSP_ERR_TIMEOUT          communication is timeout.
 * @retval FSP_ERR_ABORTED          communication is aborted.
 **********************************************************************************************************************/
fsp_err_t sensorZSSC3224_read(uint8_t *res, uint32_t res_length)
{
	fsp_err_t err = FSP_SUCCESS;

	err = g_comms_spi_sensor_zssc3224.p_api->read(g_comms_spi_sensor_zssc3224.p_ctrl, res, res_length, SPI_BIT_WIDTH_8_BITS);
	FSP_ERROR_RETURN(FSP_SUCCESS == err, err);
#if (BSP_CFG_RTOS == 0)
	uint16_t counter = 0;
	/* Start a read transfer */
	g_transfer_spi_complete = false;
  /* Wait callback event. */
	while (false == g_transfer_spi_complete)
	{
		sensor_zssc3224_delay_us(1000);
		counter++;
		FSP_ERROR_RETURN(SENSOR_ZSSC3224_TIMEOUT >= counter, FSP_ERR_TIMEOUT);
	}
    /* Check callback event */
    FSP_ERROR_RETURN(SPI_EVENT_TRANSFER_COMPLETE == g_comm_spi_event, FSP_ERR_ABORTED);
#endif
	return FSP_SUCCESS;

}

/*******************************************************************************************************************//**
 * @brief callback function called in the SPI Communications Middleware callback function.
 **********************************************************************************************************************/
void sensor_zssc3224_callback(spi_callback_args_t *p_args)
{
#if (BSP_CFG_RTOS > 0)
    FSP_PARAMETER_NOT_USED(p_args);
#else
    switch (p_args->event)
    {
        case SPI_EVENT_TRANSFER_COMPLETE:
            g_transfer_spi_complete = true;
            g_comm_spi_event = SPI_EVENT_TRANSFER_COMPLETE;
            break;

        case SPI_EVENT_ERR_MODE_FAULT:
        case SPI_EVENT_ERR_READ_OVERFLOW:
        case SPI_EVENT_ERR_PARITY:
        case SPI_EVENT_ERR_OVERRUN:
        case SPI_EVENT_ERR_FRAMING:
        case SPI_EVENT_ERR_MODE_UNDERRUN:
        	/* SPI errors */
            g_comm_spi_event = p_args->event;
            g_comm_spi_flag = true;
            break;

        default:
            break;
    }
#endif
}

/*******************************************************************************************************************//**
 * @brief Delay some microseconds.
 *
 * @retval FSP_SUCCESS              successfully configured.
 **********************************************************************************************************************/
static fsp_err_t sensor_zssc3224_delay_us (uint32_t const delay_us)
{
    /* Software delay */
    R_BSP_SoftwareDelay(delay_us, BSP_DELAY_UNITS_MICROSECONDS);

    return FSP_SUCCESS;
}


