#include "nrf24l01.h"

static void ce_set(nrf24l01* dev) {
    HAL_GPIO_WritePin(dev->config.ce_port, dev->config.ce_pin, GPIO_PIN_SET);
}

static void ce_reset(nrf24l01* dev) {
    HAL_GPIO_WritePin(dev->config.ce_port, dev->config.ce_pin, GPIO_PIN_RESET);
}

static void csn_set(nrf24l01* dev) {
    HAL_GPIO_WritePin(dev->config.csn_port, dev->config.csn_pin, GPIO_PIN_SET);
}

static void csn_reset(nrf24l01* dev) {
    HAL_GPIO_WritePin(dev->config.csn_port, dev->config.csn_pin,
                      GPIO_PIN_RESET);
}

/**
 * @brief	Initializes the NRF24 device.
 * @param dev NRF24 device to initialize
 * @param config Configuration struct that contains parameters set once
 * @return 	Communication result.
 */
NRF_RESULT nrf_init(nrf24l01* dev, nrf24l01_config* config) {
    dev->config = *config;

    //testNRF(dev);

    ce_reset(dev); //NRF24 chip in standby mode
    csn_set(dev);  //disable CSN
    //dummy writing (CSN disabled) to get the correct clock polarity
    uint8_t dummyTX = 0xFF;
    uint8_t dummyRX = 0x00;

    if (HAL_SPI_TransmitReceive(dev->config.spi, &dummyTX, &dummyRX, 1,
	                                dev->config.spi_timeout) != HAL_OK) {
	        return NRF_ERROR;
	    }

    /*** Power up sequence ***/
    if(nrf_power_up(dev, true) != NRF_OK)
    	return NRF_ERROR;

    HAL_Delay(2); //2ms delay for 1.5ms power up

    uint8_t config_reg = 0;
    if(nrf_read_register(dev, NRF_CONFIG, &config_reg) != NRF_OK)
    	return NRF_ERROR;
    //Check for configuration register power up bit correctness
    if ((config_reg & 2) == 0)
    	return NRF_ERROR;
    /*** Power up sequence successfully done ***/

    nrf_set_rx_payload_width_p0(dev, dev->config.payload_length);
    nrf_set_rx_payload_width_p1(dev, dev->config.payload_length);

    nrf_set_rx_address_p1(dev, dev->config.rx_address);
    nrf_set_rx_address_p0(dev, dev->config.tx_address);
    nrf_set_tx_address(dev, dev->config.tx_address);

    nrf_enable_rx_data_ready_irq(dev, 1);
    nrf_enable_tx_data_sent_irq(dev, 1);
    nrf_enable_max_retransmit_irq(dev, 1);
    nrf_enable_crc(dev, 1);
    nrf_set_crc_width(dev, dev->config.crc_width);
    nrf_set_address_width(dev, dev->config.addr_width);
    nrf_set_rf_channel(dev, dev->config.rf_channel);
    nrf_set_data_rate(dev, dev->config.data_rate);
    nrf_set_retransmittion_count(dev, dev->config.retransmit_count);
    nrf_set_retransmittion_delay(dev, dev->config.retransmit_delay);

    nrf_set_rx_pipes(dev, 0x03); //enable pipes 0 and 1

    nrf_enable_auto_ack(dev, 0);

    nrf_clear_interrupts(dev);

    nrf_rx_tx_control(dev, NRF_STATE_RX);

    nrf_flush_rx(dev);

    nrf_flush_tx(dev);

    //TODO: improve the portability
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

    ce_set(dev);

    return NRF_OK;
}

/**
 * @brief	Sends a command to the NRF24 using the format <command word>+<data bytes>
 * @param dev	NRF24 device struct
 * @param cmd	Command to send
 * @param tx	Data to transmit (in the order LSByte first) (to check)
 * @param rx	Received data
 * @param len	Length of data to transmit (or receive), excluding the command word.
 * @note	This function both receives and transmits the same amount of data which corresponds
 * 			to the maximum length of the data to be sent o received. In fact, the undesired
 * 			bytes will be discarded both from the NRF24 (see page 49 of Datasheet)
 * 			and from this function.
 * @warning		BEWARE! It uses Variable Length Arrays!
 * @return	Result of the communication.
 */
NRF_RESULT nrf_send_command(nrf24l01* dev, NRF_COMMAND cmd, const uint8_t* tx,
                            uint8_t* rx, uint8_t len) {
    uint8_t myTX[len + 1];
    uint8_t myRX[len + 1];

    myTX[0] = cmd;

    int i = 0;
//    for (i = 0; i < len; i++) {
//        myTX[1 + i] = tx[i];
//        myRX[i]     = 0;
//    }

    //our mod
    myRX[0] = 0;
	for (i = 0; i < len; i++) {
		myTX[1 + i] = tx[i];
		myRX[i + 1]     = 0;
	}

    csn_reset(dev);

    if (HAL_SPI_TransmitReceive(dev->config.spi, myTX, myRX, 1 + len,
                                dev->config.spi_timeout) != HAL_OK) {
        return NRF_ERROR;
    }

    csn_set(dev);

    for (i = 0; i < len; i++)
    {
    	//discard the status register (that is always the first received byte)
    	rx[i] = myRX[1 + i];
    }

    return NRF_OK;
}

void nrf_irq_handler(nrf24l01* dev) {
    uint8_t status = 0;
    if (nrf_read_register(dev, NRF_STATUS, &status) != NRF_OK) { return; }

    if ((status & (1 << 6))) { // RX FIFO Interrupt
        uint8_t fifo_status = 0;
        ce_reset(dev);
        nrf_write_register(dev, NRF_STATUS, &status);
        nrf_read_register(dev, NRF_FIFO_STATUS, &fifo_status);
        if ((fifo_status & 1) == 0) {
            uint8_t* rx_buffer = dev->config.rx_buffer;
            nrf_read_rx_payload(dev, rx_buffer);
            status |= 1 << 6;
            nrf_write_register(dev, NRF_STATUS, &status);
            // nrf_flush_rx(dev);
            nrf_packet_received_callback(dev, rx_buffer);
        }
        ce_set(dev);
    }
    if ((status & (1 << 5))) { // TX Data Sent Interrupt
        status |= 1 << 5;      // clear the interrupt flag
        ce_reset(dev);
        nrf_rx_tx_control(dev, NRF_STATE_RX);
        dev->state = NRF_STATE_RX;
        ce_set(dev);
        nrf_write_register(dev, NRF_STATUS, &status);
        dev->tx_result = NRF_OK;
        dev->tx_busy   = 0;
    }
    if ((status & (1 << 4))) { // MaxRetransmits reached
        status |= 1 << 4;

        nrf_flush_tx(dev);
        nrf_power_up(dev, 0); // power down
        nrf_power_up(dev, 1); // power up

        ce_reset(dev);
        nrf_rx_tx_control(dev, NRF_STATE_RX);
        dev->state = NRF_STATE_RX;
        ce_set(dev);

        nrf_write_register(dev, NRF_STATUS, &status);
        dev->tx_result = NRF_ERROR;
        dev->tx_busy   = 0;
    }
}

__weak void nrf_packet_received_callback(nrf24l01* dev, uint8_t* data) {
    // default implementation (__weak) is used in favor of nrf_receive_packet
    dev->rx_busy = 0;
}

/**
 * @brief	Writes a register (8 bit data)
 * @param dev NRF24 device struct
 * @param reg Address of the register to read
 * @param data 8bit data read from the register
 * @return	Communication result.
 * @note	The command word will contain the register address, then one other data
 * 			byte (dummy) is sent and it will be discarded by the NRF24.
 */
NRF_RESULT nrf_read_register(nrf24l01* dev, uint8_t reg, uint8_t* data) {
    uint8_t tx = 0; //dummy tx data
    if (nrf_send_command(dev, NRF_CMD_R_REGISTER | reg, &tx, data, 1) !=
        NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

/**
 * @brief	Writes a register (8 bit data)
 * @param dev NRF24 device struct
 * @param reg Address of the register to write
 * @param data 8bit data to write to the register
 * @return	Communication result.
 * @note	The command word will contain the register address, then one other data
 * 			byte is sent to be put in that register.
 */
NRF_RESULT nrf_write_register(nrf24l01* dev, uint8_t reg, uint8_t* data) {
    uint8_t rx = 0; //dummy rx buff

    //each register is 8bit wide, so only one data byte needs to be sent (length = 1)
    if (nrf_send_command(dev, NRF_CMD_W_REGISTER | reg, data, &rx, 1) !=
        NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}


NRF_RESULT nrf_read_rx_payload(nrf24l01* dev, uint8_t* data) {
    uint8_t tx[dev->config.payload_length];
    if (nrf_send_command(dev, NRF_CMD_R_RX_PAYLOAD, tx, data,
                         dev->config.payload_length) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT nrf_write_tx_payload(nrf24l01* dev, const uint8_t* data)
{
    uint8_t rx[dev->config.payload_length]; //place for previous payload ACK; not used
    if (nrf_send_command(dev, NRF_CMD_W_TX_PAYLOAD, data, rx,
                         dev->config.payload_length) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT nrf_write_tx_payload_noack(nrf24l01* dev, const uint8_t* data) {
    uint8_t rx[dev->config.payload_length];
    if (nrf_send_command(dev, NRF_CMD_W_TX_PAYLOAD_NOACK, data, rx,
                         dev->config.payload_length) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT nrf_flush_tx(nrf24l01* dev) {
    uint8_t rx = 0;
    uint8_t tx = 0;
    if (nrf_send_command(dev, NRF_CMD_FLUSH_TX, &tx, &rx, 0) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT nrf_flush_rx(nrf24l01* dev) {
    uint8_t rx = 0;
    uint8_t tx = 0;
    if (nrf_send_command(dev, NRF_CMD_FLUSH_RX, &tx, &rx, 0) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT nrf_set_data_rate(nrf24l01* dev, NRF_DATA_RATE rate) {
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_RF_SETUP, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    if (rate & 1) { // low bit set
        reg |= 1 << 5;
    } else { // low bit clear
        reg &= ~(1 << 5);
    }

    if (rate & 2) { // high bit set
        reg |= 1 << 3;
    } else { // high bit clear
        reg &= ~(1 << 3);
    }
    if (nrf_write_register(dev, NRF_RF_SETUP, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    dev->config.data_rate = rate;
    return NRF_OK;
}

NRF_RESULT nrf_set_tx_power(nrf24l01* dev, NRF_TX_PWR pwr) {
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_RF_SETUP, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    reg &= 0xF9;     // clear bits 1,2
    reg |= pwr << 1; // set bits 1,2
    if (nrf_write_register(dev, NRF_RF_SETUP, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    dev->config.tx_power = pwr;
    return NRF_OK;
}

NRF_RESULT nrf_set_ccw(nrf24l01* dev, bool activate) {
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_RF_SETUP, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    if (activate) {
        reg |= 0x80;
    } else {
        reg &= 0x7F;
    }

    if (nrf_write_register(dev, NRF_RF_SETUP, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT nrf_clear_interrupts(nrf24l01* dev) {
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_STATUS, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    reg |= 7 << 4; // setting bits 4,5,6

    if (nrf_write_register(dev, NRF_STATUS, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT nrf_set_rf_channel(nrf24l01* dev, uint8_t ch) {
    ch &= 0x7F;
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_RF_CH, &reg) != NRF_OK) { return NRF_ERROR; }

    reg |= ch; // setting channel

    if (nrf_write_register(dev, NRF_RF_CH, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    dev->config.rf_channel = ch;
    return NRF_OK;
}

NRF_RESULT nrf_set_retransmittion_count(nrf24l01* dev, uint8_t count) {
    count &= 0x0F;
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_SETUP_RETR, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    reg &= 0xF0;  // clearing bits 0,1,2,3
    reg |= count; // setting count

    if (nrf_write_register(dev, NRF_SETUP_RETR, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    dev->config.retransmit_count = count;
    return NRF_OK;
}

NRF_RESULT nrf_set_retransmittion_delay(nrf24l01* dev, uint8_t delay) {
    delay &= 0x0F;
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_SETUP_RETR, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    reg &= 0x0F;       // clearing bits 1,2,6,7
    reg |= delay << 4; // setting delay

    if (nrf_write_register(dev, NRF_SETUP_RETR, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    dev->config.retransmit_delay = delay;
    return NRF_OK;
}

NRF_RESULT nrf_set_address_width(nrf24l01* dev, NRF_ADDR_WIDTH width) {
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_SETUP_AW, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    reg &= 0x03;  // clearing bits 0,1
    reg |= width; // setting delay

    if (nrf_write_register(dev, NRF_SETUP_AW, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    dev->config.addr_width = width;
    return NRF_OK;
}

NRF_RESULT nrf_set_rx_pipes(nrf24l01* dev, uint8_t pipes) {
    if (nrf_write_register(dev, NRF_EN_RXADDR, &pipes) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

//Enable auto ack on selected pipe
NRF_RESULT nrf_enable_auto_ack(nrf24l01* dev, uint8_t pipe)
{
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_EN_AA, &reg) != NRF_OK)
    	return NRF_ERROR;

    reg |= 1 << pipe;

    if (nrf_write_register(dev, NRF_EN_AA, &reg) != NRF_OK)
    {
        return NRF_ERROR;
    }

    //TODO: check new register values
    return NRF_OK;
}

NRF_RESULT nrf_enable_crc(nrf24l01* dev, bool activate) {
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    if (activate) {
        reg |= 1 << 3;
    } else {
        reg &= ~(1 << 3);
    }

    if (nrf_write_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT nrf_set_crc_width(nrf24l01* dev, NRF_CRC_WIDTH width) {
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    if (width == NRF_CRC_WIDTH_2B) {
        reg |= 1 << 2;
    } else {
        reg &= ~(1 << 3);
    }

    if (nrf_write_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    dev->config.crc_width = width;
    return NRF_OK;
}

/**
 * @brief	Writes the power up flag in the NRF24.
 * @param dev NRF24 device struct
 * @param bool Power up flag value. 0 to power down; 1 to power up the device.
 * @return	Communication result.
 */
NRF_RESULT nrf_power_up(nrf24l01* dev, bool power_up) {
    uint8_t reg = 0;

    //get status register
    if (nrf_read_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    if (power_up) {
        reg |= (1 << 1);
    }
    else { //power-up
        reg &= ~(1 << 1);
    }

    //write the new status register
    if (nrf_write_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT nrf_rx_tx_control(nrf24l01* dev, NRF_TXRX_STATE rx) {
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    if (rx) {
        reg |= 1; //Set PRIM_RX flag
    } else {
        reg &= ~(1);
    }

    if (nrf_write_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT nrf_enable_rx_data_ready_irq(nrf24l01* dev, bool activate) {
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    if (!activate) {
        reg |= 1 << 6;
    } else {
        reg &= ~(1 << 6);
    }

    if (nrf_write_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT nrf_enable_tx_data_sent_irq(nrf24l01* dev, bool activate) {
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    if (!activate) {
        reg |= 1 << 5;
    } else {
        reg &= ~(1 << 5);
    }
    if (nrf_write_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT nrf_enable_max_retransmit_irq(nrf24l01* dev, bool activate) {
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    if (!activate) {
        reg |= 1 << 4;
    } else {
        reg &= ~(1 << 4);
    }
    if (nrf_write_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

NRF_RESULT nrf_set_rx_address_p0(nrf24l01* dev, const uint8_t* address) {
    uint8_t rx[5];
    if (nrf_send_command(dev, NRF_CMD_W_REGISTER | NRF_RX_ADDR_P0, address, rx,
                         5) != NRF_OK) {
        return NRF_ERROR;
    }
    dev->config.rx_address = address;
    return NRF_OK;
}

NRF_RESULT nrf_set_rx_address_p1(nrf24l01* dev, const uint8_t* address) {
    uint8_t rx[5];
    if (nrf_send_command(dev, NRF_CMD_W_REGISTER | NRF_RX_ADDR_P1, address, rx,
                         5) != NRF_OK) {
        return NRF_ERROR;
    }
    dev->config.rx_address = address;
    return NRF_OK;
}

NRF_RESULT nrf_set_tx_address(nrf24l01* dev, const uint8_t* address) {
    uint8_t rx[5];
    if (nrf_send_command(dev, NRF_CMD_W_REGISTER | NRF_TX_ADDR, address, rx,
                         5) != NRF_OK) {
        return NRF_ERROR;
    }
    dev->config.tx_address = address;
    return NRF_OK;
}

NRF_RESULT nrf_set_rx_payload_width_p0(nrf24l01* dev, uint8_t width) {
    width &= 0x3F;
    if (nrf_write_register(dev, NRF_RX_PW_P0, &width) != NRF_OK) {
        dev->config.payload_length = 0;
        return NRF_ERROR;
    }
    dev->config.payload_length = width;
    return NRF_OK;
}

NRF_RESULT nrf_set_rx_payload_width_p1(nrf24l01* dev, uint8_t width) {
    width &= 0x3F;
    if (nrf_write_register(dev, NRF_RX_PW_P1, &width) != NRF_OK) {
        dev->config.payload_length = 0;
        return NRF_ERROR;
    }
    dev->config.payload_length = width;
    return NRF_OK;
}

NRF_RESULT nrf_send_packet(nrf24l01* dev, const uint8_t* data)
{

    dev->tx_busy = 1;

    ce_reset(dev); //go back into STANDBY-I
    nrf_rx_tx_control(dev, NRF_STATE_TX);
    nrf_write_tx_payload(dev, data);
    ce_set(dev);

    while (dev->tx_busy == 1) {} // wait for end of transmittion

    return dev->tx_result;
}

NRF_RESULT nrf_send_packet_noack(nrf24l01* dev, const uint8_t* data) {
    dev->tx_busy = 1;

    ce_reset(dev);
    nrf_rx_tx_control(dev, NRF_STATE_TX);
    nrf_write_tx_payload_noack(dev, data);
    ce_set(dev);

    while (dev->tx_busy == 1) {} // wait for end of transmittion

    return dev->tx_result;
}

const uint8_t* nrf_receive_packet(nrf24l01* dev) {

    dev->rx_busy = 1;

    ce_reset(dev);
    nrf_rx_tx_control(dev, NRF_STATE_RX);
    ce_set(dev);

    while (dev->rx_busy == 1) {} // wait for reception

    return dev->config.rx_buffer;
}

NRF_RESULT nrf_push_packet(nrf24l01* dev, const uint8_t* data) {

    if (dev->tx_busy == 1) {
        nrf_flush_tx(dev);
    } else {
        dev->tx_busy = 1;
    }

    ce_reset(dev);
    nrf_rx_tx_control(dev, NRF_STATE_TX);
    nrf_write_tx_payload(dev, data);
    ce_set(dev);

    return NRF_OK;
}

void testNRF(nrf24l01* dev)
{
	uint8_t myTX[2];
	myTX[0] = 0xFF;
	myTX[1] = 0;

	uint8_t myRX[2];
	myRX[0] = 0;
	myRX[1] = 0;

	HAL_SPI_TransmitReceive(dev->config.spi, myTX, myRX, 1, dev->config.spi_timeout);

	//NOP ---> read status
	csn_reset(dev);
	HAL_SPI_TransmitReceive(dev->config.spi, myTX, myRX, 1, dev->config.spi_timeout);
	csn_set(dev);

	//Write register STATUS
	myTX[0] = 0b00100111;
	myTX[1] = 0b00011110;
	csn_reset(dev);
	HAL_SPI_TransmitReceive(dev->config.spi, myTX, myRX, 2, dev->config.spi_timeout);
	csn_set(dev);

	//NOP ---> read status
	myTX[0] = 0xFF;
	myTX[1] = 0;
	csn_reset(dev);
	HAL_SPI_TransmitReceive(dev->config.spi, myTX, myRX, 1, dev->config.spi_timeout);
	csn_set(dev);

	//NOP ---> read status
	myTX[0] = 0xFF;
	myTX[1] = 0;
	csn_reset(dev);
	HAL_SPI_TransmitReceive(dev->config.spi, myTX, myRX, 1, dev->config.spi_timeout);
	csn_set(dev);

}
