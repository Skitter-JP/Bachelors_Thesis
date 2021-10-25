/*
 * USB.h
 *
 * Created: 8/27/2016 6:54:40 AM
 *  Author: Radek
 */ 


#ifndef USB_H_
#define USB_H_


void usb_init(uint8_t gclk_number);

void usb_receive_data(uint8_t endpoint_number);
void usb_receive_bulk_data(void);
void usb_setup_rx_buffer(uint8_t endpoint_number, uint8_t *buffer, uint16_t max_length, void (*callback) (uint8_t,uint16_t));
void usb_setup_rx_bulk_buffer(uint8_t *buffer, uint16_t max_length, void (*callback)(uint8_t,uint16_t));
void usb_send_data(uint8_t endpoint_number, uint8_t *buffer, uint16_t length, void (*callback)(uint8_t));
void usb_send_bulk_data(uint8_t *buffer, uint16_t length, void (*callback)(uint8_t));
uint8_t usb_tx_ready(uint8_t endpoint_number);
uint8_t usb_bulk_tx_ready(void);
void usb_register_set_config_callback(void(*callback)(uint8_t));

void usb_disable(void);


extern volatile uint8_t configuration_enabled;

#endif /* USB_H_ */