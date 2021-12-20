
#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/examples/i2c_master_example.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define pinDIN   LATCbits.LATC1
#define pinLOAD  LATCbits.LATC2  
#define pinCLK   LATCbits.LATC0

#define BTN_MODE      PORTCbits.RC3
#define BTN_UP        PORTCbits.RC4
#define BTN_DW        PORTCbits.RC5

#define DECODE_MODE     0x09
#define INTENSITY       0x0A
#define SCAN_LIMIT      0x0B
#define SHUTDOWN        0x0C
#define DISPLAY_TEST    0x0F
#define Press    0
#define Release  1
#define DS18B20_PIN      PORTAbits.RA2
#define DS18B20_PIN_Dir  TRISAbits.TRISA2
#define ADDRESS_DS1307   0x68

#define secondCurrent 0x55;
#define minuteCurrent 0x01;
#define hourCurrent   0x02;
#define MA_DS         0x98
unsigned char CONTROL_DS, MA_DS1307;

uint8_t minute, hour, i = 0, second ;
uint8_t count = 0;
uint8_t  hour_dozens = 0,   hour_unitRow = 0;
uint8_t  minute_dozens = 0, minute_unitRow = 0;
uint16_t raw_temp;
uint16_t int_part = 0;
unsigned char flag_sys = 0;
unsigned int sys_cnt = 0;

void THIET_LAP_THOI_GIAN_HIEN_TAI(){
    second = secondCurrent;
    minute = minuteCurrent;
    hour   = hourCurrent;
    CONTROL_DS = 0x90;
    MA_DS1307  = MA_DS;
    
}

void NAP_TGIAN_HIEM_TAI(){
    I2C_Write1ByteRegister(ADDRESS_DS1307, 0x00,second);
    I2C_Write1ByteRegister(ADDRESS_DS1307, 0x01,minute);    
    I2C_Write1ByteRegister(ADDRESS_DS1307, 0x02,hour);
}


unsigned char digits[] = {
    // XABCDEFG
    0b01111110, // 0
    0b00110000, // 1
    0b01101101, // 2
    0b01111001, // 3
    0b00110011, // 4
    0b01011011, // 5
    0b01011111, // 6
    0b01110000, // 7
    0b01111111, // 8
    0b01111011, // 9
//    0b01000000,
//    0b00100000,
//    0b00000001,
//    0b00000100,
//    0b00001000,
    0b01100011,// do
    0b01001110 // C
   
};
void putch(char value){
    while(!EUSART_is_tx_ready());
    EUSART_Write(value);
}

void WriteBits(unsigned int data){
    pinLOAD = 0;
    for(unsigned int i = 0x8000; i; i = i >> 1){
        if((data & i) == 0)    pinDIN = 0;
        else                   pinDIN = 1;        
        // nhip xung
        pinCLK = 1;
        __delay_us(1);
        pinCLK = 0;     
    }
   
    pinLOAD = 1; 
    
}
void Write_Comand(unsigned char Address, unsigned char Data){
    unsigned int temp = 0;
    temp = Address;
    temp = temp << 8;
    temp = temp & 0x0F00; // D11 - D8
    temp = temp + Data;
    WriteBits(temp);
   
}
void Write_Digits(unsigned char Position, unsigned char valueDigits){
    
    unsigned int temp = 0;
    temp = Position + 1;
    temp = temp << 8;
    temp = temp & 0x0F00;
    temp = temp + digits[valueDigits];
    WriteBits(temp); 
}
void MAX7219_CLEAR(){
    for(unsigned char j = 0; j < 4;j++){
        Write_Digits(j,0x0F);    
    }
}

void MAX7219_Init(){
    // set output for pins max7219
    TRISCbits.TRISC0 = 0;
    TRISCbits.TRISC1 = 0;
    TRISCbits.TRISC2 = 0;
    
    pinDIN  = 0;
    pinLOAD = 0;
    pinCLK  = 0;        
    Write_Comand(DISPLAY_TEST,0x00);
    Write_Comand(INTENSITY,0x0F);
    Write_Comand(SCAN_LIMIT,0x03);
    Write_Comand(SHUTDOWN,0x01);
}

uint8_t bcd_to_decimal(uint8_t number) 
{ 
  return((number >> 4) * 10 + (number & 0x0F));
}

uint8_t decimal_to_bcd(uint8_t number) 
{
  return(((number / 10) << 4) + (number % 10));
}
void RTC_Display(){

   I2C_Write1ByteRegister(0b1101000,0xD1,0x00);
   minute = I2C_Read1ByteRegister(0b1101000,0x01);  // 0b1101000
   hour   = I2C_Read1ByteRegister(0b1101000,0x02);  
      
   minute = bcd_to_decimal(minute);  
   hour   = bcd_to_decimal(hour);
   
   
   Write_Digits(0,hour/10);   
   Write_Digits(1,hour%10);
   Write_Digits(2,minute/10);   
   Write_Digits(3,minute%10);
}

int debounce(){
    uint8_t count = 0;
    for(uint8_t i = 0; i < 3; i++){
        if (BTN_MODE == Press)
        count = count + 1;
        __delay_ms(10);
    }
    if(count > 2)  {return 1;} // when press return 1
    else           {return 0;} // when release return 0
}
void delay(){
  TMR1H = TMR1L = 0;  // reset Timer1
  TMR1ON = 1;         // enable Timer1 module
  // wait for 250ms or at least one button press
  while (((uint16_t)(TMR1H << 8) | TMR1L) < 62500 && BTN_MODE && BTN_UP);
  TMR1ON = 0;         // disable Timer1 module
}

uint8_t edit(uint8_t position, uint8_t parameter){   
    while(debounce());  // call debounce function (wait for B1 to be released) 
    while(1){ 
    while(!BTN_UP){    // if button UP is pressed    
        parameter = parameter + 1;
        if(i == 1 && parameter > 23) parameter = 0; 
        if(i == 2 && parameter > 59) parameter = 0;
        if(position % 2 != 0){
            Write_Digits(0,parameter/10);
            Write_Digits(1,parameter%10);   
        }else{
            Write_Digits(2,parameter/10);
            Write_Digits(3,parameter%10);  
        }
            
        __delay_ms(200);
    }

    if(position %2 != 0){
        Write_Digits(0,0x0F);
        Write_Digits(1,0x0F);
        delay();
        Write_Digits(0,parameter/10);
        Write_Digits(1,parameter%10);    
        delay();
    }else{
        Write_Digits(2,0x0F);
        Write_Digits(3,0x0F);
        delay();
        Write_Digits(2,parameter/10);
        Write_Digits(3,parameter%10);
        delay();
    }
    

    if(!BTN_MODE)     // if button B1 is pressed
    if(debounce()){   // call debounce function (make sure B1 is pressed)   
        i = i + 1;   // increment 'i' for the next parameter        
        if(i == 3)  i = 0;
        return parameter;
    }
    
  }
        
} 
void Animation(){
    
     for(int j = 0; j < 4; j++){
        Write_Digits(i,10);        
    }
     __delay_ms(100);
    for(int i = 0; i < 4; i++){
        Write_Digits(i,11);        
    }
     __delay_ms(100);
    for(int i = 0; i < 4; i++){
        Write_Digits(i,12);        
    }
      __delay_ms(100);
    for(int i = 0; i < 4; i++){
        Write_Digits(i,13);        
    }
    __delay_ms(100);
    for(int i = 0; i < 4; i++){
        Write_Digits(i,14);        
    }

}
//
__bit ds18b20_start(){
  DS18B20_PIN = 0;      // send reset pulse to the DS18B20 sensor
  DS18B20_PIN_Dir = 0;  // configure DS18B20_PIN pin as output
  __delay_us(480);      // wait 480 us
  DS18B20_PIN_Dir = 1;  // configure DS18B20_PIN pin as input
  __delay_us(70);      // wait 70 us to read the DS18B20 sensor response
 
  if (!DS18B20_PIN)
  {
    __delay_us(410);    // wait 410 us
    return 1;           // DS18B20 sensor is present
  }
 
  return 0;   // connection error
}

void ds18b20_write_bit(uint8_t value){
  DS18B20_PIN = 0;
  DS18B20_PIN_Dir = 0;  // configure DS18B20_PIN pin as output
  __delay_us(2);        // wait 2 us
 
  DS18B20_PIN = (__bit)value;
  __delay_us(80);       // wait 80 us
 
  DS18B20_PIN_Dir = 1;  // configure DS18B20_PIN pin as input
  __delay_us(2);        // wait 2 us
}

void ds18b20_write_byte(uint8_t value)
{
    for(uint8_t i = 0; i < 8; i++)
        ds18b20_write_bit(value >> i);
}

unsigned char ds18b20_read_bit(void){
  static __bit value;
  DS18B20_PIN = 0;
  DS18B20_PIN_Dir = 0;  // configure DS18B20_PIN pin as output
  __delay_us(2); 
  DS18B20_PIN_Dir = 1;  // configure DS18B20_PIN pin as input
  __delay_us(5);        // wait 5 us
  value = DS18B20_PIN;  // read and store DS18B20 state
  __delay_us(100);      // wait 100 us 
  return value;
}

uint8_t ds18b20_read_byte(void){
  uint8_t value = 0;
  for(uint8_t i = 0; i < 8; i++)
    value |= ds18b20_read_bit() << i;
  return value;
}
__bit ds18b20_read(uint16_t *raw_temp_value)
{
  if (!ds18b20_start())   // send start pulse
    return 0;             // return 0 if error
 
  ds18b20_write_byte(0xCC);   // send skip ROM command
  ds18b20_write_byte(0x44);   // send start conversion command
 
  while(ds18b20_read_byte() == 0);  // wait for conversion complete
 
  if (!ds18b20_start())  // send start pulse
    return 0;            // return 0 if error
 
  ds18b20_write_byte(0xCC);  // send skip ROM command
  ds18b20_write_byte(0xBE);  // send read command
 
  // read temperature LSB byte and store it on raw_temp_value LSB byte
  *raw_temp_value  = ds18b20_read_byte();
  // read temperature MSB byte and store it on raw_temp_value MSB byte
  *raw_temp_value = (uint16_t)(ds18b20_read_byte() << 8) + (*raw_temp_value);
 
  return 1;   // OK --> return 1
}
void display_digitalCLK(){
    if(!BTN_MODE)     // if button BTN_MODE is pressed
    if(debounce()){  // call debounce function (make sure B1 is pressed)  
        i = 1;
        hour   = edit(1,hour);
        minute = edit(2,minute);
        while(debounce()); 
        hour     = decimal_to_bcd(hour);
        minute   = decimal_to_bcd(minute);        
        I2C_Write1ByteRegister(ADDRESS_DS1307,0x01,minute);
        I2C_Write1ByteRegister(ADDRESS_DS1307,0x02,hour);
        //__delay_ms(200); 
    }   
        // read current time from the RTC chip
        RTC_Display();
       // __delay_ms(50);   // wait 50 ms    

}
void main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    MAX7219_Init();
    
    ANSELC = 0x00;
    ANSELA = 0x00;
    
    //TIMER1 configuration
    T1CON  = 0x30;   // set Timer1 clock source to internal with 1:8 prescaler                  
    TMR1H  = TMR1L = 0;  // reset Timer1
       
    // SET INPUT FOR BUTTON
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;
    TRISCbits.TRISC5 = 1;    
    
    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    //INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
   // INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
//    Animation();
//    __delay_ms(300);
    
//    THIET_LAP_THOI_GIAN_HIEN_TAI();
//    NAP_TGIAN_HIEM_TAI();
    while (1)
    {            
//        //display_digitalCLK();
           // RTC_Display();
        // RTC_Display();
//        __delay_ms(200);
        //display_digitalCLK();
        if(flag_sys == 0){
            //printf("step 1");
            display_digitalCLK();
            if(sys_cnt < 1000)  {sys_cnt++;}
            else{
                MAX7219_CLEAR();
                sys_cnt = 0;
                flag_sys = 1;
            }           
        }
        else if(flag_sys == 1){      
           // printf("step 2");
            if(ds18b20_read(&raw_temp)){
                int_part = (raw_temp >> 4);
                Write_Digits(0,(int_part/10));
                Write_Digits(1,(int_part%10));    
                Write_Digits(2,10);
                Write_Digits(3,11);    
            }
            if(sys_cnt < 10)  {sys_cnt++;}
            else{
                MAX7219_CLEAR();
                sys_cnt = 0;
                flag_sys = 0;            
            }
        }
         
    }                   
   
}
