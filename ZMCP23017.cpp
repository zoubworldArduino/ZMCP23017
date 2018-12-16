/*************************************************** 
 This is a library for the ZMCP23017 i2c port expander

 These displays use I2C to communicate, 2 pins are required to
 interface

This code is inspireed from 
- https://github.com/kasperskaarhoj/MCP23017-Arduino-Library
- https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library
.

 ****************************************************/


#include "ZMCP23017.h"
#include <WireUtility.h>

#define ZMCP23017_ADDRESS 0x20
#define ZMCP23017_ADDRESS_MASK 0x78


// registers
//Address IOCON.BANK = 0
#define ZMCP23017_IODIRA 0x00
#define ZMCP23017_IPOLA 0x02
#define ZMCP23017_GPINTENA 0x04
#define ZMCP23017_DEFVALA 0x06
#define ZMCP23017_INTCONA 0x08
#define ZMCP23017_IOCONA 0x0A
#define ZMCP23017_GPPUA 0x0C
#define ZMCP23017_INTFA 0x0E
#define ZMCP23017_INTCAPA 0x10
#define ZMCP23017_GPIOA 0x12
#define ZMCP23017_OLATA 0x14


#define ZMCP23017_IODIRB 0x01
#define ZMCP23017_IPOLB 0x03
#define ZMCP23017_GPINTENB 0x05
#define ZMCP23017_DEFVALB 0x07
#define ZMCP23017_INTCONB 0x09
#define ZMCP23017_IOCONB 0x0B
#define ZMCP23017_GPPUB 0x0D
#define ZMCP23017_INTFB 0x0F
#define ZMCP23017_INTCAPB 0x11
#define ZMCP23017_GPIOB 0x13
#define ZMCP23017_OLATB 0x15

#define ZMCP23017_INT_ERR 255


//Register defines from data sheet - we set IOCON.BANK to 0
//as it is easier to manage the registers sequentially.
#define ZMCP23017_IODIR 0x00
#define ZMCP23017_IPOL 0x2
#define ZMCP23017_GPPU 0x0C
#define ZMCP23017_GPIO 0x12

/** dummy function for compatibility with arduino API, 
always return 0
*/
uint32_t ZMCP23017::analogRead( uint32_t pin )
{ return 0;
}
bool ZMCP23017::check()
  {
  return (_i2caddr&~0x07==0x20);
}
/** define the address based of PIN A2 , A1,A0 value
*/
void ZMCP23017::setHardAddress(uint8_t A210)
  {
    _i2caddr=0x20 || A210&0x7;
  }
   
static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
  if (from == to) {
    return value;
  }
  if (from > to) {
    return value >> (from-to);
  }
  return value << (to-from);
}
  void ZMCP23017::setWire(TwoWire *My_i2c)
  {
	  _i2c=My_i2c;
  }


  bool ZMCP23017::test()
  {
    bool b=_i2c->testLine();
    b&=check();
    b&=WireTest( *_i2c, _i2caddr);
/*IOCON
  b1:0x05 = 0x15
  b0: 0x0A=0xB
  */  
 
    uint8_t d=wireRead8( *_i2c, _i2caddr, ZMCP23017_IOCONB);
      b&=wireRead8( *_i2c, _i2caddr,  ZMCP23017_IOCONB)==wireRead8( *_i2c, _i2caddr,  ZMCP23017_IOCONA);
    b&=d&0x01==0;    
    wireWrite8( *_i2c, _i2caddr, ZMCP23017_IOCONB,d|0x01);
    b&=d&0x01==0;    
    wireWrite8( *_i2c, _i2caddr, ZMCP23017_IOCONB,d|(1<<5));
    b&=wireRead8( *_i2c, _i2caddr, ZMCP23017_IOCONB)==(d|(1<<5));
    b&=wireRead8( *_i2c, _i2caddr, ZMCP23017_IOCONB)==wireRead8( *_i2c, _i2caddr,  ZMCP23017_IOCONA);
    wireWrite8( *_i2c, _i2caddr, ZMCP23017_IOCONB,d&~(1<<5));
    b&=wireRead8( *_i2c, _i2caddr, ZMCP23017_IOCONB)==(d&~(1<<5));
    b&=wireRead8( *_i2c, _i2caddr,  ZMCP23017_IOCONB)==wireRead8( *_i2c, _i2caddr, ZMCP23017_IOCONA);
    
    
  return b;
  }
////////////////////////////////////////////////////////////////////////////////
/**
 * Initializes the ZMCP23017 given its HW selected address, see datasheet for Address selection.
or use setHardAddress to over write it.

 */
void ZMCP23017::begin(TwoWire *My_i2c,uint8_t addr)
  {
	  _i2c=My_i2c ;
	if ((addr & ZMCP23017_ADDRESS_MASK)!=ZMCP23017_ADDRESS) {
		while(1);
	}
	_i2caddr = addr;

        
        //Default state is 0 for our pins
	_GPIO = 0x0000;
	_IODIR = 0xffff;//input
        _GPPU = 0x0000;// no pull up

	//_i2c->begin();

	//Set the IOCON.BANK bit to 0 to enable sequential addressing
	//IOCON 'default' address is 0x05, but will
	//change to our definition of IOCON once this write completes.
	writeRegister(0x05, (byte)0x0);//in case of bank 1, switch to bank 0
	                               //else GPINTENB=0
	//Our pins default to being inputs by default.
        writeRegister(ZMCP23017_IODIR, (word)_IODIR);//

	
}
/**
 * Initializes the ZMCP23017 given its HW selected address, see datasheet for Address selection.
 */
void ZMCP23017::begin(uint8_t addr) {
	begin( &Wire,addr);
}

/**
 * Initializes the default ZMCP23017, with 000 for the configurable part of the address
 */
void ZMCP23017::begin(void) {
	begin(0);
}

/**
 * Sets the pin mode to either INPUT or OUTPUT
 *
void ZMCP23017::pinMode(uint32_t p, uint8_t d) {
	if(acceptlocal( p))
	{
		updateRegisterBit(p,(d==INPUT),ZMCP23017_IODIRA,ZMCP23017_IODIRB);
	}
	else if (_next)
		return _next->pinMode( p,d);
	
	
}*/
/** define the mode of the pin like arduino API
mode can be INPUT_PULLUP, INPUT,OUTPUT
ulPin is a pin from  this.getPin(x) where x=0..15
*/
void ZMCP23017::pinMode(uint32_t ulPin, uint8_t mode) {
  if(acceptlocal( ulPin))
	{
            uint8_t pin=pin2channel(ulPin);
	//Arduino defines OUTPUT as 1, but
	//MCP23017 uses OUTPUT as 0. (input is 0x1)
	//mode = !mode;
	if ((mode==INPUT)||(mode==INPUT_PULLUP)) 
          _IODIR |= 1 << pin;
	else  _IODIR &= ~(1 << pin);
	writeRegister(ZMCP23017_IODIR, (word)_IODIR);
        if (mode==INPUT_PULLUP)
          pullUp(pin, 1);
        else
           pullUp(pin, 0);
        }
	else if (_next)
		return _next->pinMode( ulPin,mode);
}
/** define the output vallue of the pin like arduino API
val can be LOW, HIGH
ulPin is a pin from  this.getPin(x) where x=0..15
if input mode is set, a High active pull up.

*/
void ZMCP23017::digitalWrite(uint32_t ulPin, uint8_t val) {
  if(acceptlocal( ulPin))
	{
          uint8_t pin=pin2channel(ulPin);
	//If this pin is an INPUT pin, a write here will
	//enable the internal pullup
	//otherwise, it will set the OUTPUT voltage
	//as appropriate.
	bool isOutput = !(_IODIR & 1<<pin);

	if (isOutput) {
		//This is an output pin so just write the value
		if (val==HIGH) 
                  _GPIO |= 1 << pin;
		else _GPIO &= ~(1 << pin);
		writeRegister(ZMCP23017_GPIO, (word)_GPIO);
	}
	else {
		//This is an input pin, so we need to enable the pullup
		if (val) _GPPU |= 1 << pin;
		else _GPPU &= ~(1 << pin);
		writeRegister(ZMCP23017_GPPU, (word)_GPPU);
	}
        }
	else if (_next)
		return _next->digitalWrite( ulPin,val);
}
/** define the analog Write Resolution of the components like arduino API
res can be 1..16
*/
void  ZMCP23017::analogWriteResolution(int res)
{
  _writeResolution = res;
  if (_next)
		return _next->analogWriteResolution( res);
}

/** define the analog output value of the pin like arduino API
ulValue can be 0..4096...

ulPin is a pin from  this.getPin(x) where x=0..15
here not PWM are avalable so a value below _writeResolution/2 generate a LOW state else an HIGH state will be apply
*/
 void ZMCP23017::analogWrite( uint32_t ulPin, uint32_t ulValue ) 
 {
	if(acceptlocal( ulPin))
	{
                uint8_t channel=pin2channel(ulPin);
		ulValue = mapResolution(ulValue, _writeResolution, 8);
		if (ulValue < _writeResolution/2) {
			digitalWrite(channel, LOW);
		} else {
			digitalWrite(channel, HIGH);
		}
	}
	else if (_next)
		return _next->analogWrite( ulPin,ulValue);
 }
/** define the pull upof the pin 
d can be 0 for nio pull up and 1 to enable pull up
p is a pin from 0..15
*/
void ZMCP23017::pullUp(uint32_t p, uint8_t d) {
  if (d!=0) _GPPU |= 1<<p;
  else
     _GPPU &= ~(1<<p);
  internalPullupMask(_GPPU) ;
//	updateRegisterBit(p,d,ZMCP23017_GPPUA,ZMCP23017_GPPUB);
}

/** convert a arduino pin to a component channel
ulPin is a pin from  this.getPin(x) where x=0..15
it return x, or NO_CHANNEL
*/
 uint8_t ZMCP23017::pin2channel(uint32_t ulPin)
 {
	 if((ulPin>>16)==_i2caddr)
	 return ulPin &0xff;
 return NO_CHANNEL;
 }
/** return the digital value of the pin like arduino API

ulPin is a pin from  this.getPin(channel) where channel=0..15
returns LOW or HIGH,
in case of no pin it returns LOW
*/
uint8_t ZMCP23017::digitalRead(uint32_t ulPin) {
  if(acceptlocal( ulPin))
	{
          uint8_t channel=pin2channel(ulPin);
	_GPIO = readRegister(ZMCP23017_GPIO);
	if ( _GPIO & (1 << channel)) return HIGH;
	else return LOW;
        }
	else if (_next)
		return _next->digitalRead( ulPin);
        return LOW;
}
/** this test if the arduino pin is a pin for this object
a pin of this object is a ulPin value where the value is equal to this.getPin(channel) for a channel=0..15
*/
bool ZMCP23017::acceptlocal(uint32_t p)
{
  return (p>>16==_i2caddr);
}




//##########################################################################
/** this function return a word that is the image to of the bus GPIOB[7..0]:GPIOA[7..0]
*/
word ZMCP23017::digitalWordRead() {
	_GPIO = readRegister(ZMCP23017_GPIO);
	return _GPIO;
}
/** this function write a word that is applied to the bus GPIOB[7..0]:GPIOA[7..0]
*/
void ZMCP23017::digitalWordWrite(word w) {
	_GPIO = w;
	writeRegister(ZMCP23017_GPIO, (word)_GPIO);
}

void ZMCP23017::inputPolarityMask(word mask) {
	writeRegister(ZMCP23017_IPOL, mask);
}

/** this function define the direction of the bus GPIOB[7..0]:GPIOA[7..0], one bit for each pin
1 means input 
0 means output
*/
void ZMCP23017::inputOutputMask(word mask) {
	_IODIR = mask;
	writeRegister(ZMCP23017_IODIR, (word)_IODIR);
}
/** this function define the pull up activation of the bus GPIOB[7..0]:GPIOA[7..0], one bit for each pin
1 means enabled 
0 means disabled
*/
void ZMCP23017::internalPullupMask(word mask) {
	_GPPU = mask;
	writeRegister(ZMCP23017_GPPU, (word)_GPPU);
}

//PRIVATE
void ZMCP23017::writeRegister(int regAddress, byte data) {
	_i2c->beginTransmission(_i2caddr);
	_i2c->write(regAddress);
	_i2c->write(data);
	_i2c->endTransmission();
}

void ZMCP23017::writeRegister(int regAddress, word data) {
	_i2c->beginTransmission(_i2caddr);
	_i2c->write(regAddress);
	_i2c->write(lowByte(data));
	_i2c->write(highByte(data));
	_i2c->endTransmission();
}

word ZMCP23017::readRegister(int regAddress) {
	word returnword = 0x00;
	_i2c->beginTransmission(_i2caddr);
	_i2c->write(regAddress);
	_i2c->endTransmission();
	_i2c->requestFrom((int)_i2caddr, 2);
    
    int c=0;
	//Wait for our 2 bytes to become available
	while (_i2c->available()) {
        //high byte
        if (c==0)   { returnword = _i2c->read() << 8; }
        //low byte
        if (c==1)   { returnword |= _i2c->read(); }
        c++;
    }
    
	return returnword;
}