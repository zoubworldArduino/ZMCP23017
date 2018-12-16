/** @file ZMCP23017.h

This library manage the component MCP23017, this is a 16 pin I/O extention driver by I2C protocol

 @par dependency :
   this library use the folowing ones :   PinExtender
   you can find it on https://github.com/zoubworldArduino/

 @par Author
 strongly inspired from MCP23017 library of Adafruit

 
  This is a library for the MCP23017 i2c port expander

  These displays use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 
 @par description
 
  
  */

#ifndef _ZMCP23017_H_
#define _ZMCP23017_H_

#include "PinExtender.h"
// Don't forget the Wire library
#ifndef ARDUINO_AVR_GEMMA
                                    //TinyWireM is now part of
                                    //   Adafruit version of Wire Library, so this
                                    // will work with Adafruit ATtiny85's
                                    //But Arduino Gemma doesn't use that library
                                    // We do NOT want to include Wire if it's an arduino Gemma
  #include <Wire.h>
#else
  #include <TinyWireM.h>
  #define Wire TinyWireM
#endif

/** @name channel of the component
*/
//@{
#define  MCP23017_GPA7 7
#define  MCP23017_GPA6 6
#define  MCP23017_GPA5 5
#define  MCP23017_GPA4 4
#define  MCP23017_GPA3 3
#define  MCP23017_GPA2 2
#define  MCP23017_GPA1 1
#define  MCP23017_GPA0 0

#define  MCP23017_GPB7 15
#define  MCP23017_GPB6 14
#define  MCP23017_GPB5 13
#define  MCP23017_GPB4 12 
#define  MCP23017_GPB3 11 
#define  MCP23017_GPB2 10
#define  MCP23017_GPB1 9
#define  MCP23017_GPB0 8
//@}
#define NO_CHANNEL 0xfe
 
class ZMCP23017 : public PinExtender {
public:
 /** initialise the board,
  the Wire interface must be initialize before. see wire.begin()
  */
  void begin(TwoWire *MyWire//!< the Wire interface like &Wire for board that handle several one.
  ,uint8_t addr//!< the I2C address of MCP23017
  );
   /** initialise the board,
  the Wire interface must be initialize before. see wire.begin()
  */
  void begin(uint8_t addr//!< the I2C address of MCP23017
  );
   /** initialise the board,
  the Wire interface must be initialize before. see wire.begin()
  */
  void begin(void);
  
    /** @name hardware API
*/
//@{
    /** Test the Hardware to be sure that the connection is good.
  else it return false.
  @return true : if the communication work well and it look like that it is the good chip behing I2C interface
  */
  bool test();
   /**  check the board
  @deprecated
  */
  bool check();
  /** setup the custom address
  */
  void setHardAddress(uint8_t A210 //!< value of pin A2..A0
  );
  /** set the Wire object in case of several avalable.
*/
  void setWire(TwoWire *MyWire);
 
  void pullUp(uint32_t p, uint8_t d);
  
    /** These provide a more advanced mapping of the chip functionality
    See the data sheet for more information on what they do

     @return Returns a word with the current pin states (ie contents of the GPIO register)
	 */
    word digitalWordRead();

    /** Allows you to write a word to the GPIO register
	*/
    void digitalWordWrite(word w);

    /** Sets up the polarity mask that the MCP23017 supports
    if set to 1, it will flip the actual pin value.
	*/
    void inputPolarityMask(word mask);

    /** Sets which pins are inputs or outputs (1 = input, 0 = output) NB Opposite to arduino's
    definition for these
	*/
    void inputOutputMask(word mask);

    /** Allows enabling of the internal 100k pullup resisters (1 = enabled, 0 = disabled)
	*/
    void internalPullupMask(word mask);

//@}
  /** @name arduino like API
*/
//@{
	
  uint32_t analogRead( uint32_t pin //!< the pin requested, it is the instance number.
  );
  
  void pinMode(uint32_t p//!< the pin requested, it is the instance number.
  , uint8_t d
  );
  
  void digitalWrite(uint32_t p//!< the pin requested, it is the instance number.
  , uint8_t d
  );
  
  uint8_t digitalRead(uint32_t p//!< the pin requested, it is the instance number.
  );
   /*
 * \brief Writes an analog value (PWM wave normaly but replace by LOW and HIGH state) to a pin.
 *
 * \param ulPin
 * \param ulValue
 */
 void analogWrite( uint32_t ulPin//!< the pin requested, it is the instance number.
 , uint32_t ulValue // the analog value to write, if below 2^(res-1) it is LOW else it is HIGH state
 ) ;
/*
 * \brief Set the resolution of analogWrite parameters. Default is 8 bits (range from 0 to 255).
 *
 * \param res
 */
  void analogWriteResolution(int res
  );
//@}
/*
  void writeGPIOAB(uint16_t);
  uint16_t readGPIOAB();
  uint8_t readGPIO(uint8_t b);

  void setupInterrupts(uint8_t mirroring, uint8_t open, uint8_t polarity);
  void setupInterruptPin(uint8_t p, uint8_t mode);
  uint8_t getLastInterruptPin();
  uint8_t getLastInterruptPinValue();
*/
  
  // protected:
  // friend Zmotor2;
  bool acceptlocal(uint32_t p);
 protected:
      

 private:
//   uint8_t wirerecv(void);
   //Cached copies of the register vales
word _GPIO, _IODIR, _GPPU;
//   void wiresend(uint8_t x);
   uint8_t pin2channel(uint32_t pin);
int _writeResolution ;
  /** write component 8 bits register througth I2C
  */
void writeRegister(int regaddress, byte val);
  /** write component 16 bits register througth I2C
  */
	void writeRegister(int regaddress, word val);
	  /** read component 16 bits register througth I2C
  */
word readRegister(int regaddress);
 // TwoWire *_Wire;
 // uint8_t _i2caddr;

//  uint8_t bitForPin(uint8_t pin);
//  uint8_t regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr);

//  uint8_t readRegister(uint8_t addr);
//  void writeRegister(uint8_t addr, uint8_t value);

  /**
   * Utility private method to update a register associated with a pin (whether port A/B)
   * reads its value, updates the particular bit, and writes its value.
   */
// void updateRegisterBit(uint8_t p, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr);

};


#endif
