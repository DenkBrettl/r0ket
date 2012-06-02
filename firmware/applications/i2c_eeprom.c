
#include <sysinit.h>

#include "basic/basic.h"
#include "lcd/render.h"
#include "lcd/print.h"

#include "filesystem/ff.h"
#include "filesystem/diskio.h"

#include "core/i2c/i2c.h"

#include "usb/usbmsc.h"

/* function prototypes */
void gotoISP(void);
void gotoMSC(void);
void eeprom_read(void);
void eeprom_write(void);
int myatoi(const char *);
unsigned long int hextoul(char *);
// hackety hack :-)
char *strcpy(char *restrict dst, const char *restrict src);
signed int PutUnsignedInt(char *, char, signed int, unsigned int);

#define LOW_BYTE(x) ((char*)(&(x)))[0]
#define HIGH_BYTE(x) ((char*)(&(x)))[1]

int32_t eeprom_write_byte(uint8_t, uint16_t );
int8_t eeprom_read_byte(uint8_t *, uint16_t );
int8_t eeprom_read_byte_current(uint8_t, uint8_t *);
    
/* some variables and defines */
#define STAT_IDLE 0
#define STAT_READ 1
#define STAT_WRITE 2

#define READ_LED RB_LED2
#define WRITE_LED RB_LED0

#define EEPROM_8BIT_ADDR 0
#define EEPROM_16BIT_ADDR 1

uint status = STAT_IDLE;

struct EEPROM_CFG_DEF {
  uint8_t  base_addr; /* base address of the EEPROM, e.g. 0xA0 for 24LC01B and similar */
  uint8_t  addr_bits; /* 8 or 16 bit read/write address */
  uint16_t size; /* bytes */
  uint16_t blocksize; /* bytes */
};
struct EEPROM_CFG_DEF eeprom_cfg;

/* this is stored a FAT filesystem which can only
 * handle 8.3 chars for the filenames so
 * 15 bytes is a safe assumption */
char image_file_name[15] = "eeprom01.img";

void set_eeprom_8bit_addressing(void)
{
  eeprom_cfg.addr_bits = EEPROM_8BIT_ADDR;

  lcdPrintln("EEPROM type");
  lcdPrintln("set to 8 bit");
  lcdPrintln("addressing.");
  lcdRefresh(); 

  do{
  } while ((getInput()) != BTN_ENTER);
}

void set_eeprom_16bit_addressing(void)
{
  eeprom_cfg.addr_bits = EEPROM_16BIT_ADDR;

  lcdPrintln("EEPROM type");
  lcdPrintln("set to 16 bit");
  lcdPrintln("addressing.");
  lcdRefresh(); 

  do{
  } while ((getInput()) != BTN_ENTER);
}

void set_eeprom_blocksize(void)
{
  char size_input[6];
  PutUnsignedInt(size_input, '0', 5, eeprom_cfg.blocksize);
  input("Byte per block:", (char *)&size_input, 48, 57, 6);
  getInputWaitRelease();
	eeprom_cfg.blocksize = myatoi(size_input);
}

void set_eeprom_size(void)
{
  char size_input[6];
  PutUnsignedInt(size_input, '0', 5, eeprom_cfg.size);
  input("EEPROM bytes:", (char *)&size_input, 48, 57, 6);
  getInputWaitRelease();
	eeprom_cfg.size = myatoi(size_input);
}

void set_eeprom_baseaddr(void)
{
  char addr_input[6];
  snprintf(addr_input, 5, "0x%x", eeprom_cfg.base_addr);
  input("Base address:", (char *)&addr_input, 0x20, 0x7f, 6);
  getInputWaitRelease();
	eeprom_cfg.base_addr = hextoul(addr_input);
}

void set_imgfile_name(void)
{
  input("Image file:", (char *)&image_file_name, 0x20, 0x7f, 13);
  getInputWaitRelease();
}

static const struct MENU submenu_eeprom_type={ "Set PROM type", {
  /* got 11 chars on the display */
  /* 012345678901 */
	{ " 8 bit addrs", &set_eeprom_8bit_addressing},
	{ "16 bit addrs", &set_eeprom_16bit_addressing},
	{NULL,NULL}
}};

void show_submenu_eeprom_type(void) {
	handleMenu(&submenu_eeprom_type);
};

static const struct MENU submenu_eeprom_conf={ "Cfgure EEPROM", {
  /* got 11 chars on the display */
  /* 012345678901 */
  { "Size (bytes)",  &set_eeprom_size},
  { "Blocksize",  &set_eeprom_blocksize},
  { "Base address",  &set_eeprom_baseaddr},
  { "Address type",  &show_submenu_eeprom_type},
	{NULL,NULL}
}};

void show_submenu_eeprom_conf(void) {
	handleMenu(&submenu_eeprom_conf);
};

static const struct MENU mainmenu = {"Mainmenu", {
  /* got 11 chars on the display */
  /* 012345678901 */
  { "EEPROM conf",  &show_submenu_eeprom_conf},
  { "Image file",  &set_imgfile_name},
  { "Read  EEPROM",  &eeprom_read},
  { "Write EEPROM",  &eeprom_write},
  { "---",   NULL},
  { "Invoke ISP", &gotoISP},
  { "Invoke MSC", &gotoMSC},
  { NULL,NULL}
}};


/* 
 * Description:
 * Write a byte to a specific address.
 *
 * Parameters
 * IN:
 *   the data byte to write
 *   The address as uint16_t where to write the byte
 * OUT:
 *   none
 * RETURN Values:
 *   0 -- success
 *  -1 -- error
 * A more detailed error can be fetched from I2CSlaveState.
 */
int32_t eeprom_write_byte(uint8_t byte, uint16_t address) {
  uint32_t count = 0;
  uint32_t ret;
  uint16_t effective_base_addr = eeprom_cfg.base_addr;

  /* The EEPROM may be divided into blocks. If it is, the block is 
   * addressed inside the base address. */
  while(address >= eeprom_cfg.blocksize) {
    address -= eeprom_cfg.blocksize;
    effective_base_addr += 2;
  }
  
  I2CMasterBuffer[0] = LOW_BYTE(effective_base_addr);
  I2CMasterBuffer[1] = LOW_BYTE(address);
  if(eeprom_cfg.addr_bits == EEPROM_8BIT_ADDR) {
    I2CMasterBuffer[2] = byte;
    I2CWriteLength = 3;
  }
  else if(eeprom_cfg.addr_bits == EEPROM_16BIT_ADDR) {
    I2CMasterBuffer[2] = HIGH_BYTE(address);
    I2CMasterBuffer[3] = byte;
    I2CWriteLength = 4;
  }
  else return -1;
  I2CReadLength = 0;
  ret = i2cEngine();
  if(ret != I2CSTATE_ACK) return -1;
  
  /* at this point, the EEPROM will begin to write the byte. This internal write cycle
   * takes some time. As long as it does, the EEPROM will not send an ACK on the bus on
   * any inquiry. This can be used to wait for the end of the write cycle.
   */
  I2CMasterBuffer[0] = LOW_BYTE(effective_base_addr);
  I2CWriteLength = 1;
  I2CReadLength = 0;
  while(i2cEngine() != I2CSTATE_ACK) {
    count++;
    if(count > 10001) return -1; /* FIXME: this is just some guess for a maximum wait count */
  }
  delayms(10);
  return count;
}

/* 
 * Description:
 * Read a byte from a specific address.
 * This is achieved by first writing "nothing" into the EEPROM
 * which sets the address register to the position we want.
 * After that, read the byte at the current position.
 *
 * Parameters
 * IN:
 *   The address from where to read as uint16_t
 * OUT:
 *   pointer to uint8_t sized variable which will store the result
 * RETURN Values:
 *   0 -- success
 *  -1 -- error
 * A more detailed error can be fetched from I2CSlaveState.
 */
int8_t eeprom_read_byte(uint8_t *byte, uint16_t address) {
  uint32_t ret;
  uint16_t effective_base_addr = eeprom_cfg.base_addr;

  /* The EEPROM may be divided into blocks. If it is, the block is 
   * addressed inside the base address. */
  while(address >= eeprom_cfg.blocksize) {
    address -= eeprom_cfg.blocksize;
    effective_base_addr += 2;
  }
  
  I2CMasterBuffer[0] = LOW_BYTE(effective_base_addr);
  I2CMasterBuffer[1] = LOW_BYTE(address);
  if(eeprom_cfg.addr_bits == EEPROM_8BIT_ADDR) {
    I2CWriteLength = 2;
  }
  else if(eeprom_cfg.addr_bits == EEPROM_16BIT_ADDR) {
    I2CMasterBuffer[2] = HIGH_BYTE(address);
    I2CWriteLength = 3;
  }
  else return -1;
  I2CReadLength = 0;

  ret = i2cEngine();
  if(ret != I2CSTATE_ACK) return -1;
  return eeprom_read_byte_current(effective_base_addr, byte);
}

/* 
 * Description:
 * Read a byte from the address currently stored in 
 * the address register of the EEPROM. The register will 
 * increase with every read by one and wrap around at the 
 * end of the EEPROM storage.
 * 
 * Parameters
 * IN:
 *   I2C base address of the EEPROM (= write address)
 *   The read address will be constructed
 * OUT:
 *   pointer to uint8_t sized variable which will store the result
 * RETURN Values:
 *   0 -- success
 *  -1 -- error
 * A more detailed error can be fetched from I2CSlaveState.
 */
int8_t eeprom_read_byte_current(uint8_t addr, uint8_t *byte) {
  /* in I2C the read address is always the write (= base) address + 1 
   * (basically: the last bit is high instead of low) */
  I2CMasterBuffer[0] = eeprom_cfg.base_addr;
  I2CWriteLength = 1;
  I2CReadLength = 1;
  if(i2cEngine() != I2CSTATE_ACK) return -1;
  else {
    *byte = I2CSlaveBuffer[0];
    return 0;
  }
}


void main_i2c_eeprom(void) {
  /* set some sane defaults */
  eeprom_cfg.base_addr = 0xA0;   /* 24LCXX */
  eeprom_cfg.addr_bits = EEPROM_8BIT_ADDR;
/*  eeprom_cfg.size = 128;
  eeprom_cfg.blocksize = 128;*/
  eeprom_cfg.size = 1024;
  eeprom_cfg.blocksize = 256;

  while (1) {
    lcdDisplay();
    delayms(10);
    lcdFill(0); // clear display buffer
    handleMenu(&mainmenu);
    gotoISP();
  }
}

void tick_i2c_eeprom(void) {
  static int foo = 0;
  static int toggle = 0;

  if((status != STAT_IDLE) && (foo++ > 10)) {
    toggle = 1-toggle;
    foo = 0;
    switch(status) {
      case STAT_WRITE:
        gpioSetValue (WRITE_LED, toggle);
        break;
      case STAT_READ:
        gpioSetValue (READ_LED, toggle);
        break;
    }
  }
  
  return;
}

/* Functions */

void gotoISP(void) {
  DoString(0,0,"Enter ISP!");
  lcdDisplay();
  ISPandReset();
}

void gotoMSC(void) {
  lcdClear();
  lcdPrintln("Enter MSC!");
  lcdPrintln("Press btn to");
  lcdPrintln("exit.");
  lcdRefresh();
  usbMSCInit();
  do{
    delayms(50);
  } while ((getInput()) != BTN_ENTER);
  usbMSCOff();
}

void eeprom_write(void) {
  uint32_t write_cnt;
  uint8_t write_byte_buf;
  FIL file;
  UINT f_readbytes;
  int32_t res;

  status = STAT_WRITE;
  gpioSetValue(READ_LED, 0);
    
  lcdClear();

  i2cInit(I2CMASTER); // Init I2C

  res = f_open(&file, image_file_name, FA_OPEN_EXISTING|FA_READ);
  if(res){
    lcdPrintln("ERROR:");
    lcdPrintln("Can't open");
    lcdPrintln("image file");
    lcdPrintln(image_file_name);
    lcdRefresh();
    status = STAT_IDLE;
    gpioSetValue(WRITE_LED, 0);
    do{
    } while ((getInput()) != BTN_ENTER);
    return;
  };

  lcdPrintln("Writing...");
  lcdRefresh();

  for(write_cnt = 0; write_cnt < eeprom_cfg.size; write_cnt++) {
    res = f_read(&file, (char *)&write_byte_buf, 1, &f_readbytes);
    if(res || (f_readbytes != 1) ){
      lcdPrintln("ERROR:");
      lcdPrintln("File read");
      lcdPrintln("failed");
      lcdPrintln("at byte");
      lcdPrintln(IntToStr(write_cnt, 5, 0));
      lcdRefresh();
      f_close(&file);
      status = STAT_IDLE;
      gpioSetValue(WRITE_LED, 0);
      do{
      } while ((getInput()) != BTN_ENTER);
      return;
    }
    /* the var "written" is also the address here */
    res = eeprom_write_byte(write_byte_buf, write_cnt);
    if(res == -1) {
      lcdPrintln("Write error");
      lcdPrintln("at byte");
      lcdPrintln(IntToStr(write_cnt, 5, 0));
      lcdRefresh();
      f_close(&file);
      status = STAT_IDLE;
      gpioSetValue(WRITE_LED, 0);
      do{
      } while ((getInput()) != BTN_ENTER);
      return;
    }
  }

  f_close(&file);

  lcdPrint("Count: ");
  lcdPrintln(IntToStr(write_cnt, 5, 0));
  lcdPrintln("Write complete!");
  lcdRefresh(); 

  status = STAT_IDLE;
  gpioSetValue(WRITE_LED, 0);

  do{
  } while ((getInput()) != BTN_ENTER);

}

void eeprom_read(void) {
  uint read_cnt;
  uint8_t read_byte_buf;
  FIL file;
  UINT f_writebytes;
  int32_t res;
  
  status = STAT_READ;
  gpioSetValue(WRITE_LED, 0);

  lcdClear();

  i2cInit(I2CMASTER); // Init I2C

  res = f_open(&file, image_file_name, FA_CREATE_ALWAYS|FA_WRITE);
  if(res){
    lcdPrintln("ERROR:");
    lcdPrintln("Can't create");
    lcdPrintln("image file");
    lcdPrintln(image_file_name);
    lcdRefresh();
    status = STAT_IDLE;
    gpioSetValue(READ_LED, 0);
    do{
    } while ((getInput()) != BTN_ENTER);
    return;
  };

  lcdPrintln("Reading...");
  lcdRefresh();

  for(read_cnt = 0; read_cnt < eeprom_cfg.size; read_cnt++) {
    /* the var "read_cnt" is also the address here */
    res = eeprom_read_byte(&read_byte_buf, read_cnt);
    if(res == -1) {
      lcdPrintln("Read error");
      lcdPrintln("at byte");
      lcdPrintln(IntToStr(read_cnt, 5, 0));
      lcdRefresh();
      f_close(&file);
      status = STAT_IDLE;
      gpioSetValue(WRITE_LED, 0);
      do{
      } while ((getInput()) != BTN_ENTER);
      return;
    }

    res = f_write(&file, &read_byte_buf, 1, &f_writebytes);
    if(res || (f_writebytes != 1) ){
      lcdPrintln("ERROR:");
      lcdPrintln("File write");
      lcdPrintln("failed");
      lcdPrintln("at byte");
      lcdPrintln(IntToStr(read_cnt, 5, 0));
      lcdRefresh();
      f_close(&file);
      status = STAT_IDLE;
      gpioSetValue(WRITE_LED, 0);
      do{
      } while ((getInput()) != BTN_ENTER);
      return;
    }
  }

  f_close(&file);

  lcdPrint("Bytes: ");
  lcdPrintln(IntToStr(read_cnt, 5, 0));
  lcdPrintln("Read complete!");
  lcdRefresh(); 

  status = STAT_IDLE;
  gpioSetValue(READ_LED, 0);

  do{
  } while ((getInput()) != BTN_ENTER);
  
  return;
}



/*
 * The table below is used to convert from ASCII digits to a
 * numerical equivalent.  It maps from '0' through 'z' to integers
 * (100 for non-digit characters).
 */

static char cvtIn[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,		/* '0' - '9' */
    100, 100, 100, 100, 100, 100, 100,		/* punctuation */
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19,	/* 'A' - 'Z' */
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
    30, 31, 32, 33, 34, 35,
    100, 100, 100, 100, 100, 100,		/* punctuation */
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19,	/* 'a' - 'z' */
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
    30, 31, 32, 33, 34, 35};


uint32_t hextoul(char *string)
{
  char *p;
  uint32_t result = 0;
  uint8_t digit;

  p = string;
	if ((p[0] == '0') && (p[1] == 'x')) {
    p += 2;
	}
	for ( ; ; p++) {
    digit = *p - '0';
    if (digit > ('z' - '0')) {
      break;
    }
    digit = cvtIn[digit];
    if (digit > 15) {
      break;
    }
    result = (result << 4) + digit;
	}
  return result;
}


int myatoi(const char *string)
{
	uint32_t i;
	i=0;
	while(*string)
	{
		i=(i<<3) + (i<<1) + (*string - '0');
		string++;
	}
	return i;
}
