/*
  si2c.c - Software I2C library for ESP31B

  Copyright (c) 2015 Hristo Gochkov. All rights reserved.
  This file is part of the ESP31B core for Arduino environment.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include <stdint.h>
#include <stdbool.h>
#include "twi.h"
#include "soc/gpio_reg.h"
#include "wiring.h"
#include <stdio.h>

static inline void SDA_LOW(TwiPort* twi) {
  //Enable SDA (becomes output and since GPO is 0 for the pin,
  // it will pull the line low)
  if (twi->sda < 32) {
    REG_WRITE(GPIO_ENABLE_W1TS_REG, BIT(twi->sda));
  }
  else {
    REG_WRITE(GPIO_ENABLE1_W1TS_REG, BIT(twi->sda - 32));
  }
}

static inline void SDA_HIGH(TwiPort* twi) {
  //Disable SDA (becomes input and since it has pullup it will go high)
  if (twi->sda < 32) {
    REG_WRITE(GPIO_ENABLE_W1TC_REG, BIT(twi->sda));
  }
  else {
    REG_WRITE(GPIO_ENABLE1_W1TC_REG, BIT(twi->sda - 32));
  }
}

static inline uint32_t SDA_READ(TwiPort* twi) {
  if (twi->sda < 32) {
    return (REG_READ(GPIO_IN_REG) & BIT(twi->sda)) != 0;
  }
  else {
    return (REG_READ(GPIO_IN1_REG) & BIT(twi->sda - 32)) != 0;
  }
}

static void SCL_LOW(TwiPort* twi) {
  if (twi->scl < 32) {
    REG_WRITE(GPIO_ENABLE_W1TS_REG, BIT(twi->scl));
  }
  else {
    REG_WRITE(GPIO_ENABLE1_W1TS_REG, BIT(twi->scl - 32));
  }
}

static void SCL_HIGH(TwiPort* twi) {
  if (twi->scl < 32) {
    REG_WRITE(GPIO_ENABLE_W1TC_REG, BIT(twi->scl));
  }
  else {
    REG_WRITE(GPIO_ENABLE1_W1TC_REG, BIT(twi->scl - 32));
  }
}

static uint32_t SCL_READ(TwiPort* twi) {
  if (twi->scl < 32) {
    return (REG_READ(GPIO_IN_REG) & BIT(twi->scl)) != 0;
  }
  else {
    return (REG_READ(GPIO_IN1_REG) & BIT(twi->scl - 32)) != 0;
  }
}


#ifndef FCPU80
#define FCPU80 80000000L
#endif

#if F_CPU == FCPU80
#define TWI_CLOCK_STRETCH 800
#else
#define TWI_CLOCK_STRETCH 1600
#endif

void twi_setClock(TwiPort* twi, unsigned int freq){
#if F_CPU == FCPU80
  if(freq <= 100000) twi->dcount = 19;//about 100KHz
  else if(freq <= 200000) twi->dcount = 8;//about 200KHz
  else if(freq <= 300000) twi->dcount = 3;//about 300KHz
  else if(freq <= 400000) twi->dcount = 1;//about 400KHz
  else twi->dcount = 1;//about 400KHz
#else
  if(freq <= 100000) twi->dcount = 32;//about 100KHz
  else if(freq <= 200000) twi->dcount = 14;//about 200KHz
  else if(freq <= 300000) twi->dcount = 8;//about 300KHz
  else if(freq <= 400000) twi->dcount = 5;//about 400KHz
  else if(freq <= 500000) twi->dcount = 3;//about 500KHz
  else if(freq <= 600000) twi->dcount = 2;//about 600KHz
  else twi->dcount = 1;//about 700KHz
#endif
}

void twi_init(TwiPort* twi, unsigned char sda, unsigned char scl){
  twi->dcount = 18;
  twi->sda = sda;
  twi->scl = scl;
  pinMode(twi->sda, OUTPUT);
  pinMode(twi->scl, OUTPUT);

  digitalWrite(twi->sda, 0);
  digitalWrite(twi->scl, 0);

  pinMode(twi->sda, INPUT_PULLUP);
  pinMode(twi->scl, INPUT_PULLUP);
  twi_setClock(twi, 100000);
}

void twi_stop(TwiPort* twi){
  pinMode(twi->sda, INPUT);
  pinMode(twi->scl, INPUT);
}

static void twi_delay(unsigned char v){
#if 0
  vTaskDelay(1);
#else
  unsigned int i;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  unsigned int reg;
  for(i=0;i<v;i++) reg = REG_READ(GPIO_IN_REG);
#pragma GCC diagnostic pop
#endif
}

static bool twi_write_start(TwiPort* twi) {
  SCL_HIGH(twi);
  SDA_HIGH(twi);
  if (SDA_READ(twi) == 0) return false;
  twi_delay(twi->dcount);
  SDA_LOW(twi);
  twi_delay(twi->dcount);
  return true;
}

static bool twi_write_stop(TwiPort* twi){
  unsigned int i = 0;
  SCL_LOW(twi);
  SDA_LOW(twi);
  twi_delay(twi->dcount);
  SCL_HIGH(twi);
  while (SCL_READ(twi) == 0 && (i++) < TWI_CLOCK_STRETCH);// Clock stretching (up to 100us)
  twi_delay(twi->dcount);
  SDA_HIGH(twi);
  twi_delay(twi->dcount);

  return true;
}

static bool twi_write_bit(TwiPort* twi, bool bit) {
  unsigned int i = 0;
  SCL_LOW(twi);
  if (bit) {
    SDA_HIGH(twi);
  }
  else {
    SDA_LOW(twi);
  }
  twi_delay(twi->dcount+1);
  SCL_HIGH(twi);
  while (SCL_READ(twi) == 0 && (i++) < TWI_CLOCK_STRETCH);// Clock stretching (up to 100us)
  twi_delay(twi->dcount);
  return true;
}

static bool twi_read_bit(TwiPort* twi) {
  unsigned int i = 0;
  SCL_LOW(twi);
  SDA_HIGH(twi);
  twi_delay(twi->dcount+2);
  SCL_HIGH(twi);
  while (SCL_READ(twi) == 0 && (i++) < TWI_CLOCK_STRETCH);// Clock stretching (up to 100us)
  bool bit = SDA_READ(twi);
  twi_delay(twi->dcount);
  return bit;
}

static bool twi_write_byte(TwiPort* twi, unsigned char byte) {

  unsigned char bit;
  for (bit = 0; bit < 8; bit++) {
    twi_write_bit(twi, (byte & 0x80) != 0);
    byte <<= 1;
  }
  return !twi_read_bit(twi);//NACK/ACK
}

static unsigned char twi_read_byte(TwiPort* twi, bool nack) {
  unsigned char byte = 0;
  unsigned char bit;
  for (bit = 0; bit < 8; bit++) byte = (byte << 1) | twi_read_bit(twi);
  twi_write_bit(twi, nack);
  return byte;
}

unsigned char twi_writeTo(TwiPort* twi, unsigned char address, unsigned char * buf, unsigned int len, unsigned char sendStop){
  unsigned int i;
  if(!twi_write_start(twi)) return 4;//line busy
  if(!twi_write_byte(twi, ((address << 1) | 0) & 0xFF)) {
    if (sendStop) twi_write_stop(twi);
    return 2; //received NACK on transmit of address
  }
  for(i=0; i<len; i++) {
    if(!twi_write_byte(twi, buf[i])) {
      if (sendStop) twi_write_stop(twi);
      return 3;//received NACK on transmit of data
    }
  }
  if(sendStop) twi_write_stop(twi);
  i = 0;
  while(SDA_READ(twi) == 0 && (i++) < 10){
    SCL_LOW(twi);
    twi_delay(twi->dcount);
    SCL_HIGH(twi);
    twi_delay(twi->dcount);
  }
  return 0;
}

unsigned char twi_readFrom(TwiPort* twi, unsigned char address, unsigned char* buf, unsigned int len, unsigned char sendStop){
  unsigned int i;
  if(!twi_write_start(twi)) return 4;//line busy
  if(!twi_write_byte(twi, ((address << 1) | 1) & 0xFF)) {
    if (sendStop) twi_write_stop(twi);
    return 2;//received NACK on transmit of address
  }
  for(i=0; i<(len-1); i++) buf[i] = twi_read_byte(twi, false);
  buf[len-1] = twi_read_byte(twi, true);
  if(sendStop) twi_write_stop(twi);
  i = 0;
  while(SDA_READ(twi) == 0 && (i++) < 10){
    SCL_LOW(twi);
    twi_delay(twi->dcount);
    SCL_HIGH(twi);
    twi_delay(twi->dcount);
  }
  return 0;
}
