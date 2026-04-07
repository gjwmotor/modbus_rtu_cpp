/*
 * Modbus.h
 * ModbusRtu硬件接口层程序
 * 日期: 2026.01.15
 * 作者: txl
 */


#include "Modbus.h"

Modbus::Modbus()
{
	pSerial = NULL;
}

void Modbus::begin(HardwareSerial *_pSerial)
{
	pSerial = _pSerial;
}

void Modbus::begin(HardwareSerial *_pSerial, long u32speed)
{
	pSerial = _pSerial;
	pSerial->begin(u32speed);
	txEnd_T35 = ((1000000*35)/u32speed);
}

void Modbus::setTimeOut(uint16_t u16timeOut)
{
	pSerial->setTimeout(u16timeOut);
}

int Modbus::readModbus(uint8_t *nDat, uint8_t nLen)
{
	return pSerial->readBytes(nDat, nLen);
}

int Modbus::writeModbus(uint8_t *nDat, uint8_t nLen)
{
	if(nDat==NULL){
		return 0;
	}
	return pSerial->write(nDat, nLen);
}

void Modbus::flushModbusRx()
{
	while(pSerial->read()!=-1);
}

void Modbus::flushModbusTx()
{
	pSerial->flush();
	delayMicroseconds(txEnd_T35);
}