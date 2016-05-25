// ENC28J60 Driver
// Madhurkiran Harikrishnan

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "enc28j60.h"
#include "wait.h"

// ------------------------------------------------------------------------------
//  Globals
// ------------------------------------------------------------------------------

uint8_t nextPacketLsb = 0x00;
uint8_t nextPacketMsb = 0x00;
uint8_t sequenceId = 1;
uint32_t sum;
uint8_t macAddress[6] = {0xE8,0x03,0x9A,0x1F,0xBB,0xC1};
uint8_t destMacAddress[6] = {0xac,0x16,0x2d,0x47,0x6a,0x83};

uint8_t destIpAddress[4]={144,76,83,20};
uint8_t ipv4Address[4];
uint16_t received_Checksum;

uint8_t options[20]={2,4,5,0xb4,1,3,3,2,4,2,8,0x0a,0,0x4e,0x80,0xd0,0,0,0,0};
char Request[]={"GET /data/2.5/weather?q=London%20England&units=imperial HTTP/1.1\r\nHost: api.openweathermap.org\r\nConnection: keep-alive\r\nAccept: xml\r\nUser-Agent: Morzilla\r\n\r\n"};

// ------------------------------------------------------------------------------
//  Structures
// ------------------------------------------------------------------------------

// This M4F is little endian
// Network byte order is big endian
// Must uint8_terpret uint16_ts in reverse order

struct enc28j60Frame // 4-bytes
{
  uint16_t size;
  uint16_t status;
  uint8_t data;
} *enc28j60;

struct etherFrame // 14-bytes
{
  uint8_t destAddress[6];
  uint8_t sourceAddress[6];
  uint16_t frameType;
  uint8_t data;
} *ether;

struct _ip // minimum 20 bytes
{
  uint8_t rev_size;
  uint8_t typeOfService;
  uint16_t length;
  uint16_t id;
  uint16_t flagsAndOffset;
  uint8_t ttl;
  uint8_t protocol;
  uint16_t headerChecksum;
  uint8_t sourceIp[4];
  uint8_t destIp[4];
} *ip;

struct _icmp
{
  uint8_t type;
  uint8_t code;
  uint16_t check;
  uint16_t id;
  uint16_t seq_no;
  uint8_t data;
} *icmp;

struct _arp
{
  uint16_t hardwareType;
  uint16_t protocolType;
  uint8_t hardwareSize;
  uint8_t protocolSize;
  uint16_t op;
  uint8_t sourceAddress[6];
  uint8_t sourceIp[4];
  uint8_t destAddress[6];
  uint8_t destIp[4];
} *arp;

struct _udp // 8 bytes
{
  uint16_t sourcePort;
  uint16_t destPort;
  uint16_t length;
  uint16_t check;
  uint8_t  data;
} *udp;


struct _tcp
{
	uint16_t sourcePort;
	uint16_t destPort;
	uint32_t seqNumber;
	uint32_t ackNumber;
	uint8_t  headLength;
	uint8_t  flag;
	uint16_t windowSize;
	uint16_t check;
	uint16_t urgentPointer;
	uint8_t options;

} *tcp;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void spiWrite(uint8_t data)
{
	SSI2_DR_R = data;

	while (SSI2_SR_R & SSI_SR_BSY);
}

uint8_t spiRead()
{
    return SSI2_DR_R;
}

void etherCsOn()
{
    PIN_ETHER_CS = 0;
	__asm (" NOP");                    // allow line to settle
	__asm (" NOP");
	__asm (" NOP");
	__asm (" NOP");
}

void etherCsOff()
{
    PIN_ETHER_CS = 1;
}

void etherWriteReg(uint8_t reg, uint8_t data)
{
    etherCsOn();
    spiWrite(0x40 | (reg & 0x1F));
    spiRead();
    spiWrite(data);
    spiRead();
    etherCsOff();
}

uint8_t etherReadReg(uint8_t reg)
{
    uint8_t data;
    etherCsOn();
    spiWrite(0x00 | (reg & 0x1F));
    spiRead();
    spiWrite(0);
    data = spiRead();
    etherCsOff();
    return data;
}

void etherSetReg(uint8_t reg, uint8_t mask)
{
    etherCsOn();
    spiWrite(0x80 | (reg & 0x1F));
    spiRead();
    spiWrite(mask);
    spiRead();
    etherCsOff();
}

void etherClearReg(uint8_t reg, uint8_t mask)
{
    etherCsOn();
    spiWrite(0xA0 | (reg & 0x1F));
    spiRead();
    spiWrite(mask);
    spiRead();
    etherCsOff();
}

void etherSetBank(uint8_t reg)
{
    etherClearReg(ECON1, 0x03);
    etherSetReg(ECON1, reg >> 5);
}

void etherWritePhy(uint8_t reg, uint16_t data)
{
    etherSetBank(MIREGADR);
    etherWriteReg(MIREGADR, reg);
    etherWriteReg(MIWRL, data & 0xFF);
    etherWriteReg(MIWRH, (data >> 8) & 0xFF);
}

uint16_t etherReadPhy(uint8_t reg)
{
    uint16_t data, data2;
    etherSetBank(MIREGADR);
    etherWriteReg(MIREGADR, reg);
    etherWriteReg(MICMD, etherReadReg(MICMD) | MIIRD);
    waitMicrosecond(50);
    while ((etherReadReg(MISTAT) | MIBUSY) != 0);
    etherWriteReg(MICMD, etherReadReg(MICMD) & ~MIIRD);
    data = etherReadReg(MIRDL);
    data2 = etherReadReg(MIRDH);
    data |= (data2 << 8);
    return data;
}

void etherWriteMemStart()
{
    etherCsOn();
    spiWrite(0x7A);
    spiRead();
}

void etherWriteMem(uint8_t data)
{
    spiWrite(data);
    spiRead();
}

void etherWriteMemStop()
{
    etherCsOff();
}

void etherReadMemStart()
{
    etherCsOn();
    spiWrite(0x3A);
    spiRead();
}

uint8_t etherReadMem()
{
    spiWrite(0);
    return spiRead();
}

void etherReadMemStop()
{
    etherCsOff();
}

// Initializes ethernet device
// Uses order suggested in Chapter 6 of datasheet except 6.4 OST which is first here
void etherInit(uint8_t mode)
{
    // make sure that oscillator start-up timer has expired
    while ((etherReadReg(ESTAT) & CLKRDY) == 0) {}

    // disable transmission and reception of packets
    etherClearReg(ECON1, RXEN);
    etherClearReg(ECON1, TXRTS);

    // initialize receive buffer space
    etherSetBank(ERXSTL);
    etherWriteReg(ERXSTL, LOBYTE(0x0000));
    etherWriteReg(ERXSTH, HIBYTE(0x0000));
    etherWriteReg(ERXNDL, LOBYTE(0x1A09));
    etherWriteReg(ERXNDH, HIBYTE(0x1A09));

    // initialize receiver write and read ptrs
    // at startup, will write from 0 to 1A08 only and will not overwrite rd ptr
    etherWriteReg(ERXWRPTL, LOBYTE(0x0000));
    etherWriteReg(ERXWRPTH, HIBYTE(0x0000));
    etherWriteReg(ERXRDPTL, LOBYTE(0x1A09));
    etherWriteReg(ERXRDPTH, HIBYTE(0x1A09));
    etherWriteReg(ERDPTL, LOBYTE(0x0000));
    etherWriteReg(ERDPTH, HIBYTE(0x0000));

    // setup receive filter
    // always check CRC, use OR mode
    etherSetBank(ERXFCON);
    etherWriteReg(ERXFCON, (mode | 0x20) & 0xBF);
    // bring mac out of reset
    etherSetBank(MACON2);
    etherWriteReg(MACON2, 0);

    // enable mac rx, enable pause control for full duplex
    etherWriteReg(MACON1, TXPAUS | RXPAUS | MARXEN);

    // enable padding to 60 bytes (no runt packets)
    // add crc to tx packets, set full or half duplex
    if ((mode & ETHER_FULLDUPLEX) != 0)
        etherWriteReg(MACON3, FULDPX | FRMLNEN | TXCRCEN | PAD60);
    else
        etherWriteReg(MACON3, FRMLNEN | TXCRCEN | PAD60);

    // leave MACON4 as reset

    // set maximum rx packet size
    etherWriteReg(MAMXFLL, LOBYTE(1518));
    etherWriteReg(MAMXFLH, HIBYTE(1518));

    // set back-to-back uint8_ter-packet gap to 9.6us
    if ((mode & ETHER_FULLDUPLEX) != 0)
        etherWriteReg(MABBIPG, 0x15);
    else
        etherWriteReg(MABBIPG, 0x12);

    // set non-back-to-back uint8_ter-packet gap registers
    etherWriteReg(MAIPGL, 0x12);
    etherWriteReg(MAIPGH, 0x0C);

    // leave collision window MACLCON2 as reset

    // setup mac address
    etherSetBank(MAADR0);
    etherWriteReg(MAADR5, macAddress[0]);
    etherWriteReg(MAADR4, macAddress[1]);
    etherWriteReg(MAADR3, macAddress[2]);
    etherWriteReg(MAADR2, macAddress[3]);
    etherWriteReg(MAADR1, macAddress[4]);
    etherWriteReg(MAADR0, macAddress[5]);

    // initialize phy duplex
    if ((mode & ETHER_FULLDUPLEX) != 0)
        etherWritePhy(PHCON1, PDPXMD);
    else
        etherWritePhy(PHCON1, 0);

    // disable phy loopback if in half-duplex mode
    etherWritePhy(PHCON2, HDLDIS);

    // set LEDA (link status) and LEDB (tx/rx activity)
    // stretch LED on to 40ms (default)
    etherWritePhy(PHLCON, 0x0472);

    // enable reception
    etherSetReg(ECON1, RXEN);
}

// Returns TRUE if packet received
uint8_t etherKbhit()
{
    return ((etherReadReg(EIR) & PKTIF) != 0);
}

// Returns up to max_size characters in data buffer
// Returns number of bytes copied to buffer
// Contents written are 16-bit size, 16-bit status, payload excl crc
uint16_t etherGetPacket(uint8_t data[], uint16_t max_size)
{
    uint16_t i = 0, size, tmp;

    // enable read from FIFO buffers
    etherReadMemStart();

    // get next pckt information
    nextPacketLsb = etherReadMem();
    nextPacketMsb = etherReadMem();

    // calc size
    // don't return crc, instead return size + status, so size is correct
    size = etherReadMem();
    data[i++] = size;
    tmp = etherReadMem();
    data[i++] = tmp;
    size |= (tmp << 8);

    // copy status + data
    if (size > max_size)
        size = max_size;
    while (i < size)
        data[i++] = etherReadMem();

    // end read from FIFO buffers
    etherReadMemStop();

    // advance read ptr
    etherSetBank(ERXRDPTL);
    etherWriteReg(ERXRDPTL, nextPacketLsb); // hw ptr
    etherWriteReg(ERXRDPTH, nextPacketMsb);
    etherWriteReg(ERDPTL, nextPacketLsb); // dma rd ptr
    etherWriteReg(ERDPTH, nextPacketMsb);

    // decrement packet counter so that PKTIF is mauint8_tained correctly
    etherSetReg(ECON2, PKTDEC);

    return size;
}

// Returns TRUE is rx buffer overflowed after correcting the problem
uint8_t etherIsOverflow()
{
    uint8_t err;
    err = (etherReadReg(EIR) & RXERIF) != 0;
    if (err)
        etherClearReg(EIR, RXERIF);
    return err;
}

// Writes a packet
bool etherPutPacket(uint8_t data[], uint16_t size)
{
    uint16_t i;

    // clear out any tx errors
    if ((etherReadReg(EIR) & TXERIF) != 0)
    {
        etherClearReg(EIR, TXERIF);
        etherSetReg(ECON1, TXRTS);
        etherClearReg(ECON1, TXRTS);
    }

    // set DMA start address
    etherSetBank(EWRPTL);
    etherWriteReg(EWRPTL, LOBYTE(0x1A0A));
    etherWriteReg(EWRPTH, HIBYTE(0x1A0A));

    // start FIFO buffer write
    etherWriteMemStart();

    // write control byte
    etherWriteMem(0);

    // write data
    for (i = 0; i < size; i++)
        etherWriteMem(data[i]);

    // stop write
    etherWriteMemStop();

    // request transmit
    etherWriteReg(ETXSTL, LOBYTE(0x1A0A));
    etherWriteReg(ETXSTH, HIBYTE(0x1A0A));
    etherWriteReg(ETXNDL, LOBYTE(0x1A0A+size));
    etherWriteReg(ETXNDH, HIBYTE(0x1A0A+size));
    etherClearReg(EIR, TXIF);
    etherSetReg(ECON1, TXRTS);

    // wait for completion
    while ((etherReadReg(ECON1) & TXRTS) != 0);

    // determine success
    return ((etherReadReg(ESTAT) & TXABORT) == 0);
}

// Calculate sum of words
// Must use getEtherChecksum to complete 1's compliment addition
void etherSumWords(void* data, uint16_t size_in_bytes)
{
	uint8_t* pData = (uint8_t*)data;
    uint16_t i;
    uint8_t phase = 0;
    uint16_t data_temp;
    for (i = 0; i < size_in_bytes; i++)
    {
        if (phase)
        {
            data_temp = *pData;
            sum += data_temp << 8;
        }
        else
          sum += *pData;
        phase = 1 - phase;
        pData++;
    }
}

void myEtherSumWords(void* data,uint16_t size_in_bytes)
{
	uint8_t* pData = (uint8_t*)data;
    uint16_t i;
    uint8_t phase = 0;
    uint16_t data_temp;
    for (i = 0; i < size_in_bytes; i++)
    {
        if (phase==0)
        {
            data_temp = *pData;
            sum += data_temp << 8;
        }
        else
          sum += *pData;
        phase = 1 - phase;
        pData++;
    }
}

void EndianConversion(void* data,uint16_t size_in_bytes)
{
	uint8_t* pData = (uint8_t*)data;
    uint8_t* pData2= (uint8_t*)data;
    pData2++;
    uint16_t i;
    uint8_t phase = 0,temp;
    uint16_t data_temp;
    size_in_bytes=size_in_bytes-(size_in_bytes%2);

    size_in_bytes=size_in_bytes/2;
    for (i = 0; i < size_in_bytes; i++)
    {
        temp=*pData;
        *pData=*pData2;
        *pData2=temp;
        pData +=2;
        pData2 +=2;
    }
}





// Completes 1's compliment addition by folding carries back uint8_to field

uint16_t getEtherChecksum()
{
    uint16_t result;
    // this is based on rfc1071
    while ((sum >> 16) > 0)
      sum = (sum & 0xFFFF) + (sum >> 16);
    result = sum & 0xFFFF;
    return ~result;
}

// Converts from host to network order and vice versa
uint16_t htons(uint16_t value)
{
    return ((value & 0xFF00) >> 8) + ((value & 0x00FF) << 8);
}

#define ntohs htons

// Determines whether packet is IP datagram
uint8_t etherIsIp(uint8_t data[])
{
    uint8_t ok;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    ok = (ether->frameType == 0x0008);
    if (ok)
    {
        sum = 0;
        etherSumWords(&ip->rev_size, (ip->rev_size & 0xF) * 4);
        ok = (getEtherChecksum() == 0);
    }
    return ok;
}

// Determines whether packet is unicast to this ip
// Must be an IP packet
bool etherIsIpUnicast(uint8_t data[])
{
    uint8_t i = 0;
    bool ok = true;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    while (ok && (i < 4))
    {
        ok = (ip->destIp[i] == ipv4Address[i]);
        i++;
    }
    return ok;
}

// Determines whether packet is ping request
// Must be an IP packet
uint8_t etherIsPingReq(uint8_t data[])
{
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    icmp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    return (ip->protocol == 0x01 && icmp->type == 8);
}

// Sends a ping response given the request data
void etherSendPingResp(uint8_t data[])
{
    uint8_t i, tmp;
    uint16_t icmp_size;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    icmp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    // swap source and destination fields
    for (i = 0; i < 6; i++)
    {
        tmp = ether->destAddress[i];
        ether->destAddress[i] = ether->sourceAddress[i];
        ether->sourceAddress[i] = tmp;
    }
    for (i = 0; i < 4; i++)
    {
        tmp = ip->destIp[i];
        ip->destIp[i] = ip ->sourceIp[i];
        ip->sourceIp[i] = tmp;
    }
    // this is a response
    icmp->type = 0;
    // calc icmp checksum
    sum = 0;
    etherSumWords(&icmp->type, 2);
    icmp_size = ntohs(ip->length);
    icmp_size -= 24; // sub ip header and icmp code, type, and check
    etherSumWords(&icmp->id, icmp_size);
    icmp->check = getEtherChecksum();
    // send packet
    etherPutPacket((uint8_t*)ether, 14 + ntohs(ip->length));
}


// Determines whether packet is ARP
uint8_t etherIsArp(uint8_t data[])
{
    uint8_t ok;
    uint8_t i = 0;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    arp = (void*)&ether->data;
    ok = (ether->frameType == 0x0608);
    while (ok && (i < 4))
    {
        ok = (arp->destIp[i] == ipv4Address[i]);
        i++;
    }
    return ok;
}

// Sends an ARP response given the request data
void etherSendArpResp(uint8_t data[])
{
    uint8_t i, tmp;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    arp = (void*)&ether->data;
    // set op to response
    arp->op = 0x0200;
    // swap source and destination fields
    for (i = 0; i < 6; i++)
    {
        arp->destAddress[i] = arp->sourceAddress[i];
        ether->destAddress[i] = ether->sourceAddress[i];
        ether->sourceAddress[i] = arp->sourceAddress[i] = macAddress[i];
    }
    for (i = 0; i < 4; i++)
    {
        tmp = arp->destIp[i];
        arp->destIp[i] = arp->sourceIp[i];
        arp->sourceIp[i] = tmp;
    }
    // send packet
    etherPutPacket((uint8_t*)ether, 42);
}

// Sends an ARP request
void etherSendArpReq(uint8_t data[], uint8_t ip[])
{
    uint8_t i;
    ether = (void*)data;
    arp = (void*)&ether->data;
    // fill ethernet frame
    for (i = 0; i < 6; i++)
    {
        ether->destAddress[i] = 0xFF;
        ether->sourceAddress[i] = macAddress[i];
    }
    ether->frameType = 0x0608;
    // fill arp frame
    arp->hardwareType = 0x0100;
    arp->protocolType = 0x0008;
    arp->hardwareSize = 6;
    arp->protocolSize = 4;
    arp->op = 0x0100;
    for (i = 0; i < 6; i++)
    {
        arp->sourceAddress[i] = macAddress[i];
        arp->destAddress[i] = 0xFF;
    }
    for (i = 0; i < 4; i++)
    {
        arp->sourceIp[i] = ipv4Address[i];
        arp->destIp[i] = ip[i];
    }
    // send packet
    etherPutPacket(data, 42);
}

// Determines whether packet is UDP datagram
// Must be an IP packet
uint8_t etherIsUdp(uint8_t data[])
{
    uint8_t ok;
    uint16_t tmp_int;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    udp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    ok = (ip->protocol == 0x11);
    if (ok)
    {
        // 32-bit sum over pseudo-header
        sum = 0;
        etherSumWords(ip->sourceIp, 8);
        tmp_int = ip->protocol;
        sum += (tmp_int & 0xff) << 8;
        etherSumWords(&udp->length, 2);

        // add udp header and data
        etherSumWords(udp, ntohs(udp->length));
        ok = (getEtherChecksum() == 0);
    }
    return ok;
}

// Determines whether packet is TCP datagram
// Must be an IP packet

uint8_t etherIsTcp(uint8_t data[])
{
    uint8_t ok;
    uint16_t tmp_int;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    tcp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    ok = (ip->protocol == 0x6);

    if (ok)
    {
        // 32-bit sum over pseudo-header
        sum = 0;
        myEtherSumWords(ip->sourceIp, 8);
        tmp_int = ip->protocol;
        sum += tmp_int ;

        //Tcp header length
        tmp_int=((tcp->headLength & 0xf0)>>4)*4;
        sum+=tmp_int;


        //Tcp data length
        tcp_DataCount = (ntohs(ip->length))-tmp_int-((ip->rev_size & 0xF) * 4);
        sum += tcp_DataCount;

        // add TCP header and data
        received_Checksum=ntohs(tcp->check);
        tcp->check=0;


        myEtherSumWords(tcp, tmp_int+tcp_DataCount);


        ok=(getEtherChecksum() == received_Checksum);
    }
    return ok;
}


uint32_t htons32(uint32_t value)

{
    return ((value & 0xFF000000) >> 24) + ((value & 0x00FF0000) >> 8)+ ((value & 0x0000FF00) << 8)+ ((value & 0x000000FF) << 24);
}




void addOptions(void* data)
{
	uint8_t i;
	uint8_t* optData=(void*)data;
	for(i=0;i<20;i++)
	  {
		*optData=options[i];
		optData++;
	  }

}

void sendSynWeather(uint8_t data[])
{
	uint8_t i;
	uint16_t tmp_int;
	ether=(void*)data;
    ip = (void*)&ether->data;



	// fill ethernet frame
	for (i = 0; i < 6; i++)
    {
		ether->destAddress[i] = destMacAddress[i];
	    ether->sourceAddress[i] = macAddress[i];
	}
	ether->frameType = 0x0008;

	//IP
	ip->rev_size =0x45;
	ip->typeOfService=0x00;
	ip->length=0x3c00;
	ip->id =ntohs(id);   //random
	id++;

	ip->flagsAndOffset =0x0040;
	ip->ttl=0x80;
	ip->protocol=0x06;

	ip->headerChecksum=00;

	for (i = 0; i < 4; i++)
    {
		ip->destIp[i] = destIpAddress[i];
	    ip->sourceIp[i] = ipv4Address[i];
	}

	//IP checksum
    sum = 0;
    ip->headerChecksum=0;
    myEtherSumWords(&ip->rev_size, 20);
    ip->headerChecksum = ntohs(getEtherChecksum());


    tcp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    //TCP
    tcp->sourcePort=myport;
    tcp->destPort= 0x5000;
    tcp->seqNumber=htons32(Seq_number);
    tcp->ackNumber=0x00;
    tcp->headLength=0xA0;
    tcp->flag=0x02;
    tcp->windowSize=0xb004;
    tcp->check=0;
    tcp->urgentPointer=0x00;

    addOptions(&tcp->options);





    // 32-bit sum over pseudo-header
       sum = 0;
       myEtherSumWords(ip->sourceIp, 8);
       tmp_int = ip->protocol;
       sum += tmp_int ;

       //Tcp header length
       tmp_int=((tcp->headLength & 0xf0)>>4)*4;
       sum+=tmp_int;




       myEtherSumWords(tcp, tmp_int);



       tcp->check=ntohs(getEtherChecksum());

     etherPutPacket((uint8_t*)ether, 14 + 60);




}


void putData(void* data,uint16_t count)
{
	uint16_t i;
	uint8_t* optData=(void*)data;
	for(i=0;i<count;i++)
	  {
		*optData=Request[i];
		optData++;
	  }

}

void sendGETRequest(uint8_t data[])
{
	uint8_t i;
	uint16_t tmp_int;
	ether=(void*)data;
    ip = (void*)&ether->data;



	// fill ethernet frame
	for (i = 0; i < 6; i++)
    {
		ether->destAddress[i] = destMacAddress[i];
	    ether->sourceAddress[i] = macAddress[i];
	}
	ether->frameType = 0x0008;

	//IP
	ip->rev_size =0x45;
	ip->typeOfService=0x00;
	ip->length=ntohs(40+sizeof(Request));
	ip->id =ntohs(id);//random
	id++;
	ip->flagsAndOffset =0x0040;
	ip->ttl=0x80;
	ip->protocol=0x06;

	ip->headerChecksum=00;

	for (i = 0; i < 4; i++)
    {
		ip->destIp[i] = destIpAddress[i];
	    ip->sourceIp[i] = ipv4Address[i];
	}

	//IP checksum
    sum = 0;
    ip->headerChecksum=0;
    myEtherSumWords(&ip->rev_size, 20);
    ip->headerChecksum = ntohs(getEtherChecksum());


    tcp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    //TCP
    tcp->sourcePort=myport;
    tcp->destPort= 0x5000;
    tcp->seqNumber=htons32(Seq_number);
    tcp->ackNumber=(Ack_number);
    tcp->headLength=0x50;
    tcp->flag=0x18;
    tcp->windowSize=0xb004;
    tcp->check=0;
    tcp->urgentPointer=0x00;

    //addOptions(&tcp->options);

    tcp_DataCount=sizeof(Request);

    putData(&tcp->options,tcp_DataCount);



    // 32-bit sum over pseudo-header
       sum = 0;
       myEtherSumWords(ip->sourceIp, 8);
       tmp_int = ip->protocol;
       sum += tmp_int;

       sum= sum+20+tcp_DataCount;


       myEtherSumWords(tcp, 20+tcp_DataCount);

       tcp->check=ntohs(getEtherChecksum());

     etherPutPacket((uint8_t*)ether, 14 + 40+tcp_DataCount);

}

void sendFinRequest(uint8_t data[])
{
	uint8_t i;
	uint16_t tmp_int;
	ether=(void*)data;
    ip = (void*)&ether->data;



	// fill ethernet frame
	for (i = 0; i < 6; i++)
    {
		ether->destAddress[i] = destMacAddress[i];
	    ether->sourceAddress[i] = macAddress[i];
	}
	ether->frameType = 0x0008;

	//IP
	ip->rev_size =0x45;
	ip->typeOfService=0x00;
	ip->length=ntohs(40);
	ip->id =ntohs(id);//random
	id++;
	ip->flagsAndOffset =0x0040;
	ip->ttl=0x80;
	ip->protocol=0x06;

	ip->headerChecksum=00;

	for (i = 0; i < 4; i++)
    {
		ip->destIp[i] = destIpAddress[i];
	    ip->sourceIp[i] = ipv4Address[i];
	}

	//IP checksum
    sum = 0;
    ip->headerChecksum=0;
    myEtherSumWords(&ip->rev_size, 20);
    ip->headerChecksum = ntohs(getEtherChecksum());


    tcp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    //TCP
    tcp->sourcePort=myport;
    tcp->destPort= 0x5000;
    tcp->seqNumber=htons32(Seq_number);
    tcp->ackNumber=(Ack_number);
    tcp->headLength=0x50;
    tcp->flag=0x01;
    tcp->windowSize=0xb004;
    tcp->check=0;
    tcp->urgentPointer=0x00;

    //addOptions(&tcp->options);

    //tcp_DataCount=sizeof(Request);

    //putData(&tcp->options,tcp_DataCount);



    // 32-bit sum over pseudo-header
       sum = 0;
       myEtherSumWords(ip->sourceIp, 8);
       tmp_int = ip->protocol;
       sum += tmp_int;

       sum= sum+20;


       myEtherSumWords(tcp, 20);

       tcp->check=ntohs(getEtherChecksum());

     etherPutPacket((uint8_t*)ether, 14 + 40);

}




// Gets pouint8_ter to UDP payload of frame
uint8_t* etherGetUdpData(uint8_t data[])
{
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    udp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    return &udp->data;
}




// Gets pouint8_ter to TCP payload of frame
uint8_t* etherGetTcpData(uint8_t data[])
{

    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    tcp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    return &tcp->options + (((tcp->headLength & 0xf0)>>4)*4)-20;
}



void etherCalcIpChecksum()
{
    // 32-bit sum over ip header
    sum = 0;
    etherSumWords(&ip->rev_size, 10);
    etherSumWords(ip->sourceIp, ((ip->rev_size & 0xF) * 4) - 12);
    ip->headerChecksum = getEtherChecksum();
}

// Send responses to a udp datagram
// destination port, ip, and hardware address are extracted from provided data
// uses destination port of received packet as destination of this packet
void etherSendUdpData(uint8_t data[], uint8_t* udp_data, uint8_t udp_size)
{
    uint8_t *copy_data;
    uint8_t i, tmp;
    uint16_t tmp_int;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    udp = (void*)((uint8_t*)&ether->data + ((ip->rev_size & 0xF) * 4));

    // swap source and destination fields
    for (i = 0; i < 6; i++)
    {
        tmp = ether->destAddress[i];
        ether->destAddress[i] = ether->sourceAddress[i];
        ether->sourceAddress[i] = tmp;
    }
    for (i = 0; i < 4; i++)
    {
        tmp = ip->destIp[i];
        ip->destIp[i] = ip->sourceIp[i];
        ip->sourceIp[i] = tmp;
    }
    // set source port of resp will be dest port of req
    // dest port of resp will be left at source port of req
    // unusual nomenclature, but this allows a different tx
    // and rx port on other machine
    udp->sourcePort = udp->destPort;
    // adjust lengths
    ip->length = htons(((ip->rev_size & 0xF) * 4) + 8 + udp_size);
    // 32-bit sum over ip header
    sum = 0;
    etherSumWords(&ip->rev_size, 10);
    etherSumWords(ip->sourceIp, ((ip->rev_size & 0xF) * 4) - 12);
    ip->headerChecksum = getEtherChecksum();
    udp->length = htons(8 + udp_size);
    // copy data
    copy_data = &udp->data;
    for (i = 0; i < udp_size; i++)
        copy_data[i] = udp_data[i];
        // 32-bit sum over pseudo-header
    sum = 0;
    etherSumWords(ip->sourceIp, 8);
    tmp_int = ip->protocol;
    sum += (tmp_int & 0xff) << 8;
    etherSumWords(&udp->length, 2);
    // add udp header except crc
    etherSumWords(udp, 6);
    etherSumWords(&udp->data, udp_size);
    udp->check = getEtherChecksum();

    // send packet with size = ether + udp hdr + ip header + udp_size
    etherPutPacket((uint8_t*)ether, 22 + ((ip->rev_size & 0xF) * 4) + udp_size);
}


bool etherisHandshake1(uint8_t data[])
{
	enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    tcp = (void*)((uint8_t*)&ether->data + ((ip->rev_size & 0xF) * 4));
    return (tcp->flag & 0xFF)==0x12;
}


void etherRespondtoHandshake1(uint8_t data[])
{
		uint8_t i, tmp;
	    uint16_t tmp_int;
	    uint32_t temp_seq;
	    uint16_t temp_Port;


	    enc28j60 = (void*)data;
	    ether = (void*)&enc28j60->data;
	    ip = (void*)&ether->data;
	    tcp = (void*)((uint8_t*)&ether->data + ((ip->rev_size & 0xF) * 4));

	    // swap source and destination fields
	    for (i = 0; i < 6; i++)
	    {
	        tmp = ether->destAddress[i];
	        ether->destAddress[i] = ether->sourceAddress[i];
	        ether->sourceAddress[i] = tmp;
	    }
	    for (i = 0; i < 4; i++)
	    {
	        tmp = ip->destIp[i];
	        ip->destIp[i] = ip->sourceIp[i];
	        ip->sourceIp[i] = tmp;
	    }

	    temp_Port=tcp->destPort;
	    tcp->destPort=tcp->sourcePort;
	    tcp->sourcePort=temp_Port;
	    tcp->flag = 0x10;
	    temp_seq=tcp->ackNumber;
	    Seq_number++;
	    tcp->ackNumber=htons32(htons32(tcp->seqNumber)+1);
	    tcp->seqNumber=temp_seq;
	    Ack_number=tcp->ackNumber;

	    tcp->windowSize=0xb004;
	   // Ack_number=htons32(Ack_number);


	    sum = 0;
	    myEtherSumWords(ip->sourceIp, 8);
	    tmp_int = ip->protocol;
	    sum += tmp_int ;

	    tmp_int=((tcp->headLength & 0xf0)>>4)*4;
	    sum+=tmp_int;



	    // add TCP header and data
	    tcp->check=0;
	    myEtherSumWords(tcp, tmp_int);
	    tcp->check=ntohs(getEtherChecksum());

	    etherPutPacket((uint8_t*)ether, 14 + ((ip->rev_size & 0xF) * 4) + tmp_int);
}




void etherRespondToData(uint8_t data[])
{

			uint8_t i, tmp;
		    uint16_t tmp_int;
		    uint32_t temp_seq;
		    uint16_t temp_Port;
		    static uint8_t flag=0x11;
		   // uint8_t flag=0x11;
		    enc28j60 = (void*)data;
		    ether = (void*)&enc28j60->data;
		    ip = (void*)&ether->data;
		    tcp = (void*)((uint8_t*)&ether->data + ((ip->rev_size & 0xF) * 4));

		    // swap source and destination fields
		    for (i = 0; i < 6; i++)
		    {
		        tmp = ether->destAddress[i];
		        ether->destAddress[i] = ether->sourceAddress[i];
		        ether->sourceAddress[i] = tmp;
		    }
		    for (i = 0; i < 4; i++)
		    {
		        tmp = ip->destIp[i];
		        ip->destIp[i] = ip->sourceIp[i];
		        ip->sourceIp[i] = tmp;
		    }

		    tmp_int=((tcp->headLength & 0xf0)>>4)*4;
		    tcp_DataCount = (ntohs(ip->length))-tmp_int-((ip->rev_size & 0xF) * 4);
		    ip->id=id;
		    id++;

		    temp_Port=tcp->destPort;
		    tcp->destPort=tcp->sourcePort;
		    tcp->sourcePort=temp_Port;
		    tcp->flag = flag;
		    temp_seq=tcp->ackNumber;
		    tcp->ackNumber=htons32(tcp->seqNumber)+tcp_DataCount;
		    tcp->ackNumber=htons32(tcp->ackNumber);
		    tcp->seqNumber=temp_seq;



		    //ip->checksum
		    ip->length=ntohs(20+((tcp->headLength & 0xf0)>>4)*4);

		    sum = 0;
		    ip->headerChecksum=0;
		    myEtherSumWords(&ip->rev_size, 20);
		    ip->headerChecksum = ntohs(getEtherChecksum());


/*
		    myDataCheck=&tcp->options + (((tcp->headLength & 0xf0)>>4)*4)-20;
		    *myDataCheck='m';
		    myDataCheck++;
		    *myDataCheck='a';
		    myDataCheck++;
		    *myDataCheck='d';
		    myDataCheck++;
*/

		    //tcp->checksum
		    sum = 0;
		    myEtherSumWords(ip->sourceIp, 8);
		    tmp_int = ip->protocol;
		    sum += tmp_int ;

		    tmp_int=((tcp->headLength & 0xf0)>>4)*4;
		    sum+=tmp_int;

		    //Tcp data length
		    //  tcp_DataCount = (ntohs(ip->length))-tmp_int-((ip->rev_size & 0xF) * 4);
		    //  sum += tcp_DataCount;


		    // add TCP header and data
		    tcp->check=0;
		    myEtherSumWords(tcp, tmp_int);
		    tcp->check=ntohs(getEtherChecksum());

		    etherPutPacket((uint8_t*)ether, 14 + ((ip->rev_size & 0xF) * 4) + tmp_int);
		    flag=0x11;
}


bool etherisHandshake3(uint8_t data[])
{
	if(tcp->destPort==0x5000 | tcp->destPort==0x1700)
	{
		enc28j60 = (void*)data;
		ether = (void*)&enc28j60->data;
		ip = (void*)&ether->data;
		tcp = (void*)((uint8_t*)&ether->data + ((ip->rev_size & 0xF) * 4));
		return (tcp->flag & 0x01F)==0x010;
	}
	else
		return 0;

}
bool etherisTcpClose(uint8_t data[])
{

	if(tcp->destPort==0x5000 | tcp->destPort==0x1700)
	{
		enc28j60 = (void*)data;
		ether = (void*)&enc28j60->data;
		ip = (void*)&ether->data;
		tcp = (void*)((uint8_t*)&ether->data + ((ip->rev_size & 0xF) * 4));
		return (tcp->flag & 0x01F)==0x01;
	}
	else
		return 0;


}


void etherResponsetoTcpClose(uint8_t  data[])
{

	uint8_t i, tmp;
    uint16_t tmp_int;
    uint32_t temp_seq=0x01000000;
    uint16_t temp_Port;


    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    tcp = (void*)((uint8_t*)&ether->data + ((ip->rev_size & 0xF) * 4));

    // swap source and destination fields
    for (i = 0; i < 6; i++)
    {
        tmp = ether->destAddress[i];
        ether->destAddress[i] = ether->sourceAddress[i];
        ether->sourceAddress[i] = tmp;
    }
    for (i = 0; i < 4; i++)
    {
        tmp = ip->destIp[i];
        ip->destIp[i] = ip->sourceIp[i];
        ip->sourceIp[i] = tmp;
    }

    temp_Port=tcp->destPort;
    tcp->destPort=tcp->sourcePort;
    tcp->sourcePort=temp_Port;
    tcp->flag = 0x11;
    tcp->ackNumber=(tcp->seqNumber)+temp_seq;
    tcp->seqNumber=tcp->seqNumber-5;

    sum = 0;
    myEtherSumWords(ip->sourceIp, 8);
    tmp_int = ip->protocol;
    sum += tmp_int ;

    tmp_int=((tcp->headLength & 0xf0)>>4)*4;
    sum+=tmp_int;



    // add TCP header and data
    tcp->check=0;
    myEtherSumWords(tcp, tmp_int);
    tcp->check=ntohs(getEtherChecksum());

    etherPutPacket((uint8_t*)ether, 14 + ((ip->rev_size & 0xF) * 4) + tmp_int);




}

uint16_t etherGetId()
{
    return htons(sequenceId);
}

void etherIncId()
{
    sequenceId++;
}

// Determines if the IP address is valid
bool etherIsValidIp()
{
    return ipv4Address[0] || ipv4Address[1] || ipv4Address[2] || ipv4Address[3];
}

// Sets IP address
void etherSetIpAddress(uint8_t a, uint8_t b,  uint8_t c, uint8_t d)
{
    ipv4Address[0] = a;
    ipv4Address[1] = b;
    ipv4Address[2] = c;
    ipv4Address[3] = d;
}


