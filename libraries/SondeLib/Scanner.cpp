#include "Scanner.h"
#include <SX1278FSK.h>
#include <U8x8lib.h>

#include "Sonde.h"
#include "Display.h"

// Need scanning bandwidth to match receive Bw
// 120 * 6.3 kHz = 0.8 MHz (405 - 405.8)
#define CHANBW (6300)
#define PIXSAMPL (1)
#define SMOOTH (3)
#define STARTF (405e6)
#define NCHAN (128)

int scanresult[NCHAN];
int scandisp[NCHAN/PIXSAMPL];

#define PLOT_N 128
#define TICK1 (128/6)
#define TICK2 (TICK1/4)
//#define PLOT_MIN -250
#define PLOT_MIN (sonde.config.noisefloor*2)
//#define PLOT_SCALE(x) (x<PLOT_MIN?0:(x-PLOT_MIN)/2)
// no need to clip scale, this is handled when plotting
#define PLOT_SCALE(x) ((x-PLOT_MIN)/2)

const byte tilepatterns[9]={0,0x80,0xC0,0xE0,0xF0,0xF8,0xFC,0xFE,0xFF};
void Scanner::fillTiles(uint8_t *row, int value) {
	for(int y=0; y<8; y++) {
		int nbits = value - 8*(7-y);
		if(nbits<0) { row[8*y]=0; continue; }
		if(nbits>=8) { row[8*y]=255; continue; }
		row[8*y] = tilepatterns[nbits];
	}
}
/*
 * There are 16*8 columns to plot, NPLOT must be lower than that
 * currently, we use 128 * 50kHz channels
 * There are 8*8 values to plot; MIN is bottom end, 
 */
uint8_t tiles[16] = { 0x0f,0x0f,0x0f,0x0f,0xf0,0xf0,0xf0,0xf0, 1, 3, 7, 15, 31, 63, 127, 255};
void Scanner::plotResult()
{
	uint8_t row[8*8];
	for(int i=0; i<PLOT_N; i+=8) {
		for(int j=0; j<8; j++) {
			fillTiles(row+j, PLOT_SCALE(scandisp[i+j]));
		        // if( ((i+j)%TICK1)==0) { row[j] |= 0x07; }
		        // if( ((i+j)%TICK2)==0) { row[j] |= 0x01; }
		}
		for(int y=0; y<8; y++) {
			// if(sonde.config.marker && y==1) {
				// don't overwrite MHz marker text
			//	if(i<3*8 || (i>=7*8&&i<10*8) || i>=13*8) continue;
			// }
			disp.rdis->drawTile(i/8, y, 1, row+8*y);
		}
	}
}

void Scanner::scan()
{
	// Configure 
	sx1278.writeRegister(REG_PLL_HOP, 0x80);   // FastHopOn
	sx1278.setRxBandwidth(CHANBW);
	sx1278.writeRegister(REG_RSSI_CONFIG, SMOOTH&0x07);
	sx1278.setFrequency(STARTF);
	sx1278.writeRegister(REG_OP_MODE, FSK_RX_MODE);
	delay(20);

	unsigned long start = millis();
	uint32_t lastfrf=-1;
	{   // one interation: may miss RS41 transmissions
	    for(int i=0; i<NCHAN; i++) {
		float freq = STARTF + i*CHANBW;
		uint32_t frf = freq * 1.0 * (1<<19) / SX127X_CRYSTAL_FREQ;
		if( (lastfrf>>16)!=(frf>>16) ) {
        		sx1278.writeRegister(REG_FRF_MSB, (frf&0xff0000)>>16);
		}
		if( ((lastfrf&0x00ff00)>>8) != ((frf&0x00ff00)>>8) ) {
        		sx1278.writeRegister(REG_FRF_MID, (frf&0x00ff00)>>8);
		}
        	sx1278.writeRegister(REG_FRF_LSB, (frf&0x0000ff));
		lastfrf = frf;
		// Wait TS_HOP (20us) + TS_RSSI ( 2^(SMOOTH+1) / 4 / CHANBW us)
		// int wait = 20 + 1000*(1<<(SMOOTH+1))/4/CHANBW; // 420us * 900 => 480ms
		delayMicroseconds(500);
		int rssi = -(int)sx1278.readRegister(REG_RSSI_VALUE_FSK);
		scanresult[i] = rssi;
	    }
	}
	unsigned long duration = millis()-start;
	Serial.print("Scan time: ");
	Serial.println(duration);
	for(int i=0; i<(NCHAN/PIXSAMPL); i++) {
		// smooth over time
		scandisp[i]=(scanresult[i] + scandisp[i]) / 2 ;
                Serial.print(scandisp[i]); Serial.print(", ");
	}
	Serial.println("\n");
}

Scanner scanner = Scanner();
