/*

WD1793 Floppy Disk controller emulation

Copyright ©2017 Juan Carlos González Amestoy

(Adaptation to ESPectrum / Betadisk (C) 2025 Víctor Iborra [Eremus])

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include <stdlib.h>
#include <stdio.h>
#include "wd1793.h"
#include "Debug.h"
#include "Config.h"
#include "CPU.h"
#include "OSDMain.h"
#include "messages.h"

static bool sclConvertToTRD(rvmWD1793 *wd);

// #pragma GCC optimize("O3")

//Step rates
static const uint32_t srate[8][4]={
  {1500,3000,5000,7500}, //1mhz mfm test
  {750,1500,2500,3750}, //2mhz mfm test
  {1500,3000,5000,7500}, //1mhz fm test
  {750,1500,2500,3750}, //2mhz fm test
  {92,95,99,104}, //1mhz mfm !test
  {46,47,49,52}, //2mhz mfm !test
  {92,95,99,104}, //1mhz fm !test
  {46,47,49,52}, //2mhz fm !test
};

// Sectdatapos = (cursect * 392) + 146 + 16;
static const uint16_t sectdatapos[16]= { 162,554,946,1338,1730,2122,2514,2906,3298,3690,4082,4474,4866,5258,5650,6042 };

#define mark 0xa1a1a1
#define indexMark 0xC2
#define sectorMark 0xA1

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#if !PICO_RP2040
static uint16_t vgCrc(uint16_t crc, uint8_t byte);
#endif

#ifndef ESP_PLATFORM
#define heap_caps_calloc(n, size, caps) calloc(n, size)
#define MALLOC_CAP_8BIT 0
#endif

IRAM_ATTR static void _end(rvmWD1793 *wd) {
  wd->status &= ~kRVMWD177XStatusBusy;
  wd->state = kRVMWD177XNone;
  wd->stepState = kRVMWD177XStepIdle;
  wd->control &= ~(kRVMWD177XWriting|kRVMWD177XDRQ);
  wd->retry = 15;
  wd->led = 0;
  wd->control |= kRVMWD177XINTRQ; //TODO: ADD A INTERRUPT HANDLER
}

IRAM_ATTR void _do(rvmWD1793 *wd) {

  switch(wd->state) {

    case kRVMWD177XSettingHeader: {

      if(wd->control & kRVMWD177XHLD) { // Head load
        wd->state=wd->next;
        _do(wd);
        return;
      }

      // RVM plays motor audio sample here
      wd->led = 1;

      wd->control|=kRVMWD177XHLD;
      // wd->c=kRVMWD177XSettingHeaderTime * ((wd->control&kRVMWD177XTest)?1:0);
      wd->c=1;
      wd->state=kRVMWD177XSettingEnd;
      wd->stepState=kRVMWD177XStepWaiting;
      return;
    }

    case kRVMWD177XSettingEnd: {
      wd->control|=kRVMWD177XHLT;
      wd->state=wd->next;
      _do(wd);
      return;
    }

    case kRVMWD177XTypeI0: {
      if(wd->command & kRVMWD177XHeadBit) {
        wd->next=kRVMWD177XTypeI1;
        wd->state=kRVMWD177XSettingHeader;
      } else {
        wd->control&=~(kRVMWD177XHLD|kRVMWD177XHLT);
        wd->state=kRVMWD177XTypeI1;
      }
      _do(wd);
      return;
    }

    case kRVMWD177XTypeI1: {
      if(wd->command & kRVMWD177XStepInOut) {
         //StepIn or stepOut
         if(wd->command & 0x20) {
           //Step Out
           //printf("Step Out\n");
           wd->control&=~kRVMWD177XDire;
         } else {
           //Step In
           //printf("Step In\n");
           wd->control|=kRVMWD177XDire;

         }
         if(wd->command & kRVMWD177XUpdateBit) {
           wd->state=kRVMWD177XTypeIUpdate;
         } else {
           wd->state=kRVMWD177XTypeISeek;
         }
      } else {
        if(wd->command & 0x20) {//Step
          if(wd->command & kRVMWD177XUpdateBit) {
            wd->state=kRVMWD177XTypeIUpdate;
          } else {
            wd->state=kRVMWD177XTypeISeek;
          }
        } else {
          if(wd->command & 0x10) { // Seek

          } else { // Restore
            wd->track=0xff;
            wd->data=0;
            wd->led=0;
          }

          // printf("Seeking disk %d to track %d (disk in track: %d)\n",wd->diskS,wd->data,wd->disk[wd->diskS]->t);
          wd->dsr=wd->data;
          wd->state=kRVMWD177XTypeICheck;
        }
      }

      _do(wd);
      return;
    }

    case kRVMWD177XTypeICheck: {
      if(wd->track==wd->dsr) {
        wd->state=kRVMWD177XTypeIEnd;
      } else {
        if(wd->dsr>wd->track) {
          wd->control|=kRVMWD177XDire;
        } else {
          wd->control&=~kRVMWD177XDire;
        }
        wd->state=kRVMWD177XTypeIUpdate;
      }
      _do(wd);
      return;
    }

    case kRVMWD177XTypeIUpdate: {

      if(wd->control & kRVMWD177XDire) wd->track++;
      else wd->track--;

      //printf("UPDATE TRACK %d\n",wd->track);
      wd->state=kRVMWD177XTypeISeek;
      _do(wd);
      return;
    }

    case kRVMWD177XTypeISeek: {

      if(!(wd->control & kRVMWD177XDire) && (wd->disk[wd->diskS]->s & kRVMwdDiskOutTrack0)) {

        // printf("No seek track 0 end\n");
        wd->track=0;
        wd->state=kRVMWD177XTypeIEnd;
        _do(wd);
        return;

      } else {

        // printf("Seek track: %d side: %d\n", wd->track, wd->side);

        rvmwdDiskStep(wd, wd->control & kRVMWD177XDire ? 0x100 : 0x300);

        wd->led = 1; // RVM plays seek audio sample here
        wd->fdd_clicks++;

        wd->c=(srate[(wd->control & kRVMWD177XRateSelect) ^ 0x4][wd->command & 0x3]) >> 3; // Value for 1 bit per diskstep / 8

        // printf("wd->c: %d, RateSelect: %d\n",wd->c,(wd->control & kRVMWD177XRateSelect) ^ 0x4);
        //printf("RATE: %llu\n",wd->c);
        //printf("STEP\n");

        wd->stepState=kRVMWD177XStepWaiting;
        if(!(wd->command & 0xe0)) //Seek or restore
          wd->state=kRVMWD177XTypeICheck;
        else
          wd->state=kRVMWD177XTypeIEnd;

        return;

      }

    }

    case kRVMWD177XTypeIEnd: {
      if(wd->command & kRVMWD177XVerifyBit) {
        //printf("Verify\n");
        if(wd->control & kRVMWD177XHLD) {
          wd->next=kRVMWD177XTypeIHeadSet;
          wd->state=kRVMWD177XSettingHeader;
        } else {
          wd->retry=5; //5 retrys
          wd->state=kRVMWD177XReadHeader;
          wd->next=kRVMWD177XTypeIHeaderReaded;
        }

        _do(wd);
        return;
      } else {
        wd->control|=kRVMWD177XINTRQ;
        wd->status|=kRVMWD177XStatusSetHead;
        wd->status&=~kRVMWD177XStatusBusy;
        wd->stepState=kRVMWD177XStepIdle;
      }
      return;
    }

    case kRVMWD177XTypeIHeadSet: {
      wd->retry=5; //5 retrys
      wd->state=kRVMWD177XReadHeader;
      wd->next=kRVMWD177XTypeIHeaderReaded;
      _do(wd);
      return;
    }

    case kRVMWD177XReadHeader: {

      if(!wd->retry) {
        wd->status|=kRVMWD177XStatusSeek;
        _end(wd);
        return;
      }

      // _waitMark(wd);
      // printf("Waitmark ReadHeader\n");
      wd->stepState=kRVMWD177XStepWaitingMark;
      wd->marka = mark;

      wd->headerI=0xff;
      wd->state=kRVMWD177XReadHeaderBytes;
      return;

    }

    case kRVMWD177XReadAddressWait: {

      if(!wd->retry) {
        wd->status|=kRVMWD177XStatusSeek;
        _end(wd);
        return;
      }

      //_waitMark(wd);
      // printf("Waitmark ReadAddressWait\n");
      wd->stepState=kRVMWD177XStepWaitingMark;
      wd->marka = mark;
      //

      wd->headerI=0xff;
      wd->state=kRVMWD177XReadAddressDataFlag;
      return;

    }

    case kRVMWD177XReadAddressDataFlag: {
      wd->stepState=kRVMWD177XStepReadByte;
      wd->state=kRVMWD177XReadAddressBytes;
      return;
    }

    case kRVMWD177XReadAddressBytes: {

      if(!wd->retry) {
        wd->status|=kRVMWD177XStatusSeek;
        _end(wd);
        return;
      }

      wd->led = 1;

      if(wd->headerI==0xff) {
        if(wd->a!=0xfe) {
          wd->state=kRVMWD177XReadAddressWait;
          _do(wd);
          return;
        }
      } else {
        //printf("Data %02x crc:%04x\n",wd->a,wd->crc);
        // if(wd->headerI == 0) {
        //   wd->header[0]=1;
        //   wd->data=1;
        // } else {
          if (wd->headerI<7)
            wd->header[wd->headerI]=wd->a;
          wd->data=wd->a;
        // }
        if(wd->control & kRVMWD177XDRQ) {
          wd->status|=kRVMWD177XStatusLostData;
        }
        wd->control|=kRVMWD177XDRQ;
      }

      wd->headerI++;

      if (wd->headerI==0x6) {

        wd->sector = wd->header[0];

        // printf("Read Adress sector: %d\n",wd->header[0]);

        //printf("Header crc: %04x\n",wd->crc);

        // if(wd->crc) {
        // wd->status|=kRVMWD177XStatusCRC;
        // }

        _end(wd);

        return;

      }

      return;
    }

    case kRVMWD177XReadHeaderBytes: {

      if(!wd->retry) {
        wd->status|=kRVMWD177XStatusSeek;
        _end(wd);
        return;
      }

      if(wd->headerI!=0xff) {
        if (wd->headerI<7)
          wd->header[wd->headerI]=wd->a;
      }

      wd->headerI++;

      if(wd->headerI==0x7) {
        wd->state=wd->next;
        _do(wd);
        return;
      }

      wd->stepState=kRVMWD177XStepReadByte;
      return;
    }

    case kRVMWD177XTypeIHeaderReaded: {
      if(!wd->retry) {
        wd->status|=kRVMWD177XStatusSeek;
        _end(wd);
        return;
      }

      if(wd->header[0]!=0xfe) {
        wd->state=kRVMWD177XReadHeader;
        _do(wd);
        return;
      }

      if(wd->header[1]!=wd->track) {
        wd->state=kRVMWD177XReadHeader;
        _do(wd);
        return;
      }

      //printf("Header readed, track:%d sector:%d\n",wd->header[1],wd->header[3]);

      // if(wd->crc) {
      //   wd->status|=kRVMWD177XStatusCRC;
      // } else {
        wd->status&=~kRVMWD177XStatusCRC;
        _end(wd);
        return;
      // }

    }

    case kRVMWD177XTypeIISetHead: {
      wd->next=kRVMWD177XTypeIICommand;
      wd->state=kRVMWD177XSettingHeader;
      _do(wd);
      return;
    }

    case kRVMWD177XTypeIICommand: {
#if !PICO_RP2040
      if (wd->disk[wd->diskS] && (wd->disk[wd->diskS]->IsFDIFile || wd->disk[wd->diskS]->IsMBDFile))
          wd->fdiTstates = 0;
#endif

      if((wd->command & 0xc0)==0x80) { // Read or Write Sector

        if(wd->command & 0x20) { // Write Sector
          // Convert SCL to TRD on first write attempt
          if(wd->disk[wd->diskS]->IsSCLFile && !wd->disk[wd->diskS]->writeprotect)
            sclConvertToTRD(wd);
          if(wd->disk[wd->diskS]->writeprotect) {
            wd->status|=kRVMWD177XStatusProtected;
            OSD::osdCenteredMsg(OSD_DSK_WRITE_PROTECT[Config::lang], LEVEL_WARN);
            _end(wd);
            return;
          }
        } else { // Read Sector
        }
        wd->retry=5; //5 retrys

        // wd->state=kRVMWD177XReadHeader;
        // wd->next=kRVMWD177XReadSectorHeader;
        // _do(wd);

        // _waitMark(wd);
        // printf("Waitmark ReadHeader\n");
        wd->stepState=kRVMWD177XStepWaitingMark;
        wd->marka = mark;
        wd->headerI=0xff;
        wd->state=kRVMWD177XReadHeaderBytes;
        wd->next=kRVMWD177XReadSectorHeader;

      } else if((wd->command & 0xf0)==0xc0) { // Read Address

        wd->retry=5; //5 retrys
        wd->state=kRVMWD177XReadAddressWait;
        // printf("State -> ReadAddressWait\n");
        _do(wd);

      } else if((wd->command & 0xf0)==0xf0) { // Write Track
        // Convert SCL to TRD on first write attempt
        if(wd->disk[wd->diskS]->IsSCLFile && !wd->disk[wd->diskS]->writeprotect)
          sclConvertToTRD(wd);
        if(wd->disk[wd->diskS]->writeprotect) {
          wd->status|=kRVMWD177XStatusProtected;
          OSD::osdCenteredMsg(OSD_DSK_WRITE_PROTECT[Config::lang], LEVEL_WARN);
          _end(wd);
          return;
        }

        wd->state=kRVMWD177XWriteTrackStart;
        wd->stepState=kRVMWD177XStepWaitIndex;
        wd->control|=kRVMWD177XDRQ;
        wd->wtrackmark=0;
        //_do(wd);

      } else if((wd->command & 0xf0)==0xe0) { // Read Track

        wd->state=kRVMWD177XReadTrackStart;
        wd->stepState=kRVMWD177XStepWaitIndex;

      }

      return;

    }

    case kRVMWD177XReadSectorHeader: {

      if(wd->header[0]!=0xfe) {
        wd->state=kRVMWD177XReadHeader;
        _do(wd);
        return;
      }

      //   printf("--------------------\n");
      //   printf(" Header track, sector and side: %d, %d, %d\n",wd->header[1],wd->header[3],wd->header[2]);
      //   printf("Desired track, sector and side: %d, %d, %d\n",wd->track,wd->sector,wd->side);
      //   printf("Disk index: %d\n",wd->disk[wd->diskS]->indx);
      //   for (int i = 0; i < 7; i++) {
      //     printf("Header[%d]: %02x\n",i,wd->header[i]);
      //   }
      //   printf("--------------------\n");

      if(wd->header[1]!=wd->track) {
        wd->state=kRVMWD177XReadHeader;
        _do(wd);
        return;
      }

      if (!wd->fastmode || (wd->command & 0x20) || wd->sector < 1 || wd->sector > 16) {

        if(wd->header[3]!=wd->sector) {
          wd->state=kRVMWD177XReadHeader;
          _do(wd);
          return;
        }

        // Side compare: reject if header side != command side (WD1793 spec)
        if((wd->command & 0x2) && ((wd->header[2] & 1) != ((wd->command>>3) & 1))) {
          wd->state=kRVMWD177XReadHeader;
          _do(wd);
          return;
        }

      } else {

          // Side compare: reject if header side != command side (WD1793 spec)
          if((wd->command & 0x2) && ((wd->header[2] & 1) != ((wd->command>>3) & 1))) {
            wd->state=kRVMWD177XReadHeader;
            _do(wd);
            return;
          }

          wd->header[3] = wd->sector;
          wd->disk[wd->diskS]->indx = sectdatapos[wd->sector - 1] + 39; //5;

      }

      // if(wd->crc) {

      //   printf("ReadSectorHeader CRC %08X\n",wd->crc);

      //   wd->status|=kRVMWD177XStatusCRC;
      //   wd->state=kRVMWD177XReadHeader;
      //   _do(wd);
      //   return;

      // } else {

        wd->status &= ~kRVMWD177XStatusCRC;

        // Sector size from address mark: 0=128, 1=256, 2=512, 3=1024
        // Use real size for UDI/FDI/MBD, hardcode 256 for TRD/SCL (Betadisk standard)
        {
            uint32_t sz = 128 << (wd->header[4] & 0x03);
#if !PICO_RP2040
            if (!wd->disk[wd->diskS]->IsUDIFile && !wd->disk[wd->diskS]->IsFDIFile && !wd->disk[wd->diskS]->IsMBDFile)
#endif
                sz = 0x100;
            wd->c = sz;
        }

        // _waitMark(wd);
        // printf("Waitmark ReadSectorHeader\n");
        wd->stepState=kRVMWD177XStepWaitingMark;
        wd->marka = mark;
        //
        if(wd->command & 0x20) {

          //printf("Writing Command: %02x Track:%d Sector:%d Size:%llu\n",wd->command,wd->track,wd->sector,wd->c);

          wd->state=kRVMWD177XWriteDataFlag;
          wd->control|=kRVMWD177XDRQ;

        } else {

          wd->state=kRVMWD177XReadDataFlag;

        }

        return;

      // }

    }

    case kRVMWD177XReadDataFlag: {
      wd->stepState=kRVMWD177XStepReadByte;
      wd->state=kRVMWD177XReadDataFlag2;
      return;
    }

    case kRVMWD177XWriteDataFlag: {
      wd->stepState=kRVMWD177XStepWriteByte;
      wd->state=kRVMWD177XWriteData;
      wd->control|=kRVMWD177XWriting;
      wd->a=(wd->command & 0x1)?0xf8:0xfb;
      // wd->crc=crc(wd->crc,wd->a);
      return;
    }

case kRVMWD177XWriteData: {

      // Verificar underrun ANTES de procesar el byte
      if(wd->control & kRVMWD177XDRQ) {
        //printf("Lost data in write - aborting command\n");
        wd->status|=kRVMWD177XStatusLostData;
        wd->control&=~kRVMWD177XWriting;
        _end(wd);
        return;
      }

      wd->led = 2;

      wd->a=wd->data;
      // wd->crc=crc(wd->crc,wd->a);
      //printf("Write %d data: %02x CRC: %04x\n",wd->c,wd->a,wd->crc);
      wd->data=0;

      if(--wd->c) {
        wd->control|=kRVMWD177XDRQ;
      } else {
        wd->state=kRVMWD177XWriteCRC1;
        //_do(wd);
      }
      return;
    }

    case kRVMWD177XWriteCRC1: {
      // wd->a=wd->crc>>8;
      //printf("Write CRC byte: %02x CRC: %04x\n",wd->a,wd->crc);
      wd->state=kRVMWD177XWriteCRC2;
      return;
    }

    case kRVMWD177XWriteCRC2: {
      // wd->a=wd->crc & 0xff;
      //printf("Write CRC byte: %02x CRC: %04x\n",wd->a,wd->crc);
      wd->state=kRVMWD177XWriteLast;

      return;
    }

    case kRVMWD177XWriteLast: {
      wd->state=kRVMWD177XWriteEnd;
      wd->stepState=kRVMWD177XStepLastWriteByte;
      return;
    }

    case kRVMWD177XWriteEnd: {

      wd->control&=~kRVMWD177XWriting;

#if !PICO_RP2040
      // On real WD1793, writing a sector produces valid CRC. Fix the MFM buffer
      // and cached flags so subsequent reads on this track return correct CRC.
      if (wd->disk[wd->diskS] && (wd->disk[wd->diskS]->IsFDIFile || wd->disk[wd->diskS]->IsMBDFile) && wd->diskDirty) {
          for (int n = 0; n < wd->fdiSectorCount; n++) {
              uint32_t idPos = wd->fdiSectorIdPos[n];
              if (idPos + 5 < (uint32_t)wd->diskTrackLen &&
                  wd->diskTrackBuf[idPos + 1] == wd->header[1] &&
                  wd->diskTrackBuf[idPos + 2] == wd->header[2] &&
                  wd->diskTrackBuf[idPos + 3] == wd->header[3] &&
                  wd->diskTrackBuf[idPos + 4] == wd->header[4])
              {
                  wd->fdiSectorFlags[n] &= ~1;
                  int bufLen = wd->diskTrackLen;
                  for (int i = idPos + 7; i < (int)idPos + 87 && i < bufLen; i++) {
                      if (wd->diskTrackBuf[i] == 0xFB || wd->diskTrackBuf[i] == 0xF8) {
                          int slen = 128 << (wd->header[4] & 3);
                          int crcPos = i + 1 + slen;
                          if (crcPos + 2 <= bufLen) {
                              int crcStart = i;
                              while (crcStart > 0 && wd->diskTrackBuf[crcStart - 1] == 0xA1) crcStart--;
                              uint16_t crc = 0xFFFF;
                              for (int j = crcStart; j < crcPos; j++)
                                  crc = vgCrc(crc, wd->diskTrackBuf[j]);
                              wd->diskTrackBuf[crcPos] = (uint8_t)(crc >> 8);
                              wd->diskTrackBuf[crcPos + 1] = (uint8_t)(crc & 0xFF);
                          }
                          break;
                      }
                  }
                  break;
              }
          }
      }
#endif

      // Write buffer to diskfile
      // int saveptr = ftell(wd->disk[wd->diskS]->Diskfile);
      // int seekptr = (wd->track << (11 + wd->disk[wd->diskS]->sides)) + (wd->side ? 4096 : 0) + ((wd->sector - 1) << 8);
      // fseek(wd->disk[wd->diskS]->Diskfile,seekptr,SEEK_SET);
      // fwrite(wd->disk[wd->diskS]->cursectbuf,1,0x100, wd->disk[wd->diskS]->Diskfile);
      // fseek(wd->disk[wd->diskS]->Diskfile,saveptr,SEEK_SET);

      // printf("Track:%d, Side:%d, Sector: %d\n",wd->track,wd->side,wd->sector);
      // for (int i=0; i< 0x100; i+= 0x10) {
      //   printf("Pos %04x: ",i);
      //   for (int n=0; n< 0x10; n++) {
      //     printf("%02x ",wd->disk[wd->diskS]->cursectbuf[i + n]);
      //   }
      //   printf("\n");
      // }
      // printf("==================================\n");

      if(wd->command & 0x10) { // Write sector: Multiple record flag on
        wd->sector++;
        wd->state=kRVMWD177XTypeIICommand;
        _do(wd);
      } else { // Write sector: Multiple record flag off
        _end(wd);
      }
      return;
    }

    case kRVMWD177XReadDataFlag2: {
      if(wd->a==0xf8) {
        wd->status|=kRVMWD177XStatusRecordType;
      } else if(wd->a==0xfb) {
        wd->status&=~kRVMWD177XStatusRecordType;
      } else {
#if !PICO_RP2040
        // FDI: data mark not found after ID match (sector has no data area).
        // Go to ReadHeader (preserves retry) and force index pulse so retry
        // decrements — prevents infinite loop from TypeIICommand resetting retry=5.
        if (wd->disk[wd->diskS] && (wd->disk[wd->diskS]->IsFDIFile || wd->disk[wd->diskS]->IsMBDFile)) {
            wd->disk[wd->diskS]->indx = wd->diskTrackLen; // trigger index pulse
            wd->state=kRVMWD177XReadHeader;
            wd->next=kRVMWD177XReadSectorHeader;
            _do(wd);
            return;
        }
#endif
        wd->state=kRVMWD177XTypeIICommand;
        _do(wd);
        wd->stepState=kRVMWD177XNone;
        return;
      }
      wd->state=kRVMWD177XReadData;
      return;
    }

    case kRVMWD177XReadData: {

      wd->led = 1;

      wd->data = wd->a;

      //printf("Read %d byte: %02x CRC: %04x\n",wd->c,wd->a,wd->crc);
      if(wd->control & kRVMWD177XDRQ) {
        //printf("Lost data in read\n");
        wd->status|=kRVMWD177XStatusLostData;
      }

      wd->control|=kRVMWD177XDRQ;
      if(!--wd->c) {
        wd->state=kRVMWD177XReadCRC;
        wd->c=2; // 2 bytes CRC
        return;
      }
      return;
    }

    case kRVMWD177XReadCRC: {
      // printf("Read CRC byte: %02x CRC: %04x\n",wd->a,wd->crc);
      if(!--wd->c) { // CRC readed

#if !PICO_RP2040
        if (wd->disk[wd->diskS] && (wd->disk[wd->diskS]->IsFDIFile || wd->disk[wd->diskS]->IsMBDFile) && wd->fdiDataCrcError) {
          wd->status |= kRVMWD177XStatusCRC;
          wd->fdiDataCrcError = false;
        } else
#endif
          wd->status&=~kRVMWD177XStatusCRC;

        if(wd->command & 0x10) { // Read sector: Multiple record flag on

          wd->sector++; // Next sector

          if (!wd->fastmode || wd->sector > 16) {

            // printf("Fast mode Next sector: ");
            // if (wd->sector > 16)
            //   printf("Sector >  16: %d\n",wd->sector);
            // else
            //   printf("Sector <= 16: %d\n",wd->sector);

            // printf("Read Sector, multiple: sector %d\n",wd->sector);

            wd->state=kRVMWD177XTypeIICommand;
            _do(wd);

          } else {

            wd->header[3] = wd->sector;
            wd->disk[wd->diskS]->indx = sectdatapos[wd->sector - 1] + 39; // + 5;

            wd->status &= ~kRVMWD177XStatusCRC;
            wd->c = 0x100; // Esto, en Betadisk, siempre será el tamaño del sector

            // _waitMark(wd);
            wd->stepState=kRVMWD177XStepWaitingMark;
            wd->marka = mark;
            //

            wd->state=kRVMWD177XReadDataFlag;

          }

        } else { // Read sector: Multiple record flag off

          _end(wd);

        }

      }

      return;

    }

    case kRVMWD177XWriteTrackStart:{
      if(wd->control & kRVMWD177XDRQ) {
        wd->status|=kRVMWD177XStatusLostData;
        _end(wd);
        return; // Missing return
      }
      wd->control|=kRVMWD177XWriting;
      wd->state=kRVMWD177XWriteTrack;

      wd->disk[wd->diskS]->indx = 0xffffffff;
      wd->disk[wd->diskS]->cursectbufpos = 0xff;
      wd->disk[wd->diskS]->indexDelay = 0;

      _do(wd);
    }

case kRVMWD177XWriteTrack: {

      if(!wd->retry) {
        _end(wd);
        return;
      }

      // Verificar underrun ANTES de procesar
      if(wd->control & kRVMWD177XDRQ) {
        printf("Lost data in write track - aborting command\n");
        wd->status|=kRVMWD177XStatusLostData;
        wd->control&=~kRVMWD177XWriting;
        _end(wd);
        return;
      }

      wd->led = 2;

        switch(wd->data) {

          case 0xf5: {
            wd->wtrackmark++;

            wd->a=sectorMark;
            wd->stepState=kRVMWD177XStepWriteByte;
            wd->control|=kRVMWD177XDRQ;
            // printf("kRVMWD177XWriteTrack -> Sector Mark!! ->");
            break;
            // return;
          }

          case 0xf7: {

            wd->wtrackmark=0;

            // wd->a=wd->crc>>8;
            wd->a=0;
            wd->stepState=kRVMWD177XStepWriteByte;
            wd->state=kRVMWD177XWriteTrackCRC;
            // printf("kRVMWD177XWriteTrack -> CRC!! ->");
            break;
            // return;
          }

          default: {
            if (wd->wtrackmark == 3 && wd->data == 0xfe) {
              // printf("Write Sector Header, track %d, side %d\n",wd->track,wd->side);
              wd->wtrackmark = 0b100000000;
            } else if (wd->wtrackmark == 3 && wd->data == 0xfb) {
              // printf("Write Sector Data at sector %d\n",wd->wtracksector);
              // wd->wtrackmark=0b1000000000;
              // For raw format disks (UDI/FDI/MBD), indx runs sequentially through the track buffer;
              // sectdatapos repositioning is only valid for TRD's fixed sector layout.
#if !PICO_RP2040
              if (!wd->disk[wd->diskS]->IsUDIFile && !wd->disk[wd->diskS]->IsFDIFile && !wd->disk[wd->diskS]->IsMBDFile)
#endif
              wd->disk[wd->diskS]->indx = sectdatapos[wd->wtracksector - 1] + 41;
            } else if (wd->wtrackmark & 0b100000000) {
              wd->wtrackmark++;
              // printf("   Write Sector Header, byte: %02x\n",wd->data);
              if (wd->wtrackmark == 0b100000001) {
                // printf("Write track to track0 side1 sector header!\n");
                if (wd->track == 0 && wd->side == 1) wd->disk[wd->diskS]->t0s1_info = wd->data;
              } else
              if (wd->wtrackmark == 0b100000011) {
                wd->wtracksector = wd->data;
                wd->wtrackmark = 0;
              }
            } else {
              wd->wtrackmark=0;
            }

            wd->a = wd->data;
            // wd->crc=crc(wd->crc,wd->a);
            // printf("Format byte: %02x\n",wd->a);
            wd->stepState=kRVMWD177XStepWriteByte;
            wd->control|=kRVMWD177XDRQ;
            break;
            // return;
          }
        }

      return;

    }

    case kRVMWD177XWriteTrackCRC: {
      // wd->a=wd->crc & 0xff;
      wd->a=0x0;
      wd->stepState=kRVMWD177XStepWriteByte;
      wd->state=kRVMWD177XWriteTrack;
      wd->control|=kRVMWD177XDRQ;

      // wd->disk[wd->diskS]->indx -= 4;
      // wd->disk[wd->diskS]->cursectbufpos = 0xffff;

      // printf("kRVMWD177XWriteTrack -> CRC!! ->");
      // printf("Format byte: %02x\n",wd->a);
      return;
    }

    case kRVMWD177XReadTrackStart: {
      wd->stepState=kRVMWD177XStepReadByte;
      wd->state=kRVMWD177XReadTrackData;
      wd->retry=1;
      // printf("ReadTrackStart!\n");
      return;
    }

    case kRVMWD177XReadTrackData: {

      // printf("ReadTrackData!\n");

      if(!wd->retry) {
        _end(wd);
        return;
      }

      wd->led = 1;

      if(wd->control & kRVMWD177XDRQ) wd->status|=kRVMWD177XStatusLostData;

      wd->control|=kRVMWD177XDRQ;
      wd->data=wd->a;
      return;
    }
  }
}

IRAM_ATTR void rvmWD1793Step(rvmWD1793 *wd, uint32_t steps) {
  
  for (;steps > 0; steps--) {

    uint8_t d=0x0;
    uint8_t s=0x0;
    uint8_t dd=0x0;

    if(wd->disk[wd->diskS]) { // If active disk exists ..

      uint8_t t;
      uint16_t w = 0;

      if((wd->control & kRVMWD177XWriting) && !wd->disk[wd->diskS]->writeprotect) {
        w |= kRVMwdDiskControlWrite | wd->wb;
      }

      t = rvmwdDiskStep(wd, w);

      d = t;
      dd = wd->disk[wd->diskS]->a;
      s = wd->disk[wd->diskS]->s;

    }

    uint8_t pd = s ^ wd->diskP;
    wd->diskP=s;

    // Force Interrupt condition check: bit 2 = index pulse
    if ((wd->control >> 16) & 0x4) {
      if ((pd & kRVMwdDiskOutIndex) && (s & kRVMwdDiskOutIndex)) {
        wd->control &= 0xffff; // clear conditions
        wd->control |= kRVMWD177XINTRQ;
      }
    }

    // printf("wd->stepState: %d\n",wd->stepState);

    switch(wd->stepState) {

      case kRVMWD177XStepIdle:{

        wd->led = 0;

        if(wd->retry && (pd & kRVMwdDiskOutIndex) && (s & kRVMwdDiskOutIndex)) {
          wd->retry--;
          if(!wd->retry) {
            wd->control&=~(kRVMWD177XHLD | kRVMWD177XHLT);
            return;
          }
        }
        break;
      }

      case kRVMWD177XStepWaiting: {

        if ((wd->track == 0 && wd->sector == 0) || wd->track == 0xff ) {
          wd->led = 0;
          wd->fdd_clicks = 0;
        }

        if (wd->fastmode) {
          wd->c = 0;
          _do(wd);
        } else if(!(--wd->c)) {
            _do(wd);
        }

        break;

      }

      case kRVMWD177XStepWaitingMark: {

        // _checkIndex(wd,pd,s);
        if(wd->retry && (pd & kRVMwdDiskOutIndex) && (s & kRVMwdDiskOutIndex)) {
          wd->retry--;
          if(!wd->retry) _do(wd);
        }
        // end _checkIndex

#if !PICO_RP2040
        // FDI/MBD: empty track (0 sectors) — report Record Not Found immediately
        // instead of spinning for 5 full revolutions (~5 seconds).
        if (wd->disk[wd->diskS] && (wd->disk[wd->diskS]->IsFDIFile || wd->disk[wd->diskS]->IsMBDFile)
            && wd->state == kRVMWD177XReadHeaderBytes
            && wd->fdiSectorCount == 0)
        {
            wd->status |= kRVMWD177XStatusSeek; // bit 4 = Record Not Found (Type II)
            _end(wd);
            break;
        }

        // FDI/MBD find_marker: find nearest sector header ahead of current disk->indx.
        // Uses actual MFM buffer position (not CPU T-states) for compatibility
        // with the incremental indx++ model used in rvmwdDiskStep.
        if (wd->disk[wd->diskS] && (wd->disk[wd->diskS]->IsFDIFile || wd->disk[wd->diskS]->IsMBDFile)
            && (wd->state == kRVMWD177XReadHeaderBytes
                || wd->state == kRVMWD177XReadAddressDataFlag)
            && wd->marka == mark
            && wd->fdiSectorCount > 0)
        {
            rvmwdDisk *fdisk = wd->disk[wd->diskS];
            // Don't intercept while index pulse is pending (indx >= trkLen or 0xffffffff).
            // Let the normal step mechanism deliver the index pulse so retry decrements.
            if (fdisk->indx >= (uint32_t)wd->diskTrackLen)
                break;
            uint32_t trkLen = wd->diskTrackLen;
            if (trkLen > 0) {
                // Use current disk->indx as position in MFM buffer
                uint32_t curPos = (fdisk->indx < trkLen) ? fdisk->indx : 0;

                // Find nearest sector ahead (circular distance).
                // For Read/Write Sector (ReadHeaderBytes), skip sectors with no data area
                // (flags & 0x40) — they have no data mark, causing infinite retry loops.
                bool skipNoData = (wd->state == kRVMWD177XReadHeaderBytes);
                uint32_t bestDist = 0xFFFFFFFF;
                int bestSec = -1;
                for (int n = 0; n < wd->fdiSectorCount; n++) {
                    if (skipNoData && (wd->fdiSectorFlags[n] & 2)) continue;
                    uint32_t idPos = wd->fdiSectorIdPos[n];
                    uint32_t dist = (idPos > curPos) ? idPos - curPos
                                                     : trkLen + idPos - curPos;
                    if (dist < bestDist) { bestDist = dist; bestSec = n; }
                }
                // If all sectors have no data, trigger index pulse
                if (bestSec < 0) {
                    fdisk->indx = trkLen;
                    break;
                }

                uint32_t idPos = wd->fdiSectorIdPos[bestSec];

                // If the nearest sector is behind us (wrapped around track end),
                // we've crossed the index hole. Set indx past end of track so that
                // rvmwdDiskStep generates an index pulse (indexDelay), which will
                // naturally decrement retry via _checkIndex on the next step.
                if (idPos < curPos) {
                    fdisk->indx = trkLen; // trigger index pulse via rvmwdDiskStep
                    break; // exit — let normal stepping handle the revolution
                }

                // Populate header[] from MFM buffer (FE, C, H, R, N, CRC1, CRC2)
                for (int i = 0; i < 7 && (idPos + i) < trkLen; i++)
                    wd->header[i] = wd->diskTrackBuf[idPos + i];
                fdisk->indx = idPos + 7; // position past header
                wd->fdiDataCrcError = (wd->fdiSectorFlags[bestSec] & 1);

                if (wd->state == kRVMWD177XReadHeaderBytes) {
                    wd->state = wd->next; // → kRVMWD177XReadSectorHeader
                    _do(wd);
                } else {
                    // Read Address: position at FE for sequential read
                    fdisk->indx = idPos - 1;
                    wd->state = kRVMWD177XReadAddressDataFlag;
                    _do(wd);
                }
                break;
            }
        }
#endif

        if((wd->marka & 0xff) == dd) {
          wd->marka >>= 8;
          if(!wd->marka) {
            _do(wd);
            if(wd->control & kRVMWD177XWriting) goto write;
          }
        } else {
          wd->marka=mark;
        }

        break;

      }

      case kRVMWD177XStepReadByte: {

        // _checkIndex(wd,pd,s);
        if(wd->retry && (pd & kRVMwdDiskOutIndex) && (s & kRVMwdDiskOutIndex)) {
          wd->retry--;
          if(!wd->retry) _do(wd);
        }
        // end _checkIndex

        wd->a = dd;
        _do(wd);
        wd->a = 0;

        break;

      }

      case kRVMWD177XStepWriteByte: {

  write:

        // _checkIndex(wd,pd,s);
        if(wd->retry && (pd & kRVMwdDiskOutIndex) && (s & kRVMwdDiskOutIndex)) {
          wd->retry--;
          if(!wd->retry) _do(wd);
        }
        // end _checkIndex

        wd->wb = wd->a;
        _do(wd);

        break;

      }

      case kRVMWD177XStepWriteRaw: {

        // _checkIndex(wd,pd,s);
        if(wd->retry && (pd & kRVMwdDiskOutIndex) && (s & kRVMwdDiskOutIndex)) {
          wd->retry--;
          if(!wd->retry) _do(wd);
        }
        // end _checkIndex

        wd->wb = wd->a;
        _do(wd);

        break;
      }

      case kRVMWD177XStepLastWriteByte:
        _do(wd);
        break;

      case kRVMWD177XStepWaitIndex: {
        if((pd & kRVMwdDiskOutIndex) && (s & kRVMwdDiskOutIndex)) {
          wd->retry=1;
          _do(wd);
        }
        break;
      }
    }

  }

}

IRAM_ATTR void rvmWD1793Write(rvmWD1793 *wd,uint8_t a,uint8_t value) {
  switch(a & 0x3) {

    case 0: //Command
      // // --- WD1793 TR-DOS bug workaround ---
      // // If the command is in the write range (0xB8-0xBF, 0xF8-0xFF) and not in TR-DOS write mode, ignore!
      // if ((value & 0xF8) == 0xB8 || (value & 0xF8) == 0xF8) {
      //   // This is a garbage "write" command (write sector/track)
      //   // Do NOT perform any write operations, do not damage the disk!
      //   wd->status &= ~kRVMWD177XStatusBusy;
      //   wd->control |= kRVMWD177XINTRQ;
      //   return;
      // }
      // // --- end of patch ---

      if ((value & 0xf0) == 0xd0) {
        //Force interrupt
        if(wd->status & kRVMWD177XStatusBusy) {
            wd->status &= ~kRVMWD177XStatusBusy;
          wd->state = kRVMWD177XNone;
          wd->stepState = kRVMWD177XStepIdle;
          wd->control &= ~(kRVMWD177XWriting|kRVMWD177XDRQ);
          wd->retry = 15;
          wd->led = 0;

        } else {

          wd->status=kRVMWD177XStatusSetIndex | kRVMWD177XStatusSetTrack0 | kRVMWD177XStatusSetWP;
          // WD2797: preserve Head Loaded state and sync disk_t to Track register
          if (wd->wd2797_mode) {
            wd->status |= kRVMWD177XStatusSetHead;
            if (wd->disk[wd->diskS]) {
              wd->disk[wd->diskS]->t = wd->track;
              // Update Track 0 output signal
              wd->disk[wd->diskS]->s = wd->track ? 0 : kRVMwdDiskOutTrack0;
            }
          }

        }

        if((value & 0xf)==0x0) {

          wd->control&=~(kRVMWD177XINTRQ|kRVMWD177XFINTRQ);

        } else {

          if(value & 0x8) {

            //Inmediate interrupt
            wd->control|=kRVMWD177XFINTRQ;

          } else {

            wd->control = (wd->control & 0xffff) | ((value & 0xf) << 16); //Set conditions

          }

        }

        return;

      }

      if(!(wd->status & kRVMWD177XStatusBusy)) {

        wd->control &= ~(kRVMWD177XINTRQ|kRVMWD177XFINTRQ);

        wd->command = value;


        if(wd->disk[wd->diskS]  && (wd->control & (kRVMWD177XPower0 << wd->diskS))) {

          //Issue command
          if(wd->command & kRVMWD177XTypeI) {


            //Type II, III, IV kRVMWD177XTypeIISetHead
            if((wd->command & 0xc0)==0x80 || (wd->command & 0xfb)==0xc0 || (wd->command & 0xfb)==0xf0 || (wd->command & 0xfb)==0xe0) {

              //TYPE
              wd->state=kRVMWD177XTypeIISetHead;
              wd->status=kRVMWD177XStatusBusy;

              // WD2797: side select from command bit 1 (SSO — Side Select Output)
              if (wd->wd2797_mode) {
                wd->side = (wd->command >> 1) & 1;
              }

              if(wd->command & kRVMWD177XVerifyBit) {

                wd->stepState = kRVMWD177XStepWaiting;

                // if (wd->fastmode)
                //   wd->c = 1;
                // else
                  wd->c = 937; // 7500 (Value for 1 bit per diskstep) / 8
                // printf("Waiting 30ms\n");

              } else {

                _do(wd);

              }

            } else {
              // Force Interrupt (0xD0) or unrecognized Type IV
              wd->control |= kRVMWD177XINTRQ;
            }

          } else {

            //Type I
            // Clear DRQ (mirrors UnrealSpeccy S_TYPE1_CMD: status &= ~WDS_DRQ, rqs=0)
            wd->control &= ~kRVMWD177XDRQ;
            wd->status = kRVMWD177XStatusSetIndex | kRVMWD177XStatusSetTrack0 | kRVMWD177XStatusSetWP | kRVMWD177XStatusBusy;
            wd->state = kRVMWD177XTypeI0;

            _do(wd);

            // Complete Type I (Seek/Step/Restore) step delays immediately.
            // Copy-protected loaders issue the next command before step delays expire.
            // WD2797 (MB-02) Seek (0x10-0x1F): don't complete instantly — BS-DOS
            // calibration checks Busy flag timing to determine step rate.
            // Restore (0x00-0x0F) still completes instantly (255 steps would hang).
            if (!(wd->command & kRVMWD177XTypeI)) {
              while (wd->stepState == kRVMWD177XStepWaiting) {
                _do(wd);
              }
            }

            // If Type I completed synchronously (TypeIEnd reached inside _do chain),
            // INTRQ was set before TR-DOS ROM enters its polling loop on port 0xFF.
            // TR-DOS reads the status register first (clearing INTRQ), then polls
            // the system register — missing the INTRQ and hanging forever.
            // Fix: promote INTRQ to FINTRQ, which is not cleared by status reads
            // but is still visible on the system register (port 0xFF bit 7).
            // FINTRQ will be cleared when the next command is issued.
            if (wd->state == kRVMWD177XTypeIEnd
                && wd->stepState == kRVMWD177XStepIdle
                && (wd->control & kRVMWD177XINTRQ)) {
              wd->control |= kRVMWD177XFINTRQ;
            }

          }

        } else {

          wd->status=kRVMWD177XStatusNotReady;
          wd->control|=kRVMWD177XINTRQ;

        }

      } // else printf("BUSY!!!\n");

      break;

    case 1: //Track
      //if(!(wd->status & kRVMWD177XStatusBusy)) {
        wd->track=value;
      //}
      break;
    case 2: //Sector
      //if(!(wd->status & kRVMWD177XStatusBusy)) {
        wd->sector=value;
      //}
      break;
    case 3: //Data
      wd->data=value;
      wd->control &= ~kRVMWD177XDRQ;
      break;
  }
}

IRAM_ATTR uint8_t rvmWD1793Read(rvmWD1793 *wd,uint8_t a) {

  uint8_t r;

  switch(a & 0x3) {
    case 0: //Status
      wd->control&=~kRVMWD177XINTRQ;

      r=wd->status & 0xff;

      if(wd->disk[wd->diskS]) {
        if(wd->status & kRVMWD177XStatusSetWP)  {
          if(wd->disk[wd->diskS]->writeprotect) {
            r|=kRVMWD177XStatusProtected;
          } else {
            r&=~kRVMWD177XStatusProtected;
          }
        }

        if((wd->status & kRVMWD177XStatusSetTrack0) && (wd->disk[wd->diskS]->s & kRVMwdDiskOutTrack0)) {
          r|=kRVMWD177XStatusTrack0;
        }

        if((wd->status & kRVMWD177XStatusSetIndex) && (wd->disk[wd->diskS]->s & kRVMwdDiskOutIndex)) {
          r|=kRVMWD177XStatusIndex;
        } else {
          if(wd->control & kRVMWD177XDRQ) r|=kRVMWD177XStatusDataRequest;
        }

        // HeadLoaded is only visible in status if HLT bit (bit 3 of system reg) is set.
        // UnrealSpeccy: status & ((system & 8) ? 0xFF : ~WDS_HEADL)
        // WD2797: bit 5 in Type I status = Spin-up complete (NOT Head Loaded)
        // Don't set it — BS-DOS may use this bit differently
        if (wd->wd2797_mode) {
          // Don't set bit 5 for WD2797 Type I status
        } else {
          if((wd->status & kRVMWD177XStatusSetHead) && (wd->control & kRVMWD177XTest)) {
            r|=kRVMWD177XStatusHeadLoaded;
          }
        }
      } else {
        r|=kRVMWD177XStatusNotReady;
      }

      return r;
    case 1: //Track
      return wd->track;
    case 2: //Sector
      return wd->sector;
    case 3: //Data

      // if(!(wd->control&kRVMWD177XDRQ)) {
      //   printf("Read data register overrunning\n");
      // }

      wd->control &= ~kRVMWD177XDRQ;

      // printf("read data: %02x\n", wd->data);
      return wd->data;

  }

  return 0;
}

static void fdiFlushTrack(rvmWD1793 *wd);
static void mbdFlushTrack(rvmWD1793 *wd);

void rvmWD1793Reset(rvmWD1793 *wd) {

  wd->state = kRVMWD177XNone;
  wd->stepState = kRVMWD177XStepIdle;
  wd->next = kRVMWD177XNone;

  wd->c = 0;
  wd->control &= 0xf000;
  wd->command = wd->dsr = 0x0;
  // Note: sector and data registers are NOT cleared by WD1793 hardware reset (MR pin)
  wd->status = kRVMWD177XStatusSetIndex | kRVMWD177XStatusSetTrack0 | kRVMWD177XStatusSetWP;
  wd->track = 0xff;
  wd->led = 0;
  wd->fdd_clicks = 0;
  wd->wtrackmark = 0;
  wd->headerI = 0;
  wd->retry = 0;
  // wd->crc=0xffff;
  wd->crc = 0; // Disable CRC. Not needed for Betadisk emulation
  wd->side = wd->diskS = 0;
  // wd2797_mode is set externally (by MB02::init), preserve across reset
  // UDI/FDI/MBD raw-format disks require fastmode=false (sectdatapos incompatible with raw MFM)
#if !PICO_RP2040
  {
    bool hasRawDisk = false;
    for (int i = 0; i < 4; i++)
      if (wd->disk[i] && (wd->disk[i]->IsUDIFile || wd->disk[i]->IsFDIFile || wd->disk[i]->IsMBDFile))
        hasRawDisk = true;
    wd->fastmode = hasRawDisk ? false : Config::trdosFastMode;
  }
#else
  wd->fastmode = Config::trdosFastMode;
#endif
  wd->sclConverted = false;
#if !PICO_RP2040
  // Flush modified UDI/FDI track to SD before resetting (avoid data loss)
  if (wd->diskDirty && wd->diskLoadedCyl >= 0) {
    rvmwdDisk *disk = wd->disk[wd->diskS];
    if (disk && disk->Diskfile) {
      if (disk->IsUDIFile) {
        int trkIdx = wd->diskLoadedCyl * disk->sides + wd->diskLoadedSide;
        if (trkIdx >= 0 && trkIdx < 168) {
          UINT bw;
          f_lseek(disk->Diskfile, disk->udiTrackOffsets[trkIdx]);
          f_write(disk->Diskfile, wd->diskTrackBuf, disk->udiTrackLengths[trkIdx], &bw);
        }
      } else if (disk->IsFDIFile) {
        fdiFlushTrack(wd);
      } else if (disk->IsMBDFile) {
        mbdFlushTrack(wd);
      }
    }
  }
  wd->diskLoadedCyl = -1;
  wd->diskLoadedSide = -1;
  wd->diskTrackLen = 0;
  wd->diskDirty = false;
  wd->fdiTstates = 0;
  wd->fdiSectorCount = 0;
  wd->fdiDataCrcError = false;
#endif
}

bool rvmWD1793InsertDisk(rvmWD1793 *wd, unsigned char UnitNum, const std::string& Filename) {

    // Close any open disk in this unit
    wdDiskEject(wd,UnitNum);

    // wd->disk[UnitNum] = (rvmwdDisk *) heap_caps_calloc(1, sizeof(rvmwdDisk), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    wd->disk[UnitNum] = (rvmwdDisk *) heap_caps_calloc(1, sizeof(rvmwdDisk), MALLOC_CAP_8BIT);

    //wd->disk[UnitNum]->Diskfile = fopen(Filename.c_str(), "r+b");
    wd->disk[UnitNum]->Diskfile = fopen2(Filename.c_str(), FA_READ | FA_WRITE);
    if (wd->disk[UnitNum]->Diskfile == NULL) {
      Debug::led_blink();
      wdDiskEject(wd,UnitNum);
      return false;
    }

    uint8_t diskType;

    char magic[8];
    UINT br;
    //fread(&magic, 1, 8, wd->disk[UnitNum]->Diskfile);
    FRESULT res = f_read(wd->disk[UnitNum]->Diskfile, &magic, 8, &br);

    if (std::strncmp(magic,"SINCLAIR",8) == 0) {
        // SCL file
        printf("SCL disk loaded\n");
        wd->disk[UnitNum]->IsSCLFile=true;
#if !PICO_RP2040
        wd->disk[UnitNum]->IsUDIFile = false;
        wd->disk[UnitNum]->IsFDIFile = false;
#endif
        wd->disk[UnitNum]->fname = Filename;
        // writeprotect is seeded by the caller from the per-slot Config array.
        wd->disk[UnitNum]->writeprotect = 0;
        wd->fastmode = Config::trdosFastMode;
        diskType = 0x16;

#if !PICO_RP2040
    } else if (std::strncmp(magic,"UDI!",4) == 0) {
        // UDI file
        printf("UDI disk loaded\n");
        wd->disk[UnitNum]->IsSCLFile = false;
        wd->disk[UnitNum]->IsUDIFile = true;
        wd->disk[UnitNum]->IsFDIFile = false;
        // writeprotect is seeded by the caller from the per-slot Config array.
        wd->disk[UnitNum]->writeprotect = 0;
        wd->disk[UnitNum]->sclDataOffset = 0;

        // Read UDI header
        uint8_t hdr[16];
        f_lseek(wd->disk[UnitNum]->Diskfile, 0);
        f_read(wd->disk[UnitNum]->Diskfile, hdr, 16, &br);

        uint8_t cyls = hdr[9] + 1;
        uint8_t sides = hdr[10] + 1;
        uint32_t extHdrLen = hdr[12] | (hdr[13] << 8) | (hdr[14] << 16) | (hdr[15] << 24);

        wd->disk[UnitNum]->tracks = cyls - 1; // tracks field stores max track index
        wd->disk[UnitNum]->sides = sides;

        // Parse track offsets and lengths
        uint32_t offset = 16 + extHdrLen;
        int totalTracks = cyls * sides;
        if (totalTracks > 168) totalTracks = 168;

        for (int i = 0; i < totalTracks; i++) {
            uint8_t trkHdr[3];
            f_lseek(wd->disk[UnitNum]->Diskfile, offset);
            f_read(wd->disk[UnitNum]->Diskfile, trkHdr, 3, &br);

            uint16_t tlen = trkHdr[1] | (trkHdr[2] << 8);
            uint16_t clen = (tlen + 7) / 8;

            wd->disk[UnitNum]->udiTrackOffsets[i] = offset + 3; // data starts after 3-byte track header
            wd->disk[UnitNum]->udiTrackLengths[i] = tlen;

            offset += 3 + tlen + clen;
        }

        // Skip the normal diskType switch — we already set tracks/sides
        wd->disk[UnitNum]->t0s1_info = 0;
        wd->disk[UnitNum]->cursectbufpos = 0xff;
        wd->control |= kRVMWD177XPower0 << UnitNum;
        wd->disk[UnitNum]->fname = Filename;
        wd->diskLoadedCyl = -1;
        wd->diskLoadedSide = -1;
        wd->fastmode = false; // fastmode uses sectdatapos, incompatible with raw MFM

        printf("UDI: %d cylinders, %d sides\n", cyls, sides);
        return true;

    } else if (std::strncmp(magic,"FDI",3) == 0) {
        // FDI file — generate MFM track images on demand (ZXMAK2 approach)
        wd->disk[UnitNum]->IsSCLFile = false;
        wd->disk[UnitNum]->IsUDIFile = false;
        wd->disk[UnitNum]->IsFDIFile = true;
        wd->disk[UnitNum]->sclDataOffset = 0;

        // Read FDI header (14 bytes)
        uint8_t hdr[14];
        f_lseek(wd->disk[UnitNum]->Diskfile, 0);
        f_read(wd->disk[UnitNum]->Diskfile, hdr, 14, &br);

        uint8_t wpFlag = hdr[3];
        uint16_t cyls = hdr[4] | (hdr[5] << 8);
        uint16_t sides = hdr[6] | (hdr[7] << 8);
        uint16_t dataOffset = hdr[10] | (hdr[11] << 8);
        uint16_t extraHdrLen = hdr[12] | (hdr[13] << 8);

        wd->disk[UnitNum]->tracks = cyls - 1;
        wd->disk[UnitNum]->sides = sides;
        // Honor FDI hardware WP flag; otherwise defer to the caller's per-slot seed.
        wd->disk[UnitNum]->writeprotect = wpFlag ? 1 : 0;
        wd->disk[UnitNum]->fdiDataOffset = dataOffset;

        // Parse track headers to record their file positions
        uint32_t trkHdrPos = 14 + extraHdrLen;
        int totalTracks = cyls * sides;
        if (totalTracks > 168) totalTracks = 168;

        for (int i = 0; i < totalTracks; i++) {
            wd->disk[UnitNum]->fdiTrackHdrOffsets[i] = trkHdrPos;
            uint8_t trkHdr[7];
            f_lseek(wd->disk[UnitNum]->Diskfile, trkHdrPos);
            f_read(wd->disk[UnitNum]->Diskfile, trkHdr, 7, &br);
            uint8_t sectorCount = trkHdr[6];
            trkHdrPos += 7 + sectorCount * 7;
        }

        wd->disk[UnitNum]->t0s1_info = 0;
        wd->disk[UnitNum]->cursectbufpos = 0xff;
        wd->control |= kRVMWD177XPower0 << UnitNum;
        wd->disk[UnitNum]->fname = Filename;
        wd->diskLoadedCyl = -1;
        wd->diskLoadedSide = -1;
        wd->fastmode = false;

        printf("FDI: %d cylinders, %d sides\n", cyls, sides);
        return true;

    } else if (Filename.length() >= 4 &&
               (Filename.substr(Filename.length() - 4) == ".mbd" ||
                Filename.substr(Filename.length() - 4) == ".MBD" ||
                Filename.substr(Filename.length() - 4) == ".Mbd")) {
        // MBD file — raw sector dump (MB-02+ BS-DOS format)
        wd->disk[UnitNum]->IsSCLFile = false;
        wd->disk[UnitNum]->IsUDIFile = false;
        wd->disk[UnitNum]->IsFDIFile = false;
        wd->disk[UnitNum]->IsMBDFile = true;
        wd->disk[UnitNum]->sclDataOffset = 0;

        // Read MBD header to determine geometry
        uint8_t hdr[16];
        f_lseek(wd->disk[UnitNum]->Diskfile, 0);
        f_read(wd->disk[UnitNum]->Diskfile, hdr, 16, &br);

        uint8_t tracks = hdr[4]; // typically 82
        uint8_t spt = hdr[6];    // sectors per track (typically 11)
        uint8_t sides = hdr[8];  // typically 2

        if (tracks == 0 || spt == 0 || sides == 0) {
            // Fallback: assume standard HD format from file size
            FSIZE_t fsize = f_size(wd->disk[UnitNum]->Diskfile);
            tracks = 82; sides = 2; spt = 11;
            uint16_t secSize = (uint16_t)(fsize / (tracks * sides * spt));
            if (secSize != 256 && secSize != 512 && secSize != 1024) secSize = 1024;
            wd->disk[UnitNum]->mbdSectorSize = secSize;
        } else {
            // Derive sector size from file size
            FSIZE_t fsize = f_size(wd->disk[UnitNum]->Diskfile);
            uint16_t secSize = (uint16_t)(fsize / (tracks * sides * spt));
            if (secSize != 256 && secSize != 512 && secSize != 1024) secSize = 1024;
            wd->disk[UnitNum]->mbdSectorSize = secSize;
        }

        wd->disk[UnitNum]->tracks = tracks - 1;
        wd->disk[UnitNum]->sides = sides;
        wd->disk[UnitNum]->mbdSectorsPerTrack = spt;
        wd->disk[UnitNum]->writeprotect = 0; // MBD: BS-DOS requires writable disk

        wd->disk[UnitNum]->t0s1_info = 0;
        wd->disk[UnitNum]->cursectbufpos = 0xff;
        wd->control |= kRVMWD177XPower0 << UnitNum;
        wd->disk[UnitNum]->fname = Filename;
        wd->diskLoadedCyl = -1;
        wd->diskLoadedSide = -1;
        wd->fastmode = false;

        printf("MBD: %d tracks, %d sides, %d sec/trk, %d bytes/sec\n",
               tracks, sides, spt, wd->disk[UnitNum]->mbdSectorSize);
        return true;
#endif

    } else {
        wd->disk[UnitNum]->IsSCLFile = false;
#if !PICO_RP2040
        wd->disk[UnitNum]->IsUDIFile = false;
        wd->disk[UnitNum]->IsFDIFile = false;
        wd->disk[UnitNum]->IsMBDFile = false;
#endif
        // writeprotect is seeded by the caller from the per-slot Config array.
        wd->disk[UnitNum]->writeprotect = 0;
        wd->disk[UnitNum]->sclDataOffset = 0;
        wd->fastmode = Config::trdosFastMode;

        // fseek(wd->disk[UnitNum]->Diskfile,2048 + 227,SEEK_SET);
        // fread(&diskType,1,1,wd->disk[UnitNum]->Diskfile);
        f_lseek(wd->disk[UnitNum]->Diskfile,2048 + 227);
        f_read(wd->disk[UnitNum]->Diskfile, &diskType, 1, &br);
    }

    switch(diskType) {
        case 0x16:
            wd->disk[UnitNum]->tracks = 79;
            wd->disk[UnitNum]->sides = 2;
            break;
        case 0x17:
            wd->disk[UnitNum]->tracks = 39;
            wd->disk[UnitNum]->sides = 2;
            break;
        case 0x18:
            wd->disk[UnitNum]->tracks = 79;
            wd->disk[UnitNum]->sides = 1;
            break;
        case 0x19:
            wd->disk[UnitNum]->tracks = 39;
            wd->disk[UnitNum]->sides = 1;
            break;
        default:
            wdDiskEject(wd,UnitNum);
            Debug::led_blink();
            return false;
    }

    // Check if we have more tracks than on a standard disk
    if (!wd->disk[UnitNum]->IsSCLFile) {
      // Get file size
      f_lseek(wd->disk[UnitNum]->Diskfile, 0);
      long diskbytes = f_size(wd->disk[UnitNum]->Diskfile);
      if( diskbytes > wd->disk[UnitNum]->sides * wd->disk[UnitNum]->tracks * 16 * 256 ) {
        int i;
        for( int i = wd->disk[UnitNum]->tracks + 1; i < 83; i++ ) {
          if( wd->disk[UnitNum]->sides * i * 16 * 256 >= diskbytes ) {
            wd->disk[UnitNum]->tracks = i;
            break;
          }
        }
      }
    }

    //rewind(wd->disk[UnitNum]->Diskfile);
    f_rewind(wd->disk[UnitNum]->Diskfile);

    wd->disk[UnitNum]->t0s1_info = 0;
    wd->disk[UnitNum]->cursectbufpos = 0xff; // 0xffff;

    // Power on drive
    wd->control |= kRVMWD177XPower0 << UnitNum;

    printf("Disk %d inserted! Disktype: %d\n",UnitNum, (int) diskType);

    wd->disk[UnitNum]->fname = Filename;

    return true;

}

#if !PICO_RP2040
static void udiFlushTrack(rvmWD1793 *wd) {
    rvmwdDisk *disk = wd->disk[wd->diskS];
    if (!disk || wd->diskLoadedCyl < 0) return;
    int trkIdx = wd->diskLoadedCyl * disk->sides + wd->diskLoadedSide;
    if (trkIdx < 0 || trkIdx >= 168) return;
    uint16_t tlen = disk->udiTrackLengths[trkIdx];
    UINT bw;
    f_lseek(disk->Diskfile, disk->udiTrackOffsets[trkIdx]);
    f_write(disk->Diskfile, wd->diskTrackBuf, tlen, &bw);
    wd->diskDirty = false;
    // CRC32 at end of file not updated — our parser doesn't validate it
}

void udiLoadTrack(rvmWD1793 *wd, uint32_t cyl, uint8_t side) {
    if ((int)cyl == wd->diskLoadedCyl && (int)side == wd->diskLoadedSide)
        return;

    // Flush modified track back to file before switching tracks
    if (wd->diskDirty)
        udiFlushTrack(wd);

    rvmwdDisk *disk = wd->disk[wd->diskS];
    int trkIdx = cyl * disk->sides + side;
    if (trkIdx < 0 || trkIdx >= 168) {
        wd->diskTrackLen = 0;
        return;
    }

    uint16_t tlen = disk->udiTrackLengths[trkIdx];
    if (tlen > sizeof(wd->diskTrackBuf))
        tlen = sizeof(wd->diskTrackBuf);

    UINT br;
    f_lseek(disk->Diskfile, disk->udiTrackOffsets[trkIdx]);
    f_read(disk->Diskfile, wd->diskTrackBuf, tlen, &br);

    wd->diskTrackLen = tlen;
    wd->diskLoadedCyl = (int)cyl;
    wd->diskLoadedSide = (int)side;
}

// VG93/WD1793 CRC-CCITT (same algorithm as ZXMAK2 CrcVg93)
static uint16_t vgCrc(uint16_t crc, uint8_t byte) {
    crc ^= byte << 8;
    for (int i = 0; i < 8; i++)
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    return crc;
}

// Flush modified FDI track buffer back to FDI file.
// Parses MFM buffer to find sector data and writes it back at original FDI file offsets.
static void fdiFlushTrack(rvmWD1793 *wd) {
    rvmwdDisk *disk = wd->disk[wd->diskS];
    if (!disk || !disk->IsFDIFile || wd->diskLoadedCyl < 0) { wd->diskDirty = false; return; }

    int trkIdx = wd->diskLoadedCyl * disk->sides + wd->diskLoadedSide;
    if (trkIdx < 0 || trkIdx >= 168) { wd->diskDirty = false; return; }

    // Re-read FDI track header + sector descriptors
    uint8_t trkHdr[7];
    UINT br;
    f_lseek(disk->Diskfile, disk->fdiTrackHdrOffsets[trkIdx]);
    f_read(disk->Diskfile, trkHdr, 7, &br);

    uint32_t trkDataOffset = trkHdr[0] | (trkHdr[1]<<8) | (trkHdr[2]<<16) | (trkHdr[3]<<24);
    int secCount = trkHdr[6];
    if (secCount > 32) secCount = 32;
    if (secCount > wd->fdiSectorCount) secCount = wd->fdiSectorCount;

    uint8_t secHdrs[32 * 7];
    if (secCount > 0)
        f_read(disk->Diskfile, secHdrs, secCount * 7, &br);

    uint8_t *buf = wd->diskTrackBuf;
    int bufLen = wd->diskTrackLen;

    for (int sec = 0; sec < secCount; sec++) {
        uint8_t *sh = &secHdrs[sec * 7];
        uint8_t secN = sh[3];
        uint8_t flags = sh[4];
        uint16_t secDataOff = sh[5] | (sh[6] << 8);

        if (flags & 0x40) continue; // no data area

        int slen = 128 << (secN & 3);

        // Find data mark (0xFB/0xF8) after ID field in MFM buffer
        int idPos = (sec < 32) ? wd->fdiSectorIdPos[sec] : -1;
        if (idPos < 0) continue;
        int searchStart = idPos + 7; // skip FE + CHRN + 2 CRC bytes
        int dataPos = -1;
        for (int i = searchStart; i < searchStart + 80 && i < bufLen; i++) {
            if (buf[i] == 0xFB || buf[i] == 0xF8) {
                dataPos = i + 1;
                break;
            }
        }
        if (dataPos < 0 || dataPos + slen > bufLen) continue;

        // Write sector data back to FDI file
        uint32_t filePos = disk->fdiDataOffset + trkDataOffset + secDataOff;
        UINT bw;
        f_lseek(disk->Diskfile, filePos);
        f_write(disk->Diskfile, buf + dataPos, slen, &bw);

        // Update CRC flag only for sectors that were actually written
        if (!(flags & (1 << (secN & 3))) && sec < 32 && !(wd->fdiSectorFlags[sec] & 1)) {
            uint8_t newFlags = (1 << (secN & 3));
            if (flags & 0x80) newFlags |= 0x80;
            uint32_t flagsPos = disk->fdiTrackHdrOffsets[trkIdx] + 7 + sec * 7 + 4;
            f_lseek(disk->Diskfile, flagsPos);
            f_write(disk->Diskfile, &newFlags, 1, &bw);
        }
    }

    wd->diskDirty = false;
}

// Generate MFM track image from FDI sector data (ZXMAK2 approach).
// Called on demand when cylinder/side changes, same as udiLoadTrack.
void fdiLoadTrack(rvmWD1793 *wd, uint32_t cyl, uint8_t side) {
    if ((int)cyl == wd->diskLoadedCyl && (int)side == wd->diskLoadedSide)
        return;

    // Flush any pending writes before loading new track
    if (wd->diskDirty)
        fdiFlushTrack(wd);

    rvmwdDisk *disk = wd->disk[wd->diskS];
    int trkIdx = cyl * disk->sides + side;
    if (trkIdx < 0 || trkIdx >= 168) {
        wd->diskTrackLen = 0;
        return;
    }

    // Read FDI track header (4 bytes data offset + 2 reserved + 1 sector count)
    uint8_t trkHdr[7];
    UINT br;
    f_lseek(disk->Diskfile, disk->fdiTrackHdrOffsets[trkIdx]);
    f_read(disk->Diskfile, trkHdr, 7, &br);

    uint32_t trkDataOffset = trkHdr[0] | (trkHdr[1] << 8) | (trkHdr[2] << 16) | (trkHdr[3] << 24);
    int secCount = trkHdr[6];
    if (secCount > 32) secCount = 32;

    // Read sector headers (7 bytes each: C H R N Flags DataOffLo DataOffHi)
    uint8_t secHdrs[32 * 7];
    if (secCount > 0)
        f_read(disk->Diskfile, secHdrs, secCount * 7, &br);

    // Calculate total data length to determine gap sizes (ZXMAK2 algorithm)
    int imageSize = 6250;
    int trkdatalen = 0;
    for (int s = 0; s < secCount; s++) {
        uint8_t *sh = &secHdrs[s * 7];
        uint8_t flags = sh[4];
        int slen = 128 << (sh[3] & 3);
        trkdatalen += 2 + 6;  // A1 + FE + 6 bytes (CHRN + CRC)
        if (!(flags & 0x40)) { // has data area
            trkdatalen += 4;   // A1 + FB + 2 bytes CRC
            trkdatalen += slen;
        } else {
            slen = 0;
        }
    }

    // Dynamic gap sizing (ZXMAK2: distribute free space across gaps)
    int freeSpace = imageSize - (trkdatalen + secCount * (3 + 2)); // 3×4E + 2×00 per sector
    int synchroPulseLen = 1;
    int firstSpaceLen = 1;
    int secondSpaceLen = 1;
    int thirdSpaceLen = 1;
    int synchroSpaceLen = 1;
    freeSpace -= firstSpaceLen + secondSpaceLen + thirdSpaceLen + synchroSpaceLen;
    if (freeSpace < 0) {
        imageSize += -freeSpace;
        freeSpace = 0;
    }
    while (freeSpace > 0) {
        if (freeSpace >= (secCount * 2))
            if (synchroSpaceLen < 12) { synchroSpaceLen++; freeSpace -= secCount * 2; }
        if (freeSpace < secCount) break;
        if (firstSpaceLen < 10) { firstSpaceLen++; freeSpace -= secCount; }
        if (freeSpace < secCount) break;
        if (secondSpaceLen < 22) { secondSpaceLen++; freeSpace -= secCount; }
        if (freeSpace < secCount) break;
        if (thirdSpaceLen < 60) { thirdSpaceLen++; freeSpace -= secCount; }
        if (freeSpace < secCount) break;
        if (synchroSpaceLen >= 12 && firstSpaceLen >= 10 &&
            secondSpaceLen >= 22 && thirdSpaceLen >= 60) break;
    }
    if (freeSpace > (secCount * 2) + 10) { synchroPulseLen++; freeSpace -= secCount; }
    if (freeSpace > (secCount * 2) + 9) synchroPulseLen++;
    // Ensure minimum 3 A1 sync bytes for MFM mark detection (mark = 0xa1a1a1).
    // Each A1 is written twice per sector (ID + data), so extra A1s cost 2*secCount each.
    while (synchroPulseLen < 3) {
        synchroPulseLen++;
        imageSize += secCount * 2; // expand buffer to fit extra sync bytes
    }
    if (freeSpace < 0) { imageSize += -freeSpace; freeSpace = 0; }

    // Clamp to buffer size
    int bufSize = (int)sizeof(wd->diskTrackBuf);
    if (imageSize > bufSize) imageSize = bufSize;

    uint8_t *buf = wd->diskTrackBuf;
    int pos = 0;

    for (int sec = 0; sec < secCount; sec++) {
        uint8_t *sh = &secHdrs[sec * 7];
        uint8_t secC = sh[0], secH = sh[1], secR = sh[2], secN = sh[3];
        uint8_t flags = sh[4];
        uint16_t secDataOff = sh[5] | (sh[6] << 8);

        // Gap 1 (firstSpace)
        for (int i = 0; i < firstSpaceLen && pos < imageSize; i++) buf[pos++] = 0x4E;
        // Sync (synchroSpace)
        for (int i = 0; i < synchroSpaceLen && pos < imageSize; i++) buf[pos++] = 0x00;
        // Sync pulse (A1 bytes)
        int crcStart = pos;
        for (int i = 0; i < synchroPulseLen && pos < imageSize; i++) buf[pos++] = 0xA1;
        // Address mark
        if (sec < 32) {
            wd->fdiSectorIdPos[sec] = pos; // record position of 0xFE
            wd->fdiSectorFlags[sec] = (!(flags & (1 << (secN & 3))) ? 1 : 0)
                                    | ((flags & 0x40) ? 2 : 0);
        }
        if (pos < imageSize) buf[pos++] = 0xFE;
        // ID field: C H R N
        if (pos + 4 <= imageSize) {
            buf[pos++] = secC;
            buf[pos++] = secH;
            buf[pos++] = secR;
            buf[pos++] = secN;
        }
        // ID CRC
        uint16_t crc = 0xFFFF;
        for (int i = crcStart; i < pos; i++) crc = vgCrc(crc, buf[i]);
        if (pos + 2 <= imageSize) {
            buf[pos++] = (uint8_t)(crc >> 8);
            buf[pos++] = (uint8_t)(crc & 0xFF);
        }

        // Gap 2 (secondSpace)
        for (int i = 0; i < secondSpaceLen && pos < imageSize; i++) buf[pos++] = 0x4E;
        // Sync before data
        for (int i = 0; i < synchroSpaceLen && pos < imageSize; i++) buf[pos++] = 0x00;

        // Data area (only if bit6 of flags is clear)
        if (!(flags & 0x40)) {
            crcStart = pos;
            // Sync pulse
            for (int i = 0; i < synchroPulseLen && pos < imageSize; i++) buf[pos++] = 0xA1;
            // Data mark: F8=deleted, FB=normal
            uint8_t dataMark = (flags & 0x80) ? 0xF8 : 0xFB;
            if (pos < imageSize) buf[pos++] = dataMark;

            // Sector data from file
            int slen = 128 << (secN & 3);
            uint32_t filePos = disk->fdiDataOffset + trkDataOffset + secDataOff;
            int toRead = slen;
            if (pos + toRead > imageSize) toRead = imageSize - pos;
            f_lseek(disk->Diskfile, filePos);
            f_read(disk->Diskfile, buf + pos, toRead, &br);
            pos += toRead;

            // Data CRC
            crc = 0xFFFF;
            for (int i = crcStart; i < pos; i++) crc = vgCrc(crc, buf[i]);
            // Bad CRC if none of the low 6 bits are set
            if (!(flags & (1 << (secN & 3)))) crc ^= 0xFFFF;
            if (pos + 2 <= imageSize) {
                buf[pos++] = (uint8_t)(crc >> 8);
                buf[pos++] = (uint8_t)(crc & 0xFF);
            }
        }

        // Gap 3 (thirdSpace)
        for (int i = 0; i < thirdSpaceLen && pos < imageSize; i++) buf[pos++] = 0x4E;
    }

    wd->fdiSectorCount = (secCount < 32) ? secCount : 32;

    // Fill remainder with 0x4E
    while (pos < imageSize) buf[pos++] = 0x4E;

    wd->diskTrackLen = pos;
    wd->diskLoadedCyl = (int)cyl;
    wd->diskLoadedSide = (int)side;
}

// Generate MFM track image from MBD raw sector dump (MB-02+ BS-DOS format).
// MBD is a simple linear layout: tracks × sides × sectors × sectorSize bytes.
void mbdLoadTrack(rvmWD1793 *wd, uint32_t cyl, uint8_t side) {
    if ((int)cyl == wd->diskLoadedCyl && (int)side == wd->diskLoadedSide)
        return;

    // Flush any pending writes before loading new track
    if (wd->diskDirty)
        mbdFlushTrack(wd);

    rvmwdDisk *disk = wd->disk[wd->diskS];
    int spt = disk->mbdSectorsPerTrack;
    int secSize = disk->mbdSectorSize;
    uint8_t secN = (secSize == 1024) ? 3 : (secSize == 512) ? 2 : (secSize == 256) ? 1 : 0;

    // Base file offset for this track/side
    uint32_t trackOffset = ((uint32_t)cyl * disk->sides + side) * spt * secSize;

    // Build synthetic MFM track image (same approach as fdiLoadTrack)
    // Read each sector directly into diskTrackBuf to avoid large stack allocation
    int imageSize = (int)sizeof(wd->diskTrackBuf);
    uint8_t *buf = wd->diskTrackBuf;
    int pos = 0;

    // Gap sizes for HD MFM (approximate standard values)
    int gap1Len = 10;      // gap before each sector ID
    int syncLen = 12;      // 0x00 sync bytes
    int syncPulseLen = 3;  // 0xA1 sync pulses
    int gap2Len = 22;      // gap between ID and data
    int gap3Len = 30;      // gap after data

    UINT br;
    for (int sec = 0; sec < spt && sec < 32; sec++) {
        // Gap 1
        for (int i = 0; i < gap1Len && pos < imageSize; i++) buf[pos++] = 0x4E;
        // Sync
        for (int i = 0; i < syncLen && pos < imageSize; i++) buf[pos++] = 0x00;
        // Sync pulse (A1 bytes)
        int crcStart = pos;
        for (int i = 0; i < syncPulseLen && pos < imageSize; i++) buf[pos++] = 0xA1;
        // ID address mark
        wd->fdiSectorIdPos[sec] = pos;
        wd->fdiSectorFlags[sec] = 0; // no CRC errors, has data area
        if (pos < imageSize) buf[pos++] = 0xFE;
        // ID field: C H R N
        if (pos + 4 <= imageSize) {
            buf[pos++] = (uint8_t)cyl;
            buf[pos++] = side;
            buf[pos++] = (uint8_t)(sec + 1); // sectors numbered from 1
            buf[pos++] = secN;
        }
        // ID CRC
        uint16_t crc = 0xFFFF;
        for (int i = crcStart; i < pos; i++) crc = vgCrc(crc, buf[i]);
        if (pos + 2 <= imageSize) {
            buf[pos++] = (uint8_t)(crc >> 8);
            buf[pos++] = (uint8_t)(crc & 0xFF);
        }

        // Gap 2
        for (int i = 0; i < gap2Len && pos < imageSize; i++) buf[pos++] = 0x4E;
        // Sync before data
        for (int i = 0; i < syncLen && pos < imageSize; i++) buf[pos++] = 0x00;

        // Data area
        crcStart = pos;
        for (int i = 0; i < syncPulseLen && pos < imageSize; i++) buf[pos++] = 0xA1;
        if (pos < imageSize) buf[pos++] = 0xFB; // data mark

        // Read sector data directly from file into track buffer
        int toRead = secSize;
        if (pos + toRead > imageSize) toRead = imageSize - pos;
        uint32_t fileOffset = trackOffset + sec * secSize;
        f_lseek(disk->Diskfile, fileOffset);
        f_read(disk->Diskfile, buf + pos, toRead, &br);
        pos += toRead;

        // Data CRC
        crc = 0xFFFF;
        for (int i = crcStart; i < pos; i++) crc = vgCrc(crc, buf[i]);
        if (pos + 2 <= imageSize) {
            buf[pos++] = (uint8_t)(crc >> 8);
            buf[pos++] = (uint8_t)(crc & 0xFF);
        }

        // Gap 3
        for (int i = 0; i < gap3Len && pos < imageSize; i++) buf[pos++] = 0x4E;
    }

    wd->fdiSectorCount = (spt < 32) ? spt : 32;

    // Fill remainder with 0x4E
    while (pos < imageSize) buf[pos++] = 0x4E;

    wd->diskTrackLen = pos;
    wd->diskLoadedCyl = (int)cyl;
    wd->diskLoadedSide = (int)side;
}

// Flush modified MBD track buffer back to file
static void mbdFlushTrack(rvmWD1793 *wd) {
    if (!wd->diskDirty || wd->diskLoadedCyl < 0) return;

    rvmwdDisk *disk = wd->disk[wd->diskS];
    if (!disk || !disk->IsMBDFile || disk->writeprotect) {
        wd->diskDirty = false;
        return;
    }

    int spt = disk->mbdSectorsPerTrack;
    int secSize = disk->mbdSectorSize;

    // Extract sector data from MFM track buffer and write back
    for (int sec = 0; sec < spt && sec < wd->fdiSectorCount; sec++) {
        // Find data mark (0xFB) after the sector's ID mark
        int idPos = wd->fdiSectorIdPos[sec];
        // Skip: FE + C + H + R + N + CRC(2) + gap2 + sync + A1s + FB
        int searchStart = idPos + 7; // past ID mark and CHRN+CRC
        int dataStart = -1;
        for (int i = searchStart; i < (int)wd->diskTrackLen - 1; i++) {
            if (wd->diskTrackBuf[i] == 0xFB || wd->diskTrackBuf[i] == 0xF8) {
                dataStart = i + 1;
                break;
            }
        }
        if (dataStart < 0 || dataStart + secSize > (int)wd->diskTrackLen) continue;

        uint32_t fileOffset = ((uint32_t)wd->diskLoadedCyl * disk->sides + wd->diskLoadedSide) * spt * secSize
                            + sec * secSize;
        UINT bw;
        f_lseek(disk->Diskfile, fileOffset);
        f_write(disk->Diskfile, wd->diskTrackBuf + dataStart, secSize, &bw);
    }
    f_sync(disk->Diskfile);
    wd->diskDirty = false;
}

#endif

IRAM_ATTR uint8_t rvmwdDiskStep(rvmWD1793 *wd, uint32_t control) {

  rvmwdDisk *disk = wd->disk[wd->diskS];
  const uint32_t seek = control & 0x300;
  disk->a = 0;

  // Seek forward or backward
  if (seek) disk->t = (seek == 0x300) ? disk->t - (disk->t != 0) : disk->t + (disk->t < disk->tracks);

  disk->s = disk->t ? 0 : kRVMwdDiskOutTrack0;

  if(disk->indexDelay) {
    disk->indexDelay--;
    disk->s |= kRVMwdDiskOutIndex;
    return disk->s;
  }

  if (disk->cursectbufpos < 0xff) {

    disk->cursectbufpos++;

    disk->indx++;

    if(control & kRVMwdDiskControlWrite) {
      const uint8_t wr = control & 0xff;
      UINT bw;
      f_write(disk->Diskfile, &wr, 1, &bw);
      disk->cursectbuf[disk->cursectbufpos] = wr;
      return 0;
    }

    disk->a = disk->cursectbuf[disk->cursectbufpos];
    return disk->s;

  } else {

#if !PICO_RP2040
    if (disk->IsUDIFile) {

      // During seek (Type I), don't load track data — only update disk->t and status.
      // Track will be loaded on first actual data access (Read/Write Sector, etc.)
      if (seek)
        return disk->s;

      // Load track before checking length.
      // Don't reload track if an active command is reading/writing bytes
      {
        uint8_t loadSide = wd->side;
        bool activeCmd = (wd->stepState == kRVMWD177XStepReadByte
                       || wd->stepState == kRVMWD177XStepWaitingMark
                       || wd->stepState == kRVMWD177XStepWriteByte);
        if (activeCmd && wd->diskLoadedCyl == (int)disk->t && wd->diskLoadedSide >= 0)
            loadSide = (uint8_t)wd->diskLoadedSide;
        udiLoadTrack(wd, disk->t, loadSide);
      }

      if(disk->indx != 0xffffffff && disk->indx >= wd->diskTrackLen) {
        disk->indx = 0xffffffff;
        disk->indexDelay = 25;
        return disk->s;
      }

      disk->indx++;

      if(control & kRVMwdDiskControlWrite) {
        if (disk->indx < wd->diskTrackLen)
          wd->diskTrackBuf[disk->indx] = control & 0xff;
        wd->diskDirty = true;
        return 0;
      }

      if (disk->indx < wd->diskTrackLen)
        disk->a = wd->diskTrackBuf[disk->indx];
      else
        disk->a = 0x4e; // gap filler

      return disk->s;

    }

    if (disk->IsFDIFile) {

      if (seek)
        return disk->s;

      {
        uint8_t loadSide = wd->side;
        // Preserve the loaded track buffer only during actual data transfer
        // (ReadByte/WriteByte). During WaitingMark (mark search), always use
        // wd->side so a side change is detected and the correct track is loaded.
        bool activeCmd = (wd->stepState == kRVMWD177XStepReadByte
                       || wd->stepState == kRVMWD177XStepWriteByte);
        if (activeCmd && wd->diskLoadedCyl == (int)disk->t && wd->diskLoadedSide >= 0)
            loadSide = (uint8_t)wd->diskLoadedSide;
        fdiLoadTrack(wd, disk->t, loadSide);
      }

      if(disk->indx != 0xffffffff && disk->indx >= wd->diskTrackLen) {
        disk->indx = 0xffffffff;
        disk->indexDelay = 25;
        return disk->s;
      }

      disk->indx++;

      if(control & kRVMwdDiskControlWrite) {
        if (disk->indx < wd->diskTrackLen)
          wd->diskTrackBuf[disk->indx] = control & 0xff;
        wd->diskDirty = true;
        return 0;
      }

      if (disk->indx < wd->diskTrackLen)
        disk->a = wd->diskTrackBuf[disk->indx];
      else
        disk->a = 0x4e;

      return disk->s;

    }

    if (disk->IsMBDFile) {

      if (seek)
        return disk->s;

      {
        uint8_t loadSide = wd->side;
        bool activeCmd = (wd->stepState == kRVMWD177XStepReadByte
                       || wd->stepState == kRVMWD177XStepWriteByte);
        if (activeCmd && wd->diskLoadedCyl == (int)disk->t && wd->diskLoadedSide >= 0)
            loadSide = (uint8_t)wd->diskLoadedSide;
        mbdLoadTrack(wd, disk->t, loadSide);
      }

      if(disk->indx != 0xffffffff && disk->indx >= wd->diskTrackLen) {
        disk->indx = 0xffffffff;
        disk->indexDelay = 25;
        return disk->s;
      }

      disk->indx++;

      if(control & kRVMwdDiskControlWrite) {
        if (disk->indx < wd->diskTrackLen)
          wd->diskTrackBuf[disk->indx] = control & 0xff;
        wd->diskDirty = true;
        return 0;
      }

      if (disk->indx < wd->diskTrackLen)
        disk->a = wd->diskTrackBuf[disk->indx];
      else
        disk->a = 0x4e;

      return disk->s;

    }

#endif

    if(disk->indx != 0xffffffff && disk->indx >= /*6417*/ 6663) {
      disk->indx = 0xffffffff;
      disk->cursectbufpos = 0xff;
      disk->indexDelay = 25;
      return disk->s;
    }

    disk->indx++;

    const uint32_t cursect = (disk->indx - 146) / 392;

    // const uint32_t cursect = System34_track_info[disk->indx];

    if (cursect < 16) {

      if (disk->indx == sectdatapos[cursect]) // Track in sector header
        disk->a = (!disk->t && wd->side) ? disk->t0s1_info : disk->t;
      else if (disk->indx == sectdatapos[cursect] + 44) { // Sector data

        // const uint32_t side = (control & 0x800) << 1;

        if ((disk->IsSCLFile) && (!disk->t) && (!wd->side)) {

          // Create track0 from SCL file if not already done
          if (!wd->sclConverted) {
              SCLtoTRD(disk, wd->Track0);
              wd->sclConverted = true;
          }

          // SCL disk -> Read sector to cache from created Track0
          if (cursect < 9)
            memcpy(disk->cursectbuf, wd->Track0 + (cursect << 8), 0x100);
          else
            memset(disk->cursectbuf, 0, 0x100);

          disk->a = disk->cursectbuf[0];

        } else {

          const int seekptr = (disk->t << (11 + disk->sides)) + (wd->side << 12) + (cursect << 8) + disk->sclDataOffset;

          UINT br;
          f_lseek(disk->Diskfile,seekptr);
          f_read(disk->Diskfile, disk->cursectbuf, 0x100, &br);

          if(control & kRVMwdDiskControlWrite) {
            uint8_t wr = control & 0xff;
            UINT bw;
            f_lseek(disk->Diskfile,seekptr);
            f_write(disk->Diskfile, &wr,1,&bw);
            disk->cursectbuf[0] = wr;
          } else {
            disk->a = disk->cursectbuf[0];
          }

        }

        disk->cursectbufpos = 0;

      } else disk->a = System34_track[disk->indx];

    } else disk->a = System34_track[disk->indx];

    if(control & kRVMwdDiskControlWrite) {
      disk->a = 0;
      return 0;
    }

    return disk->s;

  }

}

void wdDiskEject(rvmWD1793 *wd, unsigned char UnitNum) {

  if(wd->disk[UnitNum] != NULL) {

    printf("Ejecting disk\n");

    if (wd->disk[UnitNum]->Diskfile != NULL) {
#if !PICO_RP2040
        if (wd->diskDirty && wd->diskS == UnitNum) {
            if (wd->disk[UnitNum]->IsUDIFile) udiFlushTrack(wd);
            else if (wd->disk[UnitNum]->IsFDIFile) fdiFlushTrack(wd);
            else if (wd->disk[UnitNum]->IsMBDFile) mbdFlushTrack(wd);
        }
#endif
        fclose2(wd->disk[UnitNum]->Diskfile);
        wd->disk[UnitNum]->Diskfile = NULL;
    }

    free(wd->disk[UnitNum]);
    wd->disk[UnitNum] = NULL;

    if (wd->diskS == UnitNum) {

      _end(wd);

      wd->side = 0;

      wd->sclConverted = false;

    }

    // Power off drive
    wd->control &= ~(kRVMWD177XPower0 << UnitNum);

  } else printf("No disk to eject\n");

}

void SCLtoTRD(rvmwdDisk *d, unsigned char* track0) {

    uint8_t numberOfFiles;

    // Reset track 0 info
    memset(track0,0,2304);

    // fseek(d->Diskfile,8,SEEK_SET);
    // fread(&numberOfFiles,1,1,d->Diskfile);
    UINT br;
    f_lseek(d->Diskfile,8);
    f_read(d->Diskfile, &numberOfFiles,1,&br);

    // printf("Number of files: %d\n",(int)numberOfFiles);

    char diskNameArray[9]="SCL_DISK";

    // Populate FAT.
    int startSector = 0;
    int startTrack = 1; // Since Track 0 is reserved for FAT and Disk Specification.

    uint8_t data;
    for (int i = 0; i < numberOfFiles; i++) {

        int n = i << 4;

        UINT bw;
        for (int j = 0; j < 13; j++) {
            // fread(&data,1,1,d->Diskfile);
            f_read(d->Diskfile, &data,1,&br);
            track0[n + j] = data;
        }

        // fread(&data,1,1,d->Diskfile); // Filelenght
        f_read(d->Diskfile, &data,1,&br);
        track0[n + 13] = data;

        // printf("File #: %d, Filelenght: %d\n",i + 1,(int)data);

        track0[n + 14] = (uint8_t)startSector;
        track0[n + 15] = (uint8_t)startTrack;

        int newStartTrack = (startTrack * 16 + startSector + data) / 16;
        startSector = (startTrack * 16 + startSector + data) - 16 * newStartTrack;
        startTrack = newStartTrack;

    }

    // Populate Disk Specification.
    track0[2273] = (uint8_t)startSector;
    track0[2274] = (uint8_t)startTrack;
    track0[2275] = 22; // Disk Type
    track0[2276] = (uint8_t)numberOfFiles; // File Count
    uint16_t freeSectors = 2560 - (startTrack << 4) + startSector;
    // printf("Free Sectors: %d\n",freeSectors);
    track0[2277] = freeSectors & 0x00ff;
    track0[2278] = freeSectors >> 8;
    track0[2279] = 0x10; // TR-DOS ID

    for (int i = 0; i < 9; i++) track0[2282 + i] = 0x20;

    // Store the image file name in the disk label section of the Disk Specification.
    for (int i = 0; i < 8; i++) track0[2293 + i] = diskNameArray[i];

    d->sclDataOffset =  (9 + (numberOfFiles * 14)) - 4096;

}

// Create an empty formatted TRD disk image at the given path.
// 80 tracks, 2 sides, 16 sectors/track, 256 bytes/sector = 655360 bytes.
// Track 0: 9 catalog sectors (0xFF-filled) + service sector + 6 zero sectors.
bool rvmWD1793CreateEmptyTRD(const char *path) {
    FIL f;
    if (f_open(&f, path, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) return false;
    UINT bw;
    uint8_t buf[256];

    // Sectors 0-7: catalog (empty = zeros, 8 sectors = 128 directory entries)
    memset(buf, 0, 256);
    for (int s = 0; s < 8; s++) f_write(&f, buf, 256, &bw);

    // Sector 9 (0-indexed: 8): service sector (disk info at offsets 0xE1-0xE7)
    // Reader expects this at file offset 2048+227 = 8*256+0xE3
    memset(buf, 0, 256);
    buf[0xe1] = 0x00; // first free sector on free track
    buf[0xe2] = 0x01; // first free track (track 1)
    buf[0xe3] = 0x16; // disk type: 80T 2DS
    buf[0xe4] = 0x00; // number of files
    buf[0xe5] = 0xF0; // free sectors low  (0x09F0 = 2544)
    buf[0xe6] = 0x09; // free sectors high
    buf[0xe7] = 0x10; // TR-DOS ID byte
    // 0xEA-0xF2: 9 spaces (password field)
    memset(buf + 0xea, 0x20, 9);
    // 0xF5-0xFC: disk name (8 chars)
    memcpy(buf + 0xf5, "NEW DISK", 8);
    f_write(&f, buf, 256, &bw);

    // Sectors 9-15: rest of track 0, zeros
    memset(buf, 0, 256);
    for (int s = 9; s < 16; s++) f_write(&f, buf, 256, &bw);

    // Remaining 2544 sectors (tracks 1-79, both sides): zeros
    for (int s = 0; s < 2544; s++) f_write(&f, buf, 256, &bw);

    f_close(&f);
    return true;
}

// Convert SCL disk to TRD file on first write attempt.
// Creates a .trd file alongside the .scl, copies all data, and switches the disk handle.
static bool sclConvertToTRD(rvmWD1793 *wd) {
    rvmwdDisk *disk = wd->disk[wd->diskS];
    if (!disk || !disk->IsSCLFile || !disk->Diskfile) return false;

    // Ensure Track0 is populated
    if (!wd->sclConverted) {
        SCLtoTRD(disk, wd->Track0);
        wd->sclConverted = true;
    }

    // Build .trd filename from .scl filename
    std::string trdName = disk->fname;
    size_t dotPos = trdName.rfind('.');
    if (dotPos != std::string::npos)
        trdName = trdName.substr(0, dotPos);
    trdName += ".trd";

    // Create TRD file
    Debug::log("SCL->TRD: creating %s", trdName.c_str());
    FIL *trdFile = fopen2(trdName.c_str(), FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
    if (!trdFile) {
        Debug::log("SCL->TRD: fopen2 failed");
        return false;
    }

    UINT bw;
    uint8_t zeroBuf[256];
    memset(zeroBuf, 0, 256);

    // Write track 0 side 0: 16 sectors from Track0 (first 4096 bytes = 16 * 256)
    f_write(trdFile, wd->Track0, 2304, &bw);
    // Pad remaining sectors of track 0 (sectors 9..15 are already in Track0 as zeros,
    // but Track0 is only 2304 bytes = 9 sectors; pad to 16 sectors = 4096 bytes)
    for (int s = 9; s < 16; s++)
        f_write(trdFile, zeroBuf, 256, &bw);

    // Copy remaining data from SCL using same seek formula as rvmwdDiskStep
    // TRD layout: [T0S0: 16*256][T0S1: 16*256][T1S0: 16*256][T1S1: 16*256]...
    uint8_t secBuf[256];
    int totalTracks = disk->tracks + 1;
    int sclFileSize = (int)f_size(disk->Diskfile);
    for (int trk = 0; trk < totalTracks; trk++) {
        for (int side = 0; side < disk->sides; side++) {
            for (int sec = 0; sec < 16; sec++) {
                // Skip track 0 side 0 — already written from Track0
                if (trk == 0 && side == 0) continue;
                int seekptr = (trk << (11 + disk->sides)) + (side << 12) + (sec << 8) + disk->sclDataOffset;
                if (seekptr >= 0 && seekptr < sclFileSize) {
                    UINT br;
                    f_lseek(disk->Diskfile, seekptr);
                    f_read(disk->Diskfile, secBuf, 256, &br);
                    if (br < 256) memset(secBuf + br, 0, 256 - br);
                } else {
                    memset(secBuf, 0, 256);
                }
                // TRD file offset for this sector
                uint32_t trdOffset = (trk * disk->sides + side) * 16 * 256 + sec * 256;
                f_lseek(trdFile, trdOffset);
                f_write(trdFile, secBuf, 256, &bw);
            }
        }
    }

    // Flush TRD to SD card
    f_sync(trdFile);

    // Close SCL, switch to TRD
    fclose2(disk->Diskfile);
    disk->Diskfile = trdFile;
    disk->IsSCLFile = false;
    disk->sclDataOffset = 0;
    disk->fname = trdName;

    Debug::log("SCL->TRD: done, size=%d", (int)f_size(trdFile));
    return true;
}