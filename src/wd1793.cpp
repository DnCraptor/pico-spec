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
#if !PICO_RP2040
  if (wd->disk[wd->diskS] && wd->disk[wd->diskS]->IsUDIFile && wd->track >= 0x40)
    Debug::log("E T%02x S%02x c%02x st%02x",
               wd->track, wd->sector, wd->command, wd->status);
#endif
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
#if !PICO_RP2040
      if (wd->disk[wd->diskS] && wd->disk[wd->diskS]->IsUDIFile && wd->dsr >= 0x40)
        Debug::log("chk T%02x dsr%02x dt%d", wd->track, wd->dsr, wd->disk[wd->diskS]->t);
#endif
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
#if !PICO_RP2040
      if (wd->disk[wd->diskS] && wd->disk[wd->diskS]->IsUDIFile && wd->track >= 0x28)
        Debug::log("T1end T%02x v%d", wd->track, (wd->command & kRVMWD177XVerifyBit) ? 1 : 0);
#endif
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
        //printf("Type I end\n");
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
      //

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

      if((wd->command & 0xc0)==0x80) { // Read or Write Sector

        if(wd->command & 0x20) { // Write Sector
          if(wd->disk[wd->diskS]->writeprotect) {
            wd->status|=kRVMWD177XStatusProtected;
            _end(wd);
            return;
          }
          // printf("WRITE SECTOR: track: %d, sector: %d\n",wd->track,wd->sector);
        } else {
          // printf("READ SECTOR: track: %d, sector: %d\n",wd->track,wd->sector);
        }
        wd->retry=5; //5 retrys

        // wd->state=kRVMWD177XReadHeader;
        // wd->next=kRVMWD177XReadSectorHeader;
        // _do(wd);

        // _waitMark(wd);
        // printf("Waitmark ReadHeader\n");
        wd->stepState=kRVMWD177XStepWaitingMark;
        wd->marka = mark;
        //

        wd->headerI=0xff;
        wd->state=kRVMWD177XReadHeaderBytes;
        wd->next=kRVMWD177XReadSectorHeader;

      } else if((wd->command & 0xf0)==0xc0) { // Read Address

        wd->retry=5; //5 retrys
        wd->state=kRVMWD177XReadAddressWait;
        // printf("State -> ReadAddressWait\n");
        _do(wd);

      } else if((wd->command & 0xf0)==0xf0) { // Write Track

        if(wd->disk[wd->diskS]->writeprotect) {
          wd->status|=kRVMWD177XStatusProtected;
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

        if((wd->command & 0x2) && (wd->header[2] & ((wd->command>>3) & 0x1))) {
          wd->state=kRVMWD177XReadHeader;
          _do(wd);
          return;
        }

      } else {

          if((wd->command & 0x2) && (wd->header[2] & ((wd->command>>3) & 0x1))) {
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
        // Use real size for UDI/FDI, hardcode 256 for TRD/SCL (Betadisk standard)
        {
            uint32_t sz = 128 << (wd->header[4] & 0x03);
#if !PICO_RP2040
            if (!wd->disk[wd->diskS]->IsUDIFile && !wd->disk[wd->diskS]->IsFDIFile)
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
#if !PICO_RP2040
        if (wd->disk[wd->diskS] && wd->disk[wd->diskS]->IsUDIFile)
          Debug::log("LostData T=%02x S=%02x c=%d", wd->track, wd->sector, (int)wd->c);
#endif
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

        // if(wd->crc) {
        //   wd->status|=kRVMWD177XStatusCRC;
        // } else {
          wd->status&=~kRVMWD177XStatusCRC;
        // }

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

        if ((wd->track == 0 || wd->track == 0xff) && wd->sector == 0) {
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

        if((wd->marka & 0xff) == dd) {
          wd->marka >>= 8;
          if(!wd->marka) {
            // printf("Mark found for Track: %d, Sector: %d, Index: %d!\n",wd->track, wd->sector, wd->disk[wd->diskS]->indx);
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
#if !PICO_RP2040
  if (wd->disk[wd->diskS] && wd->disk[wd->diskS]->IsUDIFile && wd->track == 0x4b) {
    static int wr4b_cnt = 0;
    if (wr4b_cnt < 20) {
      Debug::log("WR r%d=%02x T%02x S%02x st=%04x", a&3, value, wd->track, wd->sector, wd->status);
      wr4b_cnt++;
    }
  }
#endif
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
#if !PICO_RP2040
        if (wd->disk[wd->diskS] && wd->disk[wd->diskS]->IsUDIFile && wd->track >= 0x40)
          Debug::log("FINT %02x T%02x busy=%d st=%04x ctl=%04x",
                     value, wd->track, (wd->status & kRVMWD177XStatusBusy)?1:0,
                     wd->status, wd->control);
#endif
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

#if !PICO_RP2040
      if (wd->disk[wd->diskS] && wd->disk[wd->diskS]->IsUDIFile && (wd->status & kRVMWD177XStatusBusy) && wd->track >= 0x40)
          Debug::log("BUSY! C%02x T%02x S%02x st%02x state=%d",
                     value, wd->track, wd->sector, wd->status, wd->state);
#endif
      if(!(wd->status & kRVMWD177XStatusBusy)) {

        wd->control &= ~(kRVMWD177XINTRQ|kRVMWD177XFINTRQ);

        wd->command = value;

#if !PICO_RP2040
        if (wd->disk[wd->diskS] && wd->disk[wd->diskS]->IsUDIFile && wd->track >= 0x40)
          Debug::log("C%02x T%02x>%02x S%02x s%d",
                     value, wd->track, wd->data, wd->sector, wd->side);
#endif

        if(wd->disk[wd->diskS]  && (wd->control & (kRVMWD177XPower0 << wd->diskS))) {

          //Issue command
          if(wd->command & kRVMWD177XTypeI) {

            // printf("TYPE II,III or IV COMMAND: %02x TRACK: %02x SECTOR %02x SIDE %02x\n",wd->command,wd->track,wd->sector, wd->side);

            //Type II, III, IV kRVMWD177XTypeIISetHead
            if((wd->command & 0xc0)==0x80 || (wd->command & 0xfb)==0xc0 || (wd->command & 0xfb)==0xf0 || (wd->command & 0xfb)==0xe0) {

              //TYPE
              wd->state=kRVMWD177XTypeIISetHead;
              wd->status=kRVMWD177XStatusBusy;

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

              wd->control |= kRVMWD177XINTRQ;

            }

          } else {

            // printf("TYPE I COMMAND: %02x TRACK: %02x SECTOR %02x SIDE %02x\n",wd->command,wd->track,wd->sector, wd->side);

            //Type I
            // Clear DRQ (mirrors UnrealSpeccy S_TYPE1_CMD: status &= ~WDS_DRQ, rqs=0)
            wd->control &= ~kRVMWD177XDRQ;
            wd->status = kRVMWD177XStatusSetIndex | kRVMWD177XStatusSetTrack0 | kRVMWD177XStatusSetWP | kRVMWD177XStatusBusy;
            wd->state = kRVMWD177XTypeI0;

            _do(wd);

            // Complete Type I (Seek/Step/Restore) immediately — don't wait for step delays.
            // Copy-protected loaders issue the next command before step delays expire.
            if (!(wd->command & kRVMWD177XTypeI)) {
              while (wd->stepState == kRVMWD177XStepWaiting) {
                _do(wd);
              }
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
        if((wd->status & kRVMWD177XStatusSetHead) && (wd->control & kRVMWD177XTest)) {
          r|=kRVMWD177XStatusHeadLoaded;
        }
      } else {
        r|=kRVMWD177XStatusNotReady;
      }

      //printf("Read status: %02x\n",r);
#if !PICO_RP2040
      if (wd->disk[wd->diskS] && wd->disk[wd->diskS]->IsUDIFile && wd->track == 0x4b) {
        static int st4b_cnt = 0;
        if (st4b_cnt < 10) {
          Debug::log("RD_ST T4b st=%02x ctl=%04x step=%d state=%d cnt=%d",
                     r, wd->control, wd->stepState, wd->state, st4b_cnt);
          st4b_cnt++;
        }
      }
#endif
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

void rvmWD1793Reset(rvmWD1793 *wd) {

  wd->state = kRVMWD177XNone;
  wd->stepState = kRVMWD177XStepIdle;
  wd->next = kRVMWD177XNone;

  wd->c = 0;
  wd->control &= 0xf000;
  wd->command = wd->sector = wd->data = wd->dsr = 0x0;
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
  wd->fastmode = Config::trdosFastMode; //(Config::DiskCtrl == 2 || Config::DiskCtrl == 4);
  wd->sclConverted = false;
#if !PICO_RP2040
  wd->udiLoadedCyl = -1;
  wd->udiLoadedSide = -1;
  wd->udiTrackLen = 0;
  wd->udiDirty = false;
#endif
}

bool rvmWD1793InsertDisk(rvmWD1793 *wd, unsigned char UnitNum, std::string Filename) {

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
        wd->disk[UnitNum]->writeprotect = 1; // SCL files are read only
        wd->fastmode = Config::trdosFastMode;
        diskType = 0x16;

#if !PICO_RP2040
    } else if (std::strncmp(magic,"UDI!",4) == 0) {
        // UDI file
        printf("UDI disk loaded\n");
        Debug::log("UDI disk loaded unit=%d wp=%d", UnitNum, Config::trdosWriteProtect);
        wd->disk[UnitNum]->IsSCLFile = false;
        wd->disk[UnitNum]->IsUDIFile = true;
        wd->disk[UnitNum]->IsFDIFile = false;
        wd->disk[UnitNum]->writeprotect = Config::trdosWriteProtect;
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
        wd->udiLoadedCyl = -1;
        wd->udiLoadedSide = -1;
        wd->fastmode = false; // fastmode uses sectdatapos, incompatible with raw MFM

        printf("UDI: %d cylinders, %d sides\n", cyls, sides);
        return true;

    } else if (std::strncmp(magic,"FDI",3) == 0) {
        // FDI file
        printf("FDI disk loaded\n");
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
        wd->disk[UnitNum]->writeprotect = wpFlag ? 1 : Config::trdosWriteProtect;
        wd->disk[UnitNum]->fdiDataOffset = dataOffset;

        // Parse track headers — located right after the 14-byte header + extra header
        uint32_t trkHdrPos = 14 + extraHdrLen;
        int totalTracks = cyls * sides;
        if (totalTracks > 168) totalTracks = 168;

        for (int i = 0; i < totalTracks; i++) {
            wd->disk[UnitNum]->fdiTrackHdrOffsets[i] = trkHdrPos;

            // Read track header to get sector count
            uint8_t trkHdr[7];
            f_lseek(wd->disk[UnitNum]->Diskfile, trkHdrPos);
            f_read(wd->disk[UnitNum]->Diskfile, trkHdr, 7, &br);

            uint8_t sectorCount = trkHdr[6];

            // Skip past track header (7 bytes) + sector headers (7 bytes each)
            trkHdrPos += 7 + sectorCount * 7;
        }

        wd->disk[UnitNum]->t0s1_info = 0;
        wd->disk[UnitNum]->cursectbufpos = 0xff;
        wd->control |= kRVMWD177XPower0 << UnitNum;
        wd->disk[UnitNum]->fname = Filename;
        wd->udiLoadedCyl = -1;
        wd->udiLoadedSide = -1;
        wd->fastmode = false; // fastmode uses sectdatapos, incompatible with raw MFM

        printf("FDI: %d cylinders, %d sides\n", cyls, sides);
        return true;
#endif

    } else {
        wd->disk[UnitNum]->IsSCLFile = false;
#if !PICO_RP2040
        wd->disk[UnitNum]->IsUDIFile = false;
        wd->disk[UnitNum]->IsFDIFile = false;
#endif
        wd->disk[UnitNum]->writeprotect = Config::trdosWriteProtect;
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
    if (!disk || wd->udiLoadedCyl < 0) return;
    int trkIdx = wd->udiLoadedCyl * disk->sides + wd->udiLoadedSide;
    if (trkIdx < 0 || trkIdx >= 168) return;
    uint16_t tlen = disk->udiTrackLengths[trkIdx];
    UINT bw;
    f_lseek(disk->Diskfile, disk->udiTrackOffsets[trkIdx]);
    f_write(disk->Diskfile, wd->udiTrackBuf, tlen, &bw);
    Debug::log("udiFlush cyl=%d side=%d tlen=%d bw=%d off=%u",
               wd->udiLoadedCyl, wd->udiLoadedSide, tlen, bw,
               (unsigned)disk->udiTrackOffsets[trkIdx]);
    wd->udiDirty = false;
    // CRC32 at end of file not updated — our parser doesn't validate it
}

void udiLoadTrack(rvmWD1793 *wd, uint32_t cyl, uint8_t side) {
    if ((int)cyl == wd->udiLoadedCyl && (int)side == wd->udiLoadedSide)
        return;

    // Flush modified track back to file before switching tracks
    if (wd->udiDirty)
        udiFlushTrack(wd);

    rvmwdDisk *disk = wd->disk[wd->diskS];
    int trkIdx = cyl * disk->sides + side;
    if (trkIdx < 0 || trkIdx >= 168) {
        wd->udiTrackLen = 0;
        return;
    }

    uint16_t tlen = disk->udiTrackLengths[trkIdx];
    if (tlen > sizeof(wd->udiTrackBuf))
        tlen = sizeof(wd->udiTrackBuf);

    UINT br;
    f_lseek(disk->Diskfile, disk->udiTrackOffsets[trkIdx]);
    f_read(disk->Diskfile, wd->udiTrackBuf, tlen, &br);

    wd->udiTrackLen = tlen;
    wd->udiLoadedCyl = (int)cyl;
    wd->udiLoadedSide = (int)side;
}

static uint16_t fdiCrc16(uint16_t crc, uint8_t byte) {
    crc ^= byte << 8;
    for (int i = 0; i < 8; i++)
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    return crc;
}

void fdiLoadTrack(rvmWD1793 *wd, uint32_t cyl, uint8_t side) {
    if ((int)cyl == wd->udiLoadedCyl && (int)side == wd->udiLoadedSide)
        return;

    rvmwdDisk *disk = wd->disk[wd->diskS];
    int trkIdx = cyl * disk->sides + side;
    if (trkIdx < 0 || trkIdx >= 168) {
        wd->udiTrackLen = 0;
        return;
    }

    uint8_t *buf = wd->udiTrackBuf;
    int pos = 0;

    // Track header: 80×0x4E gap
    for (int i = 0; i < 80 && pos < 6400; i++) buf[pos++] = 0x4e;
    // 12×0x00 sync
    for (int i = 0; i < 12 && pos < 6400; i++) buf[pos++] = 0x00;
    // Index mark: 0xC2,0xC2,0xC2,0xFC
    if (pos < 6396) { buf[pos++] = 0xc2; buf[pos++] = 0xc2; buf[pos++] = 0xc2; buf[pos++] = 0xfc; }
    // 50×0x4E gap
    for (int i = 0; i < 50 && pos < 6400; i++) buf[pos++] = 0x4e;

    // Read FDI track header
    uint8_t trkHdr[7];
    UINT br;
    f_lseek(disk->Diskfile, disk->fdiTrackHdrOffsets[trkIdx]);
    f_read(disk->Diskfile, trkHdr, 7, &br);

    uint32_t trkDataOffset = trkHdr[0] | (trkHdr[1] << 8) | (trkHdr[2] << 16) | (trkHdr[3] << 24);
    uint8_t sectorCount = trkHdr[6];

    // Read sector headers
    uint8_t secHdrs[16 * 7]; // max 16 sectors
    int nsec = sectorCount;
    if (nsec > 16) nsec = 16;
    if (nsec > 0) {
        f_read(disk->Diskfile, secHdrs, nsec * 7, &br);
    }

    // Generate sectors
    for (int s = 0; s < nsec && pos < 6300; s++) {
        uint8_t *sh = &secHdrs[s * 7];
        uint8_t secCyl = sh[0];
        uint8_t secHead = sh[1];
        uint8_t secNum = sh[2];
        uint8_t secSizeCode = sh[3];
        uint8_t secFlags = sh[4];
        uint16_t secDataOff = sh[5] | (sh[6] << 8);

        uint16_t secSize = 128 << secSizeCode;
        if (secSize > 1024) secSize = 1024; // clamp for buffer safety

        // Sector header sync: 12×0x00
        for (int i = 0; i < 12 && pos < 6400; i++) buf[pos++] = 0x00;
        // Address mark: 0xA1,0xA1,0xA1,0xFE
        if (pos < 6396) { buf[pos++] = 0xa1; buf[pos++] = 0xa1; buf[pos++] = 0xa1; buf[pos++] = 0xfe; }
        // ID field: cyl, head, sector, size code
        uint16_t crc = 0xffff;
        crc = fdiCrc16(crc, 0xa1); crc = fdiCrc16(crc, 0xa1); crc = fdiCrc16(crc, 0xa1); crc = fdiCrc16(crc, 0xfe);
        if (pos < 6396) {
            buf[pos++] = secCyl;  crc = fdiCrc16(crc, secCyl);
            buf[pos++] = secHead; crc = fdiCrc16(crc, secHead);
            buf[pos++] = secNum;  crc = fdiCrc16(crc, secNum);
            buf[pos++] = secSizeCode; crc = fdiCrc16(crc, secSizeCode);
            buf[pos++] = (crc >> 8) & 0xff;
            buf[pos++] = crc & 0xff;
        }
        // Gap2: 22×0x4E
        for (int i = 0; i < 22 && pos < 6400; i++) buf[pos++] = 0x4e;
        // Data sync: 12×0x00
        for (int i = 0; i < 12 && pos < 6400; i++) buf[pos++] = 0x00;

        // Data mark
        bool hasData = !(secFlags & 0x40);
        bool deleted = (secFlags & 0x80);
        uint8_t dataMark = deleted ? 0xf8 : 0xfb;
        if (pos < 6396) { buf[pos++] = 0xa1; buf[pos++] = 0xa1; buf[pos++] = 0xa1; buf[pos++] = dataMark; }

        // Data CRC init
        crc = 0xffff;
        crc = fdiCrc16(crc, 0xa1); crc = fdiCrc16(crc, 0xa1); crc = fdiCrc16(crc, 0xa1); crc = fdiCrc16(crc, dataMark);

        // Sector data
        if (hasData) {
            uint32_t fileDataPos = disk->fdiDataOffset + trkDataOffset + secDataOff;
            uint16_t toRead = secSize;
            if (pos + toRead > 6400) toRead = 6400 - pos;
            f_lseek(disk->Diskfile, fileDataPos);
            f_read(disk->Diskfile, buf + pos, toRead, &br);
            for (int i = 0; i < (int)toRead; i++) {
                crc = fdiCrc16(crc, buf[pos + i]);
            }
            pos += toRead;
        } else {
            for (int i = 0; i < (int)secSize && pos < 6400; i++) {
                buf[pos++] = 0x00;
                crc = fdiCrc16(crc, 0x00);
            }
        }

        // Data CRC
        if (pos < 6398) {
            buf[pos++] = (crc >> 8) & 0xff;
            buf[pos++] = crc & 0xff;
        }

        // Gap3: variable (54 for 256-byte sectors, less for larger)
        int gap3 = (secSizeCode <= 1) ? 54 : 24;
        for (int i = 0; i < gap3 && pos < 6400; i++) buf[pos++] = 0x4e;
    }

    // Fill rest with 0x4E
    while (pos < 6400) buf[pos++] = 0x4e;

    wd->udiTrackLen = pos;
    wd->udiLoadedCyl = (int)cyl;
    wd->udiLoadedSide = (int)side;
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
      //Debug::led_blink();
      //fwrite(&wr,1,1,disk->Diskfile);
      f_write(disk->Diskfile, &wr, 1, &bw);
      disk->cursectbuf[disk->cursectbufpos] = wr;
      return 0;
    }

    disk->a = disk->cursectbuf[disk->cursectbufpos];
    return disk->s;

  } else {

#if !PICO_RP2040
    if (disk->IsUDIFile) {

      // Load track before checking length
      udiLoadTrack(wd, disk->t, wd->side);

      if(disk->indx != 0xffffffff && disk->indx >= wd->udiTrackLen) {
        disk->indx = 0xffffffff;
        disk->indexDelay = 25;
        return disk->s;
      }

      disk->indx++;

      if(control & kRVMwdDiskControlWrite) {
        if (disk->indx < wd->udiTrackLen)
          wd->udiTrackBuf[disk->indx] = control & 0xff;
        if (!wd->udiDirty)
          Debug::log("udiWrite first byte=%02x indx=%u cyl=%d side=%d",
                     control & 0xff, disk->indx, wd->udiLoadedCyl, wd->udiLoadedSide);
        wd->udiDirty = true;
        return 0;
      }

      if (disk->indx < wd->udiTrackLen)
        disk->a = wd->udiTrackBuf[disk->indx];
      else
        disk->a = 0x4e; // gap filler

      return disk->s;

    }

    if (disk->IsFDIFile) {

      // Generate MFM track before checking length
      fdiLoadTrack(wd, disk->t, wd->side);

      if(disk->indx != 0xffffffff && disk->indx >= wd->udiTrackLen) {
        disk->indx = 0xffffffff;
        disk->indexDelay = 25;
        return disk->s;
      }

      disk->indx++;

      if (disk->indx < wd->udiTrackLen)
        disk->a = wd->udiTrackBuf[disk->indx];
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
            // fseek(disk->Diskfile,seekptr,SEEK_SET);
            //fwrite(&wr,1,1,disk->Diskfile);
            //Debug::led_blink();
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
        if (wd->disk[UnitNum]->IsUDIFile && wd->udiDirty && wd->diskS == UnitNum)
            udiFlushTrack(wd);
#endif
        f_close(wd->disk[UnitNum]->Diskfile);
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