/*

ESPectrum, a Sinclair ZX Spectrum emulator for Espressif ESP32 SoC

Copyright (c) 2023, 2024 Víctor Iborra [Eremus] and 2023 David Crespo [dcrespo3d]
https://github.com/EremusOne/ZX-ESPectrum-IDF

Based on ZX-ESPectrum-Wiimote
Copyright (c) 2020, 2022 David Crespo [dcrespo3d]
https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote

Based on previous work by Ramón Martinez and Jorge Fuertes
https://github.com/rampa069/ZX-ESPectrum

Original project by Pete Todd
https://github.com/retrogubbins/paseVGA

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

To Contact the dev team you can write to zxespectrum@gmail.com or 
visit https://zxespectrum.speccy.org/contacto

*/

#include <stdio.h>
#include <vector>
#include <string>
#include <inttypes.h>
#include "miniz/miniz.h"
// #include "rom/miniz.h"

using namespace std;

#include "Tape.h"
#include "FileUtils.h"
#include "CPU.h"
#include "Video.h"
#include "OSDMain.h"
#include "Config.h"
#include "Snapshot.h"
#include "messages.h"
#include "Z80_JLS/z80.h"
#include "pwm_audio.h"

#include "music_file.h"

wav_t Tape::wav;
uint32_t Tape::wav_offset = 0;

uint32_t Tape:: mp3_read = 0;

FIL Tape::tape;
FIL Tape::cswBlock;
string Tape::tapeFileName = "none";
string Tape::tapeSaveName = "none";
int Tape::tapeFileType = TAPE_FTYPE_EMPTY;
uint8_t Tape::tapeStatus = TAPE_STOPPED;
uint8_t Tape::SaveStatus = SAVE_STOPPED;
uint8_t Tape::romLoading = false;
uint8_t Tape::tapeEarBit;
std::vector<TapeBlock> Tape::TapeListing;
int Tape::tapeCurBlock;
int Tape::tapeNumBlocks;
uint32_t Tape::tapebufByteCount;
uint32_t Tape::tapePlayOffset;
size_t Tape::tapeFileSize;

// Tape timing values
uint16_t Tape::tapeSyncLen;
uint16_t Tape::tapeSync1Len;
uint16_t Tape::tapeSync2Len;
uint16_t Tape::tapeBit0PulseLen; // lenght of pulse for bit 0
uint16_t Tape::tapeBit1PulseLen; // lenght of pulse for bit 1
uint16_t Tape::tapeHdrLong;  // Header sync lenght in pulses
uint16_t Tape::tapeHdrShort; // Data sync lenght in pulses
uint32_t Tape::tapeBlkPauseLen; 
uint32_t Tape::tapeNext;
uint8_t Tape::tapeLastByteUsedBits = 8;
uint8_t Tape::tapeEndBitMask;

uint8_t Tape::tapePhase = TAPE_PHASE_STOPPED;

uint8_t Tape::tapeCurByte;
uint64_t Tape::tapeStart;
uint16_t Tape::tapeHdrPulses;
uint32_t Tape::tapeBlockLen;
uint8_t Tape::tapeBitMask;

uint16_t Tape::nLoops;
uint16_t Tape::loopStart;
uint32_t Tape::loop_tapeBlockLen;
uint32_t Tape::loop_tapebufByteCount;
bool Tape::loop_first = false;

uint16_t Tape::callSeq = 0;
int Tape::callBlock;

int Tape::CSW_SampleRate;
int Tape::CSW_PulseLenght;
uint8_t Tape::CSW_CompressionType;
uint32_t Tape::CSW_StoredPulses;

 // GDB vars
uint32_t Tape::totp;
uint8_t Tape::npp;
uint16_t Tape::asp;
uint32_t Tape::totd;
uint8_t Tape::npd;
uint16_t Tape::asd;
uint32_t Tape::curGDBSymbol;
uint8_t Tape::curGDBPulse;
uint8_t Tape::GDBsymbol;
uint8_t Tape::nb;
uint8_t Tape::curBit;
bool Tape::GDBEnd = false;
Symdef* Tape::SymDefTable;

#define my_max(a,b) (((a) > (b)) ? (a) : (b))
#define my_min(a,b) (((a) < (b)) ? (a) : (b))
#define BUF_SIZE 1024

int Tape::inflateCSW(int blocknumber, long startPos, long data_length) {

    char destFileName[16]; // Nombre del archivo descomprimido
    uint8_t s_inbuf[BUF_SIZE];
    uint8_t s_outbuf[BUF_SIZE];
    FIL pOutfile;
    z_stream stream;

    // printf(MOUNT_POINT_SD "/.csw%04d.tmp\n",blocknumber);

    sprintf(destFileName, "/tmp/.csw%04d.tmp", blocknumber);

    // Move to input file compressed data position
    f_lseek(&tape, startPos);

    // Open output file.
    if (FR_OK != f_open(&pOutfile, destFileName, FA_WRITE | FA_CREATE_ALWAYS)) {
        // TODO:
        printf("Failed opening output file!\n");
        return EXIT_FAILURE;
    }

    // Init the z_stream
    memset(&stream, 0, sizeof(stream));
    stream.next_in = s_inbuf;
    stream.avail_in = 0;
    stream.next_out = s_outbuf;
    stream.avail_out = BUF_SIZE;

    // Decompression.
    uint infile_remaining = data_length;

    uint32_t *speccyram = (uint32_t *)MemESP::ram[1].direct();

    VIDEO::SaveRect.store_ram(speccyram, 0x8000);
    MemESP::ram[1].cleanup();
    MemESP::ram[3].cleanup();
    
    if (inflateInit(&stream, MemESP::ram[1].direct())) {
        printf("inflateInit() failed!\n");
        return EXIT_FAILURE;
    }

    for ( ; ; ) {

        int status;
        if (!stream.avail_in) {

            // Input buffer is empty, so read more bytes from input file.
            uint n = my_min(BUF_SIZE, infile_remaining);
            UINT br;
            if (f_read(&tape, s_inbuf, n, &br) != FR_OK || br != n) {
                printf("Failed reading from input file!\n");
                return EXIT_FAILURE;
            }

            stream.next_in = s_inbuf;
            stream.avail_in = n;

            infile_remaining -= n;

        }

        status = inflate(&stream, Z_SYNC_FLUSH);

        if ((status == Z_STREAM_END) || (!stream.avail_out)) {
            // Output buffer is full, or decompression is done, so write buffer to output file.
            uint n = BUF_SIZE - stream.avail_out;
            UINT bw;
            if (f_write(&pOutfile, s_outbuf, n, &bw) != FR_OK && bw != n) {
                printf("Failed writing to output file!\n");
                return EXIT_FAILURE;
            }
            stream.next_out = s_outbuf;
            stream.avail_out = BUF_SIZE;
        }

        if (status == Z_STREAM_END)
            break;
        else if (status != Z_OK) {
            printf("inflate() failed with status %i!\n", status);
            return EXIT_FAILURE;
        }

    }

    if (inflateEnd(&stream) != Z_OK) {
        printf("inflateEnd() failed!\n");
        return EXIT_FAILURE;
    }

    f_close(&pOutfile);

    // printf("Total input bytes: %u\n", (mz_uint32)stream.total_in);
    // printf("Total output bytes: %u\n", (mz_uint32)stream.total_out);
    // printf("Success.\n");

    VIDEO::SaveRect.restore_ram(speccyram, 0x8000);

    return EXIT_SUCCESS;

}

void (*Tape::GetBlock)() = &Tape::TAP_GetBlock;

void StopRealPlayer(void) {
    Config::real_player = false;
#if LOAD_WAV_PIO
    pcm_audio_in_stop();
#endif
}

// Load tape file (.wav, .tap, .tzx)
void Tape::LoadTape(string mFile) {
    if (!FileUtils::fsMount) {
        OSD::osdCenteredMsg(OSD_TAPE_LOAD_ERR, LEVEL_WARN);
        return;
    }
    StopRealPlayer();
    if (FileUtils::hasMP3extension(mFile)) {
        string keySel = mFile.substr(0,1);
        mFile.erase(0, 1);
        Tape::Stop();
        // Read and analyze tap file
        Tape::MP3_Open(mFile);
        ESPectrum::TapeNameScroller = 0;
        Tape::Play();
    } else if (FileUtils::hasWAVextension(mFile)) {
        string keySel = mFile.substr(0,1);
        mFile.erase(0, 1);
        Tape::Stop();
        // Read and analyze tap file
        Tape::WAV_Open(mFile);
        ESPectrum::TapeNameScroller = 0;
        Tape::Play();
    } else if (FileUtils::hasTAPextension(mFile)) {
        string keySel = mFile.substr(0,1);
        mFile.erase(0, 1);
        // Flashload .tap if needed
        if ((keySel == "R") && (Config::flashload) && (Config::arch != "ALF") &&
             (Config::romSet != "ZX81+") && (Config::romSet != "48Kcs") && (Config::romSet != "128Kcs")
        ) {
                OSD::osdCenteredMsg(OSD_TAPE_FLASHLOAD, LEVEL_INFO, 100);
                uint8_t OSDprev = VIDEO::OSD;
                if (Z80Ops::is48)
                    FileZ80::loader48();
                else
                    FileZ80::loader128();
                // Put something random on FRAMES SYS VAR as recommended by Mark Woodmass
                // https://skoolkid.github.io/rom/asm/5C78.html
                MemESP::writebyte(0x5C78,rand() % 256);
                MemESP::writebyte(0x5C79,rand() % 256);            

                if (Config::ram_file != NO_RAM_FILE) {
                    Config::ram_file = NO_RAM_FILE;
                }
                Config::last_ram_file = NO_RAM_FILE;

                if (OSDprev) {
                    VIDEO::OSD = OSDprev;
                    if (Config::aspect_16_9)
                        VIDEO::Draw_OSD169 = VIDEO::MainScreen_OSD;
                    else
                        VIDEO::Draw_OSD43  = Z80Ops::isPentagon ? VIDEO::BottomBorder_OSD_Pentagon : VIDEO::BottomBorder_OSD;
                    ESPectrum::TapeNameScroller = 0;
                }    
        }
        Tape::Stop();
        // Read and analyze tap file
        Tape::TAP_Open(mFile);
        ESPectrum::TapeNameScroller = 0;
    } else if (FileUtils::hasTZXextension(mFile)) {
        string keySel = mFile.substr(0,1);
        mFile.erase(0, 1);
        Tape::Stop();
        // Read and analyze tzx file
        Tape::TZX_Open(mFile);
        ESPectrum::TapeNameScroller = 0;
        // printf("%s loaded.\n",mFile.c_str());
    }
    else {
        OSD::osdCenteredMsg(OSD_TAPE_LOAD_ERR, LEVEL_WARN);
    }
}

void Tape::Init() {
    f_close(&tape);
    tapeFileType = TAPE_FTYPE_EMPTY;
}

typedef struct INFO {
    char INFO[4];
} INFO_t;

typedef struct info {
    char info_id[4];
    uint32_t size;
} info_t;

typedef struct info_desc {
    const char FORB[5];
    const char* desc;
} info_desc_t;

static const info_desc_t info_descs[22] = {
 { "IARL",	"The location where the subject of the file is archived" },
 { "IART",	"The artist of the original subject of the file" },
 { "ICMS",	"The name of the person or organization that commissioned the original subject of the file" },
 { "ICMT",	"General comments about the file or its subject" },
 { "ICOP",	"Copyright information about the file" },
 { "ICRD",	"The date the subject of the file was created" },
 { "ICRP",	"Whether and how an image was cropped" },
 { "IDIM",	"The dimensions of the original subject of the file" },
 { "IDPI",	"Dots per inch settings used to digitize the file" },
 { "IENG",	"The name of the engineer who worked on the file" },
 { "IGNR",	"The genre of the subject" },
 { "IKEY",	"A list of keywords for the file or its subject" },
 { "ILGT",	"Lightness settings used to digitize the file" },
 { "IMED",	"Medium for the original subject of the file" },
 { "INAM",	"Title of the subject of the file (name)" },
 { "IPLT",	"The number of colors in the color palette used to digitize the file" },
 { "IPRD",	"Name of the title the subject was originally intended for" },
 { "ISB",	"Description of the contents of the file (subject)" },
 { "ISFT",	"Name of the software package used to create the file" },
 { "ISRC",	"The name of the person or organization that supplied the original subject of the file" },
 { "ISRF",	"The original form of the material that was digitized (source form)" },
 { "ITCH",	"The name of the technician who digitized the subject file" }
};


void Tape::WAV_Open(string name) {
    f_close(&tape);
    tapeFileType = TAPE_FTYPE_EMPTY;
    string fname = FileUtils::TAP_Path + name;
    if (f_open(&tape, fname.c_str(), FA_READ) != FR_OK) {
        OSD::osdCenteredMsg(OSD_TAPE_LOAD_ERR "\n" + fname + "\n", LEVEL_ERROR);
        return;
    }
    tapeFileSize = f_size(&tape);
    if (tapeFileSize == 0) return;
    
    tapeFileName = name;

    UINT rb;
    if (f_read(&tape, &wav, sizeof(wav_t), &rb) != FR_OK) {
        OSD::osdCenteredMsg(OSD_TAPE_LOAD_ERR "\n" + fname + "\n", LEVEL_ERROR);
        return;
    }
    if (strncmp("RIFF", wav.RIFF, 4) != 0 || strncmp("WAVEfmt ", wav.WAVEfmt, 8) != 0 || wav.h_size != 16) {
        OSD::osdCenteredMsg("Unexpected file header\n" + fname + "\n", LEVEL_ERROR);
        return;
    }
    if (wav.pcm != 1) {
        OSD::osdCenteredMsg("Unsupported file type (not PCM)\n" + fname + "\n", LEVEL_ERROR);
        return;
    }
    if (wav.ch != 1 && wav.ch != 2) {
        OSD::osdCenteredMsg("Unsupported number of chanels\n" + fname + "\n", LEVEL_ERROR);
        return;
    }
    if (wav.bit_per_sample != 8 && wav.bit_per_sample != 16) {
        OSD::osdCenteredMsg("Unsupported bitness\n" + fname + "\n", LEVEL_ERROR);
        return;
    }
    if (strncmp(wav.data, "LIST", 4) == 0) {
        char* sch = new char[wav.subchunk_size];
        size_t size;
        if (f_read(&tape, sch, wav.subchunk_size, &size) != FR_OK || size != wav.subchunk_size) {
            OSD::osdCenteredMsg("Unexpected end of file\n" + fname + "\n", LEVEL_ERROR);
            delete sch;
            return;
        }
        INFO_t* ch = (INFO_t*)sch;
        if (strncmp(ch->INFO, "INFO", 4) != 0) {
            OSD::osdCenteredMsg("Unexpected LIST section in the file\n" + fname + "\n", LEVEL_ERROR);
            delete sch;
            return;
        }
        delete sch;
    }
    tapeFileType = TAPE_FTYPE_WAV;
    wav_offset = f_tell(&tape);
    tapePlayOffset = wav_offset; // initial offset
    tapeNumBlocks = 1; // one huge block
}

#define WORKING_SIZE        4000
///16000
#define RAM_BUFFER_LENGTH   2000
///6000

static unsigned char* working = 0;;
static music_file* mf = 0;
static int16_t* d_buff = 0;

extern "C" void osd_printf(const char* msg, ...);

struct free_ptr {
    uint8_t* p;
    size_t off;
};

static vector<free_ptr> free_ptrs;

extern "C" void* malloc2(size_t sz) {
	void* res = 0;
    for (auto it = free_ptrs.begin(); it != free_ptrs.end(); ++it) {
        free_ptr& fp = *it;
        if (sz + fp.off <= (16 << 10)) {
            res = fp.p + fp.off;
            fp.off += sz;
            break;
        }
    }
	if (!res) {
        char buf[16];
		snprintf(buf, 16, "E %d\n", sz);
		osd_printf(buf);
	}
	return res;
}

static bool revoke_ram_4_mp3 = false;

bool writeWavHeader(FIL* fo, uint32_t sample_rate, uint16_t num_channels)
{
    uint32_t val32;
    uint16_t val16;
    UINT written;

    // 0 RIFF
    val32 = 0x46464952;
    if ((f_write(fo, &val32, sizeof(uint32_t), &written) != FR_OK) &&
        (written != sizeof(uint32_t)))
    {
        return false;
    }

    // 4 cksize - to be written at end
    val32 = 0;
    if ((f_write(fo, &val32, sizeof(uint32_t), &written) != FR_OK) &&
        (written != sizeof(uint32_t)))
    {
        return false;
    }

    // 8 WAVE
    val32 = 0x45564157;
    if ((f_write(fo, &val32, sizeof(uint32_t), &written) != FR_OK) &&
        (written != sizeof(uint32_t)))
    {
        return false;
    }

    // 12 fmt
    val32 = 0x20746d66;
    if ((f_write(fo, &val32, sizeof(uint32_t), &written) != FR_OK) &&
        (written != sizeof(uint32_t)))
    {
        return false;
    }

    // 16 Subchunk1Size
    val32 = 16;
    if ((f_write(fo, &val32, sizeof(uint32_t), &written) != FR_OK) &&
        (written != sizeof(uint32_t)))
    {
        return false;
    }

    // 20 Audio format - PCM
    val16 = 1;
    if ((f_write(fo, &val16, sizeof(uint16_t), &written) != FR_OK) &&
        (written != sizeof(uint16_t)))
    {
        return false;
    }

    // 22 Number of channels
    if ((f_write(fo, &num_channels, sizeof(uint16_t), &written) != FR_OK) &&
        (written != sizeof(uint16_t)))
    {
        return false;
    }

    // 24 Sample rate
    if ((f_write(fo, &sample_rate, sizeof(uint32_t), &written) != FR_OK) &&
        (written != sizeof(uint32_t)))
    {
        return false;
    }

    // 28 Byte rate per sec = num_channel * sample_rate * sample_size_in_bytes
    val32 = sample_rate * num_channels; /// * 2;
    if ((f_write(fo, &val32, sizeof(uint32_t), &written) != FR_OK) &&
        (written != sizeof(uint32_t)))
    {
        return false;
    }

    // 32 Block alignment
    val16 = (uint16_t)(num_channels); /// * 2);
    if ((f_write(fo, &val16, sizeof(uint16_t), &written) != FR_OK) &&
        (written != sizeof(uint16_t)))
    {
        return false;
    }

    // 34 Bits per sample
    val16 = 8; ///16;
    if ((f_write(fo, &val16, sizeof(uint16_t), &written) != FR_OK) &&
        (written != sizeof(uint16_t)))
    {
        return false;
    }

    // 36 data
    val32 = 0x61746164;
    if ((f_write(fo, &val32, sizeof(uint32_t), &written) != FR_OK) &&
        (written != sizeof(uint32_t)))
    {
        return false;
    }

    // 40 Size of data - write 0, then update at end
    val32 = 0;
    if ((f_write(fo, &val32, sizeof(uint32_t), &written) != FR_OK) &&
        (written != sizeof(uint32_t)))
    {
        return false;
    }

    return true;
}

bool updateWavHeader(FIL* fo, uint32_t num_samples, uint16_t num_channels)
{
    uint32_t val32;
    UINT written;

    if (f_lseek(fo, 4) != FR_OK)
        return false;

    val32 = 36 + num_samples * num_channels; /// * sizeof(int16_t);
    if ((f_write(fo, &val32, sizeof(uint32_t), &written) != FR_OK) &&
        (written != sizeof(uint32_t)))
    {
        return false;
    }

    if (f_lseek(fo, 40) != FR_OK)
        return false;

    val32 = num_samples * num_channels; /// * sizeof(int16_t);
    if ((f_write(fo, &val32, sizeof(uint32_t), &written) != FR_OK) &&
        (written != sizeof(uint32_t)))
    {
        return false;
    }

    return true;
}

void Tape::MP3_Open(string name) {
    f_close(&tape);
    tapeFileType = TAPE_FTYPE_EMPTY;
    string fname = FileUtils::TAP_Path + name;
    tapeFileName = fname + ".wav";
    if (f_open(&tape, tapeFileName.c_str(), FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
        OSD::osdCenteredMsg(OSD_TAPE_LOAD_ERR "\n" + tapeFileName + "\n", LEVEL_ERROR);
    }

    if (!revoke_ram_4_mp3) {
        for (size_t i = 0; i < 2; ++i) {
            uint8_t* p = mem_desc_t::revoke_1_ram_page();
            if (p == 0) continue;
            free_ptr fp = { p , 0 };
            free_ptrs.push_back(fp);
        }
    }
    working = (uint8_t*) malloc2(WORKING_SIZE);
    d_buff = (int16_t*) malloc2(RAM_BUFFER_LENGTH * 2);
    mf = new music_file();

    if (!musicFileCreate(mf, fname.c_str(), working, WORKING_SIZE)) {
        OSD::osdCenteredMsg(OSD_TAPE_LOAD_ERR "\n" + fname + "\n", LEVEL_ERROR);
        delete mf;
        mf = 0;
        d_buff = 0;
        working = 0;
        for (auto it = free_ptrs.begin(); it != free_ptrs.end(); ++it)  {
            it->off = 0;
        }
        return;
    }

    FSIZE_t fz = f_size(&mf->fil);
    FSIZE_t lp = 0;
    uint32_t sample_rate = musicFileGetSampleRate(mf);
    uint16_t num_channels =  musicFileGetChannels(mf);
    writeWavHeader(&tape, sample_rate, num_channels);
    bool success = true;
    UINT num_samples, written = 0;
    OSD::progressDialog("Convert mp3 to wav", tapeFileName, 0, 0);
    do {
        success = musicFileRead(mf, d_buff, RAM_BUFFER_LENGTH, &mp3_read);
        FSIZE_t ip = f_tell(&mf->fil);
        if (ip < lp) break;
        lp = ip;
        OSD::progressDialog("Convert mp3 to wav", tapeFileName, lp * 100 / fz, 1);
        if (success && mp3_read) {
            num_samples += mp3_read / num_channels;
            int8_t* t = (int8_t*)d_buff;
            for (size_t i = 0; i < mp3_read; ++i) {
                t[i] = d_buff[i] >> 8;
            }
            if ((f_write(&tape, d_buff, mp3_read, &written) != FR_OK) || (written != mp3_read)) {
                osd_printf("Error in f_write\n");
                success = false;
            }
        } else {
            if (!success) {
                osd_printf("Error in mp3FileRead\n");
            } else {
                osd_printf("Convert to WAV passed\n");
            }
        }
    } while (success && mp3_read) ; /// && (total_dec_time / GetClockFrequency() < stop_time));
    OSD::progressDialog("Convert mp3 to wav", tapeFileName, 100, 2);
    OSD::osdCenteredMsg(
        "sample_rate: " + to_string(sample_rate) +
        "\nchannels: " + to_string(num_channels) +
        "; Press F6...\n",
        LEVEL_WARN,
        1000
    );
    if (success) {
        updateWavHeader(&tape, num_samples, num_channels);
    }
    if (mf) {
        musicFileClose(mf);
        delete mf;
        mf = 0;
        d_buff = 0;
        working = 0;
    }
    for (auto it = free_ptrs.begin(); it != free_ptrs.end(); ++it)  {
        it->off = 0;
    }
    WAV_Open(name + ".wav");
/*
    tapeFileName = name;
    tapeFileType = TAPE_FTYPE_MP3;
    tapePlayOffset = mf->file_offset;
    tapeNumBlocks = 1; // one huge block
*/
}

void Tape::TAP_Open(string name) {
    f_close(&tape);
    tapeFileType = TAPE_FTYPE_EMPTY;
    string fname = FileUtils::TAP_Path + name;
    if (f_open(&tape, fname.c_str(), FA_READ) != FR_OK) {
        OSD::osdCenteredMsg(OSD_TAPE_LOAD_ERR "\n" + fname + "\n", LEVEL_ERROR);
        return;
    }
    tapeFileSize = f_size(&tape);
    if (tapeFileSize == 0) return;
    
    tapeFileName = name;

    Tape::TapeListing.clear(); // Clear TapeListing vector
    std::vector<TapeBlock>().swap(TapeListing); // free memory

    int tapeListIndex = 0;
    int tapeContentIndex = 0;
    int tapeBlkLen = 0;
    TapeBlock block;
    FIL* tape = &Tape::tape;
    do {
        // Analyze .tap file
        tapeBlkLen = (readByteFile(tape) | (readByteFile(tape) << 8));

        // printf("Analyzing block %d\n",tapeListIndex);
        // printf("    Block Len: %d\n",tapeBlockLen - 2);        

        // Read the flag byte from the block.
        // If the last block is a fragmented data block, there is no flag byte, so set the flag to 255
        // to indicate a data block.
        uint8_t flagByte;
        if (tapeContentIndex + 2 < tapeFileSize) {
            flagByte = readByteFile(tape);
        } else {
            flagByte = 255;
        }

        // Process the block depending on if it is a header or a data block.
        // Block type 0 should be a header block, but it happens that headerless blocks also
        // have block type 0, so we need to check the block length as well.
        if (flagByte == 0 && tapeBlkLen == 19) { // This is a header.

            // Get the block type.
            TapeBlock::BlockType dataBlockType;
            uint8_t blocktype = readByteFile(tape);
            switch (blocktype)
            {
                case 0:
                    dataBlockType = TapeBlock::BlockType::Program_header;
                    break;
                case 1:
                    dataBlockType = TapeBlock::BlockType::Number_array_header;
                    break;
                case 2:
                    dataBlockType = TapeBlock::BlockType::Character_array_header;
                    break;
                case 3:
                    dataBlockType = TapeBlock::BlockType::Code_header;
                    break;
                default:
                    dataBlockType = TapeBlock::BlockType::Unassigned;
                    break;
            }

            // Get the filename.
            for (int i = 0; i < 10; i++) {
                uint8_t tst = readByteFile(tape);
            }

            f_lseek(tape, f_tell(tape) + 6);

            // Get the checksum.
            uint8_t checksum = readByteFile(tape);
        
            if ((tapeListIndex & (TAPE_LISTING_DIV - 1)) == 0) {
                block.StartPosition = tapeContentIndex;
                TapeListing.push_back(block);
            }

        } else {

            // Get the block content length.
            int contentLength;
            int contentOffset;
            if (tapeBlkLen >= 2) {
                // Normally the content length equals the block length minus two
                // (the flag byte and the checksum are not included in the content).
                contentLength = tapeBlkLen - 2;
                // The content is found at an offset of 3 (two byte block size + one flag byte).
                contentOffset = 3;
            } else {
                // Fragmented data doesn't have a flag byte or a checksum.
                contentLength = tapeBlkLen;
                // The content is found at an offset of 2 (two byte block size).
                contentOffset = 2;
            }

            f_lseek(tape, f_tell(tape) + contentLength);

            // Get the checksum.
            uint8_t checksum = readByteFile(tape);

            if ((tapeListIndex & (TAPE_LISTING_DIV - 1)) == 0) {
                block.StartPosition = tapeContentIndex;
                TapeListing.push_back(block);
            }

        }

        tapeListIndex++;
        
        tapeContentIndex += tapeBlkLen + 2;

    } while(tapeContentIndex < tapeFileSize);

    tapeCurBlock = 0;
    tapeNumBlocks = tapeListIndex;

    f_lseek(tape, 0);

    tapeFileType = TAPE_FTYPE_TAP;

    // Set tape timing values
    if (Config::tape_timing_rg) {

        tapeSyncLen = TAPE_SYNC_LEN_RG;
        tapeSync1Len = TAPE_SYNC1_LEN_RG;
        tapeSync2Len = TAPE_SYNC2_LEN_RG;
        tapeBit0PulseLen = TAPE_BIT0_PULSELEN_RG;
        tapeBit1PulseLen = TAPE_BIT1_PULSELEN_RG;
        tapeHdrLong = TAPE_HDR_LONG_RG;
        tapeHdrShort = TAPE_HDR_SHORT_RG;
        tapeBlkPauseLen = TAPE_BLK_PAUSELEN_RG; 

    } else {

        tapeSyncLen = TAPE_SYNC_LEN;
        tapeSync1Len = TAPE_SYNC1_LEN;
        tapeSync2Len = TAPE_SYNC2_LEN;
        tapeBit0PulseLen = TAPE_BIT0_PULSELEN;
        tapeBit1PulseLen = TAPE_BIT1_PULSELEN;
        tapeHdrLong = TAPE_HDR_LONG;
        tapeHdrShort = TAPE_HDR_SHORT;
        tapeBlkPauseLen = TAPE_BLK_PAUSELEN; 

    }

}

uint32_t Tape::CalcTapBlockPos(int block) {

    int TapeBlockRest = block & (TAPE_LISTING_DIV -1);
    int CurrentPos = TapeListing[block / TAPE_LISTING_DIV].StartPosition;
    // printf("TapeBlockRest: %d\n",TapeBlockRest);
    // printf("Tapecurblock: %d\n",Tape::tapeCurBlock);

    f_lseek(&tape, CurrentPos);

    while (TapeBlockRest-- != 0) {
        uint16_t tapeBlkLen=(readByteFile(&tape) | (readByteFile(&tape) << 8));
        // printf("Tapeblklen: %d\n",tapeBlkLen);
        f_lseek(&tape, f_tell(&tape) + tapeBlkLen);
        CurrentPos += tapeBlkLen + 2;
    }

    return CurrentPos;

}

string Tape::tapeBlockReadData(int Blocknum) {

    int tapeContentIndex=0;
    int tapeBlkLen=0;
    string blktype;
    char buf[48];
    char fname[10];

    tapeContentIndex = Tape::CalcTapBlockPos(Blocknum);
    FIL* tape = &Tape::tape;
    // Analyze .tap file
    tapeBlkLen=(readByteFile(tape) | (readByteFile(tape) << 8));

    // Read the flag byte from the block.
    // If the last block is a fragmented data block, there is no flag byte, so set the flag to 255
    // to indicate a data block.
    uint8_t flagByte;
    if (tapeContentIndex + 2 < Tape::tapeFileSize) {
        flagByte = readByteFile(tape);
    } else {
        flagByte = 255;
    }

    // Process the block depending on if it is a header or a data block.
    // Block type 0 should be a header block, but it happens that headerless blocks also
    // have block type 0, so we need to check the block length as well.
    if (flagByte == 0 && tapeBlkLen == 19) { // This is a header.

        // Get the block type.
        uint8_t blocktype = readByteFile(tape);

        switch (blocktype) {
        case 0: 
            blktype = "Program      ";
            break;
        case 1: 
            blktype = "Number array ";
            break;
        case 2: 
            blktype = "Char array   ";
            break;
        case 3: 
            blktype = "Code         ";
            break;
        case 4: 
            blktype = "Data block   ";
            break;
        case 5: 
            blktype = "Info         ";
            break;
        case 6: 
            blktype = "Unassigned   ";
            break;
        default:
            blktype = "Unassigned   ";
            break;
        }

        // Get the filename.
        if (blocktype > 5) {
            fname[0] = '\0';
        } else {
            for (int i = 0; i < 10; i++) {
                fname[i] = readByteFile(tape);
            }
            fname[9]='\0';
        }
    } else {
        blktype = "Data block   ";
        fname[0]='\0';
    }
    snprintf(buf, sizeof(buf), "%04d %s %10s % 6d\n", Blocknum + 1, blktype.c_str(), fname, tapeBlkLen);
    return buf;
}

void Tape::Play() {

    if (tapeFileType != TAPE_FTYPE_MP3 && !tape.obj.fs) {
        OSD::osdCenteredMsg(OSD_TAPE_LOAD_ERR, LEVEL_ERROR);
        return;
    }

    if (VIDEO::OSD) VIDEO::OSD = 1;

    // Prepare current block to play
    switch(tapeFileType) {
        case TAPE_FTYPE_TAP:
            tapePlayOffset = CalcTapBlockPos(tapeCurBlock);
            GetBlock = &TAP_GetBlock;
            break;
        case TAPE_FTYPE_TZX:
            tapePlayOffset = CalcTZXBlockPos(tapeCurBlock);
            GetBlock = &TZX_GetBlock;
            break;
        case TAPE_FTYPE_WAV:
            tapePlayOffset = tapeStart;
            GetBlock = &WAV_GetBlock;
            break;
        case TAPE_FTYPE_MP3:
            tapePlayOffset = tapeStart; /// TODO:
            GetBlock = &MP3_GetBlock;
            break;
    }

    // Init tape vars
    tapeEarBit = 1;
    tapeBitMask = 0x80;
    tapeLastByteUsedBits = 8;
    tapeEndBitMask = 0x80;
    tapeBlockLen = 0;
    tapebufByteCount = 0;
    GDBEnd = false;

    // Get block data
    tapeCurByte = readByteFile(&tape);
    GetBlock();

    // Start loading
    Tape::tapeStatus = TAPE_LOADING;
    tapeStart = CPU::global_tstates + CPU::tstates;
}

void Tape::WAV_GetBlock() {
}

void Tape::MP3_GetBlock() {
}

void Tape::TAP_GetBlock() {

    // Check end of tape
    if (tapeCurBlock >= tapeNumBlocks) {
        tapeCurBlock = 0;
        Stop();
        f_lseek(&tape, 0);
        return;
    }

    // Get block len and first byte of block
    tapeBlockLen += (tapeCurByte | (readByteFile(&tape) << 8)) + 2;
    tapeCurByte = readByteFile(&tape);
    tapebufByteCount += 2;

    // Set sync phase values
    tapePhase = TAPE_PHASE_SYNC;
    tapeNext = tapeSyncLen;
    if (tapeCurByte) tapeHdrPulses = tapeHdrShort;
    else tapeHdrPulses = tapeHdrLong;
}

void Tape::Stop() {
    OSD::osdCenteredMsg("Tape loading is stopped", LEVEL_INFO, 100);
    tapeStatus = TAPE_STOPPED;
    tapePhase = TAPE_PHASE_STOPPED;
    if (VIDEO::OSD) {
        VIDEO::OSD = 2;
    }
}

IRAM_ATTR void Tape::Read() {
#if LOAD_WAV_PIO
    if ( tapeFileType == TAPE_FTYPE_EMPTY && Config::real_player ) {
        tapeEarBit = pcm_data_in();
        return;
    }
#endif
    uint64_t tapeCurrent = CPU::global_tstates + CPU::tstates - tapeStart; // states since start
    FIL* tape = &Tape::tape;
    if ( tapeFileType == TAPE_FTYPE_MP3 ) {
        if (!mf) return;
        uint32_t FPS = 50;
        uint32_t samplesPerFrame = musicFileGetSampleRate(mf) / FPS; // samples/second / frame/second; ~44100 / 50 = 882
        uint32_t statesPerSample = CPU::statesInFrame / samplesPerFrame; // states/frame / samples/frame; ~70000 / 882 = 79
        FSIZE_t sampleNumber = sampleNumber = tapeCurrent * (musicFileIsStereo(mf) ? 2 : 1) / statesPerSample;
        // + wav_offset; // states / states/sample
        size_t t = mp3_read;
        if (sampleNumber < mp3_read) {
            int16_t v = d_buff[sampleNumber];
            tapeEarBit = v > 0 ? 1 : 0;
//                    tapePlayOffset = sampleNumber;
        } else if (!musicFileRead(mf, d_buff, RAM_BUFFER_LENGTH, &mp3_read)) {
            Stop();
            musicFileClose(mf);
        } else {
            tapePlayOffset = mf->file_offset;
            sampleNumber -= t;
            if (sampleNumber < mp3_read) {
                int16_t v = d_buff[sampleNumber];
                tapeEarBit = v > 0 ? 1 : 0;
            }
        }
        tapeStart = CPU::global_tstates + CPU::tstates - tapeCurrent; // recover?
        return;
    }
    else if ( tapeFileType == TAPE_FTYPE_WAV ) {
        uint32_t FPS = 50; /// VIDEO::framecnt / (ESPectrum::totalseconds / 1000000); // ~50 fps
        uint32_t samplesPerFrame = wav.freq / FPS; // samples/second / frame/second; ~44100 / 50 = 882
        uint32_t statesPerSample = CPU::statesInFrame / samplesPerFrame; // states/frame / samples/frame; ~70000 / 882 = 79
        FSIZE_t sampleNumber;
        if (wav.ch == 1) { // mono
            if (wav.byte_per_sample == 1) { // 8-bit
                sampleNumber = tapeCurrent / statesPerSample + wav_offset; // states / states/sample
            } else { // 2 bytes per sample
                sampleNumber = tapeCurrent * 2 / statesPerSample + wav_offset + 1; // states / states/sample
            }
        } else { // 2 channels
            if (wav.byte_per_sample == 2) { // 8-bit per channel
                sampleNumber = tapeCurrent * 2 / statesPerSample + wav_offset; // states / states/sample
            } else  { // 16-bit
                sampleNumber = tapeCurrent * 4 / statesPerSample + wav_offset + 1; // states / states/sample
            }
        }
        if (tapeFileSize >= sampleNumber) {
            f_lseek(tape, sampleNumber);
            int8_t v = readByteFile(tape);
            tapeEarBit = v > 0 ? 1 : 0;
            tapePlayOffset = sampleNumber;
        } else {
            Stop();
            f_lseek(tape, 0);
        }
        tapeStart = CPU::global_tstates + CPU::tstates - tapeCurrent; // recover?
        return;
    }
    if (tapeCurrent >= tapeNext) {
        do {
            tapeCurrent -= tapeNext;
            switch (tapePhase) {
            case TAPE_PHASE_CSW:
                tapeEarBit ^= 1;
                if (CSW_CompressionType == 1) { // RLE
                    CSW_PulseLenght = readByteFile(tape);
                    tapebufByteCount++;                
                    if (tapebufByteCount == tapeBlockLen) {
                        tapeCurByte = CSW_PulseLenght;
                        if (tapeBlkPauseLen == 0) {
                            tapeCurBlock++;
                            GetBlock();
                        } else {
                            tapePhase = TAPE_PHASE_TAIL;
                            tapeNext  = TAPE_PHASE_TAIL_LEN;
                        }
                        break;
                    }
                    if (CSW_PulseLenght == 0) {
                        CSW_PulseLenght = readByteFile(tape) | (readByteFile(tape) << 8) | (readByteFile(tape) << 16) | (readByteFile(tape) << 24);
                        tapebufByteCount += 4;
                    }                
                    tapeNext = CSW_SampleRate * CSW_PulseLenght;
                } else { // Z-RLE
                    CSW_PulseLenght = readByteFile(&cswBlock);
                    if (f_eof(&cswBlock)) {
                        f_close(&cswBlock);
                        tapeCurByte = readByteFile(tape);
                        if (tapeBlkPauseLen == 0) {
                            tapeCurBlock++;
                            GetBlock();
                        } else {
                            tapePhase = TAPE_PHASE_TAIL;
                            tapeNext  = TAPE_PHASE_TAIL_LEN;
                        }
                        break;
                    }
                    if (CSW_PulseLenght == 0) {
                        CSW_PulseLenght = readByteFile(&cswBlock) | (readByteFile(&cswBlock) << 8)
                                      | (readByteFile(&cswBlock) << 16) | (readByteFile(&cswBlock) << 24);
                    }                
                    tapeNext = CSW_SampleRate * CSW_PulseLenght;
                }
                break;
            case TAPE_PHASE_GDB_PILOTSYNC:

                // Get next pulse lenght from current symbol
                if (++curGDBPulse < npp)
                    tapeNext = SymDefTable[GDBsymbol].PulseLenghts[curGDBPulse];

                if (tapeNext == 0 || curGDBPulse == npp) {

                    // printf("curGDBPulse: %d, npp: %d\n",(int)curGDBPulse,(int)npp);

                    // Next repetition
                    if (--tapeHdrPulses == 0) {
                        
                        // Get next symbol in PRLE
                        curGDBSymbol++;

                        if (curGDBSymbol < totp) { // If not end of PRLE

                            // Read pulse data
                            GDBsymbol = readByteFile(tape); // Read Symbol to be represented from PRLE

                            // Get symbol flags
                            switch (SymDefTable[GDBsymbol].SymbolFlags) {
                                case 0:
                                    tapeEarBit ^= 1;
                                    break;
                                case 1:
                                    break;                                    
                                case 2:
                                    tapeEarBit = 0;
                                    break;
                                case 3:
                                    tapeEarBit = 1;
                                    break;
                            }

                            // Get first pulse lenght from array of pulse lenghts
                            tapeNext = SymDefTable[GDBsymbol].PulseLenghts[0];

                            // Get number of repetitions from PRLE[0]
                            tapeHdrPulses = readByteFile(tape) | (readByteFile(tape) << 8); // Number of repetitions of symbol
                            
                            curGDBPulse = 0;

                            tapebufByteCount += 3;

                        } else {
                            
                            // End of PRLE

                            // Free SymDefTable
                            for (int i = 0; i < asp; i++)
                                delete[] SymDefTable[i].PulseLenghts;
                            delete[] SymDefTable;

                            // End of pilotsync. Is there data stream ?
                            if (totd > 0) {

                                // printf("\nPULSES (DATA)\n");

                                // Allocate memory for the array of pointers to struct Symdef
                                SymDefTable = new Symdef[asd];

                                // Allocate memory for each row
                                for (int i = 0; i < asd; i++) {
                                    // Initialize each element in the row
                                    SymDefTable[i].SymbolFlags = readByteFile(tape);
                                    tapebufByteCount += 1;
                                    SymDefTable[i].PulseLenghts = new uint16_t[npd];
                                    for(int j = 0; j < npd; j++) {
                                        SymDefTable[i].PulseLenghts[j] = readByteFile(tape) | (readByteFile(tape) << 8);
                                        tapebufByteCount += 2;
                                    }

                                }

                                // printf("-----------------------\n");
                                // printf("Data Sync Symbol Table\n");
                                // printf("Asd: %d, Npd: %d\n",asd,npd);
                                // printf("-----------------------\n");
                                // for (int i = 0; i < asd; i++) {
                                //     printf("%d: %d; ",i,(int)SymDefTable[i].SymbolFlags);
                                //     for (int j = 0; j < npd; j++) {
                                //         printf("%d,",(int)SymDefTable[i].PulseLenghts[j]);
                                //     }
                                //     printf("\n");
                                // }
                                // printf("-----------------------\n");

                                // printf("END DATA SYMBOL TABLE GDB -> tapeCurByte: %d, Tape pos: %d, Tapebbc: %d\n", tapeCurByte,(int)(ftell(tape)),tapebufByteCount);

                                curGDBSymbol = 0;
                                curGDBPulse = 0;
                                curBit = 7;

                                // Read data stream first symbol
                                GDBsymbol = 0;

                                tapeCurByte = readByteFile(tape);
                                tapebufByteCount += 1;

                                // printf("tapeCurByte: %d, nb:%d\n", (int)tapeCurByte,(int)nb);

                                for (int i = nb; i > 0; i--) {
                                    GDBsymbol <<= 1;
                                    GDBsymbol |= ((tapeCurByte >> (curBit)) & 0x01);
                                    if (curBit == 0) {
                                        tapeCurByte = readByteFile(tape);
                                        tapebufByteCount += 1;
                                        curBit = 7;
                                    } else
                                        curBit--;
                                }
                                
                                // Get symbol flags
                                switch (SymDefTable[GDBsymbol].SymbolFlags) {
                                case 0:
                                    tapeEarBit ^= 1;
                                    break;
                                case 1:
                                    break;                                    
                                case 2:
                                    tapeEarBit = 0;
                                    break;
                                case 3:
                                    tapeEarBit = 1;
                                    break;
                                }

                                // Get first pulse lenght from array of pulse lenghts
                                tapeNext = SymDefTable[GDBsymbol].PulseLenghts[0];

                                tapePhase = TAPE_PHASE_GDB_DATA;

                                // printf("PULSE%d %d Flags: %d\n",tapeEarBit,tapeNext,(int)SymDefTable[GDBsymbol].SymbolFlags);

                                // printf("Curbit: %d, GDBSymbol: %d, Flags: %d, tapeNext: %d\n",(int)curBit,(int)GDBsymbol,(int)(SymDefTable[GDBsymbol].SymbolFlags & 0x3),(int)tapeNext);

                            } else {

                                tapeCurByte = readByteFile(tape);
                                tapeEarBit ^= 1;
                                tapePhase=TAPE_PHASE_TAIL_GDB;
                                tapeNext = TAPE_PHASE_TAIL_LEN_GDB;

                            }

                        }

                    } else {

                        // Modify tapeearbit according to symbol flags
                        switch (SymDefTable[GDBsymbol].SymbolFlags) {
                            case 0:
                                tapeEarBit ^= 1;
                                break;
                            case 1:
                                break;                                    
                            case 2:
                                tapeEarBit = 0;
                                break;
                            case 3:
                                tapeEarBit = 1;
                                break;
                        }

                        tapeNext = SymDefTable[GDBsymbol].PulseLenghts[0];

                        curGDBPulse = 0;

                    }

                } else {

                    tapeEarBit ^= 1;

                }

                break;
            
            case TAPE_PHASE_GDB_DATA:

                // Get next pulse lenght from current symbol
                if (++curGDBPulse < npd)
                    tapeNext = SymDefTable[GDBsymbol].PulseLenghts[curGDBPulse];

                if (curGDBPulse == npd || tapeNext == 0) {

                    // Get next symbol in data stream
                    curGDBSymbol++;

                    if (curGDBSymbol < totd) { // If not end of data stream

                        // Read data stream next symbol
                        GDBsymbol = 0;

                        // printf("tapeCurByte: %d, NB: %d, ", tapeCurByte,nb);

                        for (int i = nb; i > 0; i--) {
                            GDBsymbol <<= 1;
                            GDBsymbol |= ((tapeCurByte >> (curBit)) & 0x01);
                            if (curBit == 0) {
                                tapeCurByte = readByteFile(tape);
                                tapebufByteCount += 1;
                                curBit = 7;
                            } else
                                curBit--;
                        }

                        // Get symbol flags
                        switch (SymDefTable[GDBsymbol].SymbolFlags) {
                            case 0:
                                tapeEarBit ^= 1;
                                break;
                            case 1:
                                break;                                    
                            case 2:
                                tapeEarBit = 0;
                                break;
                            case 3:
                                tapeEarBit = 1;
                                break;
                        }

                        // Get first pulse lenght from array of pulse lenghts
                        tapeNext = SymDefTable[GDBsymbol].PulseLenghts[0];

                        curGDBPulse = 0;

                    } else {

                        // Needed Adjustment
                        tapebufByteCount--;

                        // printf("END DATA GDB -> tapeCurByte: %d, Tape pos: %d, Tapebbc: %d, TapeBlockLen: %d\n", tapeCurByte,(int)(ftell(tape)),tapebufByteCount, tapeBlockLen);
                        
                        // Free SymDefTable
                        for (int i = 0; i < asd; i++)
                            delete[] SymDefTable[i].PulseLenghts;
                        delete[] SymDefTable;

                        if (tapeBlkPauseLen == 0) {

                            if (tapeCurByte == 0x13) tapeEarBit ^= 1; // This is needed for Basil, maybe for others (next block == Pulse sequence)
                            // if (tapeCurByte != 0x19) tapeEarBit ^= 1; // This is needed for Basil, maybe for others (next block != GDB)

                            GDBEnd = true; // Provisional: add special end to GDB data blocks with pause 0

                            tapeCurBlock++;
                            GetBlock();

                        } else {

                            GDBEnd = false; // Provisional: add special end to GDB data blocks with pause 0

                            tapeEarBit ^= 1;
                            tapePhase=TAPE_PHASE_TAIL_GDB;
                            tapeNext = TAPE_PHASE_TAIL_LEN_GDB;

                        }

                    }
                } else {
                    tapeEarBit ^= 1;
                }
                break;

            case TAPE_PHASE_TAIL_GDB:
                tapeEarBit = 0;
                tapePhase=TAPE_PHASE_PAUSE_GDB;
                tapeNext=tapeBlkPauseLen;
                break;

            case TAPE_PHASE_PAUSE_GDB:
                tapeEarBit = 1;
                tapeCurBlock++;
                GetBlock();
                break;

            case TAPE_PHASE_DRB:
                tapeBitMask = (tapeBitMask >> 1) | (tapeBitMask << 7);
                if (tapeBitMask == tapeEndBitMask) {
                    tapeCurByte = readByteFile(tape);
                    tapebufByteCount++;
                    if (tapebufByteCount == tapeBlockLen) {
                        if (tapeBlkPauseLen == 0) {
                            tapeCurBlock++;
                            GetBlock();
                        } else {
                            tapePhase=TAPE_PHASE_TAIL;
                            tapeNext = TAPE_PHASE_TAIL_LEN;
                        }
                        break;
                    } else if ((tapebufByteCount + 1) == tapeBlockLen) {
                        if (tapeLastByteUsedBits < 8 )
                            tapeEndBitMask >>= tapeLastByteUsedBits;
                        else
                            tapeEndBitMask = 0x80;                        
                    } else {
                        tapeEndBitMask = 0x80;
                    }
                    tapeEarBit = tapeCurByte & tapeBitMask ? 1 : 0;
                } else {
                    tapeEarBit = tapeCurByte & tapeBitMask ? 1 : 0;
                }
                break;
            case TAPE_PHASE_SYNC:
                tapeEarBit ^= 1;
                if (--tapeHdrPulses == 0) {
                    tapePhase=TAPE_PHASE_SYNC1;
                    tapeNext=tapeSync1Len;
                }
                break;
            case TAPE_PHASE_SYNC1:
                tapeEarBit ^= 1;
                tapePhase=TAPE_PHASE_SYNC2;
                tapeNext=tapeSync2Len;
                break;
            case TAPE_PHASE_SYNC2:
                if (tapebufByteCount == tapeBlockLen) { // This is for blocks with data lenght == 0
                    if (tapeBlkPauseLen == 0) {
                        tapeCurBlock++;
                        GetBlock();
                    } else {
                        tapePhase=TAPE_PHASE_TAIL;
                        tapeNext=TAPE_PHASE_TAIL_LEN;                        
                    }
                    break;
                }
                tapeEarBit ^= 1;
                tapePhase=TAPE_PHASE_DATA1;
                tapeNext = tapeCurByte & tapeBitMask ? tapeBit1PulseLen : tapeBit0PulseLen;
                break;
            case TAPE_PHASE_DATA1:
                tapeEarBit ^= 1;
                tapePhase=TAPE_PHASE_DATA2;
                break;
            case TAPE_PHASE_DATA2:
                tapeEarBit ^= 1;
                tapeBitMask = tapeBitMask >>1 | tapeBitMask <<7;
                if (tapeBitMask == tapeEndBitMask) {
                    tapeCurByte = readByteFile(tape);                    
                    tapebufByteCount++;
                    if (tapebufByteCount == tapeBlockLen) {
                        if (tapeBlkPauseLen == 0) {
                            tapeCurBlock++;
                            GetBlock();
                        } else {
                            tapePhase=TAPE_PHASE_TAIL;
                            tapeNext=TAPE_PHASE_TAIL_LEN;
                        }
                        break;
                    } else if ((tapebufByteCount + 1) == tapeBlockLen) {
                        if (tapeLastByteUsedBits < 8 )
                            tapeEndBitMask >>= tapeLastByteUsedBits;
                        else
                            tapeEndBitMask = 0x80;                        
                    } else {
                        tapeEndBitMask = 0x80;
                    }
                }
                tapePhase=TAPE_PHASE_DATA1;
                tapeNext = tapeCurByte & tapeBitMask ? tapeBit1PulseLen : tapeBit0PulseLen;
                break;
            case TAPE_PHASE_PURETONE:
                tapeEarBit ^= 1;
                if (--tapeHdrPulses == 0) {
                    tapeCurByte = readByteFile(tape);
                    tapeCurBlock++;
                    GetBlock();
                }
                break;
            case TAPE_PHASE_PULSESEQ:
                tapeEarBit ^= 1;
                if (--tapeHdrPulses == 0) {
                    tapeCurByte = readByteFile(tape);
                    tapeCurBlock++;
                    GetBlock();
                } else {
                    tapeNext=(readByteFile(tape) | (readByteFile(tape) << 8));
                    tapebufByteCount += 2;
                }
                break;
            case TAPE_PHASE_END:
                tapeEarBit = 1;
                tapeCurBlock = 0;
                Stop();
                f_lseek(tape, 0);
                tapeNext = 0xFFFFFFFF;
                break;
            case TAPE_PHASE_TAIL:
                tapeEarBit = 0;
                tapePhase=TAPE_PHASE_PAUSE;
                tapeNext=tapeBlkPauseLen;
                break;
            case TAPE_PHASE_PAUSE:
                tapeEarBit = 1;
                tapeCurBlock++;
                GetBlock();
            } 
        } while (tapeCurrent >= tapeNext);

        // More precision just for DRB and CSW. Makes some loaders work but bigger TAIL_LEN also does and seems better solution.
        // if (tapePhase == TAPE_PHASE_DRB || tapePhase == TAPE_PHASE_CSW)
            tapeStart = CPU::global_tstates + CPU::tstates - tapeCurrent;
        // else
        //     tapeStart = CPU::global_tstates + CPU::tstates;

    }
}

void Tape::Save() {
    unsigned char xxor,salir_s;
	uint8_t dato;
	int longitud;
	FIL* fichero = fopen2(tapeSaveName.c_str(), FA_WRITE | FA_CREATE_ALWAYS);
    if (!fichero)
    {
        OSD::osdCenteredMsg(OSD_TAPE_SAVE_ERR, LEVEL_ERROR);
        return;
    }

	xxor=0;
	
	longitud=(int)(Z80::getRegDE());
	longitud+=2;
	
	dato=(uint8_t)(longitud%256);
    writeByteFile(dato, fichero);
	dato=(uint8_t)(longitud/256);
    writeByteFile(dato, fichero); // file length

    writeByteFile(Z80::getRegA(), fichero); // flag
	xxor^=Z80::getRegA();

	salir_s = 0;
	do {
	 	if (Z80::getRegDE() == 0)
	 		salir_s = 2;
	 	if (!salir_s) {
            dato = MemESP::readbyte(Z80::getRegIX());
            writeByteFile(dato, fichero);
	 		xxor^=dato;
	        Z80::setRegIX(Z80::getRegIX() + 1);
	        Z80::setRegDE(Z80::getRegDE() - 1);
	 	}
	} while (!salir_s);
    writeByteFile(xxor, fichero);
	Z80::setRegIX(Z80::getRegIX() + 2);
    fclose2(fichero);
}

bool Tape::FlashLoad() {
    if (Z80Ops::isALF) { // unsupported now
        return false;
    }
    if (!tape.obj.fs) {
        string fname = FileUtils::TAP_Path + tapeFileName;        
        if (f_open(&tape, fname.c_str(), FA_READ) != FR_OK) {
            return false;
        }
        CalcTapBlockPos(tapeCurBlock);
    }

    // printf("--< BLOCK: %d >--------------------------------\n",(int)tapeCurBlock);    
    FIL* tape = &Tape::tape;
    uint16_t blockLen=(readByteFile(tape) | (readByteFile(tape) <<8));
    uint8_t tapeFlag = readByteFile(tape);

    // printf("blockLen: %d\n",(int)blockLen - 2);
    // printf("tapeFlag: %d\n",(int)tapeFlag);
    // printf("AX: %d\n",(int)Z80::getRegAx());

    if (Z80::getRegAx() != tapeFlag) {
        // printf("No coincide el flag\n");
        Z80::setFlags(0x00);
        Z80::setRegA(Z80::getRegAx() ^ tapeFlag);
        if (tapeCurBlock < (tapeNumBlocks - 1)) {
            tapeCurBlock++;
            CalcTapBlockPos(tapeCurBlock);
            return true;
        } else {
            tapeCurBlock = 0;
            f_lseek(tape, 0);
            return false;
        }
    }

    // La paridad incluye el byte de flag
    Z80::setRegA(tapeFlag);

    int count = 0;
    int addr = Z80::getRegIX();    // Address start
    int nBytes = Z80::getRegDE();  // Lenght
    int addr2 = addr & 0x3fff;
    uint8_t page = addr >> 14;

    // printf("nBytes: %d\n",nBytes);

    if ((addr2 + nBytes) <= 0x4000) {

        // printf("Case 1\n");
        UINT br;
        uint8_t* p = MemESP::ramCurrent[page];
        if ( p < (uint8_t*)0x20000000 || (page == 0 && !MemESP::page0ram) ) {
            f_lseek(tape, f_tell(tape) + nBytes);
        } else {
            f_read(tape, &p[addr2], nBytes, &br);
        }

        while ((count < nBytes) && (count < blockLen - 1)) {
            Z80::Xor(MemESP::readbyte(addr));        
            addr = (addr + 1) & 0xffff;
            count++;
        }

    } else {

        // printf("Case 2\n");

        int chunk1 = 0x4000 - addr2;
        int chunkrest = nBytes > (blockLen - 1) ? (blockLen - 1) : nBytes;

        do {

            if ((page > 0) && (page < 4)) {
                UINT br;
                f_read(tape, &MemESP::ramCurrent[page][addr2], chunk1, &br);

                for (int i=0; i < chunk1; i++) {
                    Z80::Xor(MemESP::readbyte(addr));
                    addr = (addr + 1) & 0xffff;
                    count++;
                }

            } else {

                for (int i=0; i < chunk1; i++) {
                    Z80::Xor(readByteFile(tape));
                    addr = (addr + 1) & 0xffff;
                    count++;
                }

            }

            addr2 = 0;
            chunkrest = chunkrest - chunk1;
            if (chunkrest > 0x4000) chunk1 = 0x4000; else chunk1 = chunkrest;
            page++;

        } while (chunkrest > 0);

    }

    if (nBytes > (blockLen - 2)) {
        // Hay menos bytes en la cinta de los indicados en DE
        // En ese caso habrá dado un error de timeout en LD-SAMPLE (0x05ED)
        // que se señaliza con CARRY==reset & ZERO==set
        Z80::setFlags(0x50);
    } else {
        Z80::Xor(readByteFile(tape)); // Byte de paridad
        Z80::Cp(0x01);
    }

    if (tapeCurBlock < (tapeNumBlocks - 1)) {        
        tapeCurBlock++;
        if (nBytes != (blockLen -2)) CalcTapBlockPos(tapeCurBlock);
    } else {
        tapeCurBlock = 0;
        f_lseek(tape, 0);
    }

    Z80::setRegIX(addr);
    Z80::setRegDE(nBytes - (blockLen - 2));

    return true;

}
