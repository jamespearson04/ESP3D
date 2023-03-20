/*
gcode_host.h - gcode host functions class

  Copyright (c) 2014 Luc Lebosse & 2022 James Pearson. All rights reserved.

  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with This code; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#ifndef _GCODE_HOST_H
#define _GCODE_HOST_H

#include <Arduino.h>
#include "../authentication/authentication_service.h" 


#if defined(FILESYSTEM_FEATURE) || defined(SD_DEVICE) //probably need sd too
#include "../filesystem/esp_filesystem.h"
#endif //FILESYSTEM_FEATURE
#if defined(SD_DEVICE)
#include "../filesystem/esp_sd.h"
#endif

class ESP3DOutput;

//Error states
#define ERROR_NO_ERROR          0
#define ERROR_TIME_OUT          1
#define ERROR_CANNOT_SEND_DATA  2
#define ERROR_LINE_NUMBER       3
#define ERROR_ACK_NUMBER        4
#define ERROR_MEMORY_PROBLEM    5
#define ERROR_RESEND            6
#define ERROR_NUMBER_MISMATCH   7
#define ERROR_LINE_IGNORED      8
#define ERROR_FILE_SYSTEM       9
#define ERROR_CHECKSUM          10
#define ERROR_UNKNOW            11
#define ERROR_FILE_NOT_FOUND    12
#define ERROR_STREAM_ABORTED    13

//Host streaming steps
#define HOST_READ_LINE     0
#define HOST_PROCESS_LINE  1
#define HOST_NO_STREAM     4
#define HOST_START_STREAM  5
#define HOST_PAUSE_STREAM  6
#define HOST_RESUME_STREAM 7
#define HOST_STOP_STREAM   8
#define HOST_ERROR_STREAM  9
#define HOST_ABORT_STREAM  10
#define HOST_STREAM_PAUSED 11
#define HOST_STREAM_COMMAND 12
#define HOST_GOTO_LINE     13
#define HOST_STREAMING_SCRIPT 14
#define HOST_STREAM_RESUMED 15

#define TYPE_SCRIPT_STREAM 0
#define TYPE_FS_STREAM     1
#define TYPE_SD_STREAM     2

#define  ESP_HOST_BUFFER_SIZE 255

#define MUTEX_TIMEOUT 10000 //portMAX_DELAY

class GcodeHost
{
public:

    //SemaphoreHandle_t _injectionMutex;

    GcodeHost();
    ~GcodeHost();

    bool begin();
    void reset();
    void end();

    void handle();

    bool processFile(const char * filename, level_authenticate_type auth_type = LEVEL_ADMIN, ESP3DOutput * output=nullptr);
    bool sendCommand(const uint8_t* injection, size_t len);
    
    void readNextCommand();
    void readInjectedCommand();
    bool gotoLine(uint32_t line);
    void awaitAck();
    void processCommand();
    void readScript();
    

    bool startStream();
    void endStream();
    bool pause();
    bool resume();
    bool abort();

    bool push(const uint8_t * sbuf, size_t len);
    void flush();
    bool isAck(String & line);
    bool isBusy(String & line);

    void resetCommandNumber();
    uint32_t resendCommandNumber(String & response);
    uint32_t getCommandNumber(){ return _commandNumber;}
    void setCommandNumber(uint32_t n){ _commandNumber = n;}

    uint8_t Checksum(const char * command, uint32_t commandSize);
    String CheckSumCommand(const char* command, uint32_t commandnb);

    void  setErrorNum(uint8_t error){ _error = error;}
    uint8_t getErrorNum(){ return _error;}
    uint8_t getStatus(){ return _step;}

    size_t totalSize(){ return _totalSize;}
    size_t processedSize(){ return _processedSize;}

    uint8_t getFSType(){ return _fsType;}
    const char * fileName()
    {
        if (_fileName.length() == 0) { return nullptr;}
        return _fileName.c_str();
    }

private:
    uint8_t _buffer [ESP_HOST_BUFFER_SIZE+1];
    size_t _bufferSize;

    
#if defined(FILESYSTEM_FEATURE) || defined(SD_DEVICE)
    ESP_File fileHandle;
#endif //FILESYSTEM_FEATURE

#if defined(SD_DEVICE)
    ESP_SDFile SDfileHandle;
    bool _needRelease;
#endif //SD_DEVICE

    size_t _totalSize;
    size_t _processedSize;
    size_t _saveProcessedSize;

    uint32_t _currentPosition;
    
    // May be better to replace strings with char arrays if memory turns out to be an issue.
    String _currentCommand;
    String _saveCommand;
    String _injectedCommand;

    uint32_t _commandNumber;
    uint32_t _needCommandNumber;
    uint32_t _saveCommandNumber;

    String _fileName;
    String _saveFileName;

    String _script;
    uint8_t _fsType;


    uint8_t _step;
    uint8_t _nextStep;
    uint8_t _error;
    bool _injectionQueued;
    bool _injectionNext;
    bool _skipChecksum;
    bool _needAck;
    bool _noTimeout;

    String _response;

    uint64_t _startTimeOut;
    uint64_t _timeoutInterval;

    ESP3DOutput _outputStream;
    level_authenticate_type _auth_type;
    
    
};

extern GcodeHost esp3d_gcode_host;

#endif