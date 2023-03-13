/*
  gcode_host.cpp -  gcode host functions class

  Copyright (c) 2014 Luc Lebosse. All rights reserved.

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
//#define ESP_DEBUG_FEATURE DEBUG_OUTPUT_SERIAL0
#include "../../include/esp3d_config.h"
#if defined(GCODE_HOST_FEATURE)
#include "gcode_host.h"
#include "../../core/settings_esp3d.h"
#include "../../core/commands.h"
#include "../../core/esp3doutput.h"
#if defined(FILESYSTEM_FEATURE)
#include "../filesystem/esp_filesystem.h"
#endif //FILESYSTEM_FEATURE
#if defined(SD_DEVICE)
#include "../filesystem/esp_sd.h"
#endif //SD_DEVICE

/* Defined in configuration.h
#define ESP_HOST_TIMEOUT 16000
#define ESP_HOST_HEATING_TIMEOUT 60000
#define MAX_TRY_2_SEND 5
*/

GcodeHost esp3d_gcode_host;

GcodeHost::GcodeHost()
{
    reset();
}

GcodeHost::~GcodeHost()
{
    reset();
}

bool GcodeHost::begin()
{
    reset();
    _injectionMutex = xSemaphoreCreateMutex();
    assert(_injectionMutex);
    return true;
}

void GcodeHost::end()
{
    reset();
    vSemaphoreDelete(_injectionMutex);
}

void GcodeHost::reset()
{
    _commandNumber = 0;
    _needCommandNumber = 0;
    _currentCommand = "";
    _injectedCommand = "";
    _injectionQueued = false;
    _needAck = false;
    _noTimeout = false;
    _error = ERROR_NO_ERROR;
    _step = HOST_NO_STREAM;
    _nextStep = HOST_NO_STREAM;
    _currentPosition = 0; //is technically +1 to the address of the character it corresponds with.
    memset(_buffer, 0, sizeof(_buffer));
    _bufferSize = 0;
    _outputStream.client(ESP_STREAM_HOST_CLIENT);
    _totalSize = 0;
    _processedSize = 0;
}


//Input/response from the serial port to sbuf
bool GcodeHost::push(const uint8_t * sbuf, size_t len)
{
    log_esp3d("Push got %d bytes", len);
    for (size_t i = 0; i < len; i++) {
        //Found a line end, flush it
        if (sbuf[i]=='\n' || sbuf[i]=='\r') {
            flush();
        } else {
            //fill buffer until it is full
            if (_bufferSize < ESP_HOST_BUFFER_SIZE) {
                _buffer[_bufferSize++] = sbuf[i];
            } else {
                //buffer is full flush it - should probably be reworked, 
                //as the last half of a command isn't likely to be useful after the first is flushed
                flush();
                _buffer[_bufferSize++] = sbuf[i];
            }
            _buffer[_bufferSize] =0;
        }
    }
    flush();

    return true;

}

bool GcodeHost::isAck(String & line)
{ //should probably also consider "invalid command" messages as Ack if they don't send one seperately
    if (line.indexOf("ok") != -1) { //maybe should have "ok\n" to ensure it's at line end (or some other way), but would need adding to end of string first
        log_esp3d("got ok");
        return true;
    }
    if (Settings_ESP3D::GetFirmwareTarget()==SMOOTHIEWARE) {
        if (line.indexOf("smoothie out") != -1) {
            log_esp3d("got smoothie out");
            return true;
        }
    }
    return false;
}

uint32_t GcodeHost::resendCommandNumber(String & response)
{
    uint32_t l = 0;
    String sresend = "resend:"; //decapitalised from "Resend:" due to _response.toLowerCase() (Maybe not necessary if we catalogue enough responses)
    if ( Settings_ESP3D::GetFirmwareTarget() == SMOOTHIEWARE) {
        sresend = "rs n"; //same here, "rs N"
    }
    int pos = response.indexOf(sresend);
    if (pos == -1 ) {
        log_esp3d("Cannot find label %d", _error);
        return 0;
    }
    pos+=sresend.length();
    int pos2 = response.indexOf("\n", pos);
    String snum = response.substring(pos, pos2);
    //remove potential unwished char
    snum.replace("\r", "");
    snum.trim();
    l = snum.toInt();
    log_esp3d("Command number to resend is %s", String((uint32_t)l).c_str());
    return l;
}

//GcodeHost::push should call this at the end of each line that it parses
//it checks for ack or errors and performs the appropriate actions.
//WIP only handles simple ok so far
void GcodeHost::flush()
{
    //if buffer is empty, return
    if(_bufferSize==0) {
        return;
    }
    _response = (const char*)_buffer;
    log_esp3d("Stream got the response: %s", _response.c_str());
    _response.toLowerCase();

    if (isAck(_response)) {
        //check if we have proper ok response
        //like if numbering is enabled
        /*
        if((_step == HOST_WAIT4_ACK) || (_step == HOST_WAIT4_ACK_NT)) {
            _step = _nextStep;
        } else {
            log_esp3d("Got ok but out of the query");
        }
        */
        if (_needAck == true){
            _needAck = false;
            _noTimeout = false;
        } else {
            log_esp3d("Got ok but out of the query");
        }
        
    } else {
        if (_response.indexOf("error") != -1) {
            log_esp3d("Got error");
            _step = HOST_ERROR_STREAM;
        }
    }

    if ((_needCommandNumber = resendCommandNumber(_response)) != 0){// -----------------------------------------------------------------------------------------
        //set a flag to set the appropriate command before reading or processing any more.
        //if((_step == HOST_WAIT4_ACK) || (_step == HOST_WAIT4_ACK_NT)) {
        //    _step = _nextStep;
        if (_needAck == true){
            _needAck = false;
            _noTimeout = false;
        } else {
            log_esp3d("Got resend out of the query");
        }

    }


    //TODO
    //if resend request
    //if other commands

    //Clear the buffer
    memset(_buffer, 0, sizeof(_buffer));
    _bufferSize = 0;

}

//Opens the file/script initialized by processScript or processFile and sets the stream state as reading
bool GcodeHost::startStream()
{
    if (_fsType ==TYPE_SCRIPT_STREAM) {
        _totalSize = _script.length();
        log_esp3d("Script line %s opened, size is %d", _script.c_str(), _totalSize);
    }
#if defined(FILESYSTEM_FEATURE)
    if (_fsType ==TYPE_FS_STREAM) {
        if (ESP_FileSystem::exists(_fileName.c_str())) {
            fileHandle = ESP_FileSystem::open(_fileName.c_str());
        }
        if (fileHandle.isOpen()) {
            _totalSize = fileHandle.size();
            log_esp3d("File %s opened, size is %d", _fileName.c_str(), _totalSize);
        } else {
            _error = ERROR_FILE_NOT_FOUND;
            _step = HOST_ERROR_STREAM;
            log_esp3d("File not found: %s", _fileName.c_str());
            return false;
        }
    }
#endif //FILESYSTEM_FEATURE
#if defined(SD_DEVICE)
    if (_fsType ==TYPE_SD_STREAM) {
        if (!ESP_SD::accessFS()) {
            _error = ERROR_FILE_NOT_FOUND;
            _step = HOST_ERROR_STREAM;
            _needRelease = false;
            log_esp3d("File not found: %s", _fileName.c_str());
            return false;
        }
        _needRelease = true;
        if (ESP_SD::getState(true) == ESP_SDCARD_NOT_PRESENT) {
            _error = ERROR_FILE_NOT_FOUND;
            _step = HOST_ERROR_STREAM;
            log_esp3d("File not found: %s", _fileName.c_str());
            return false;
        }
        ESP_SD::setState(ESP_SDCARD_BUSY );

        if (ESP_SD::exists(_fileName.c_str())) {
            fileHandle = ESP_SD::open(_fileName.c_str());
        }
        if (fileHandle.isOpen()) {
            _totalSize = fileHandle.size();
            log_esp3d("File %s opened, size is %d", _fileName.c_str(), _totalSize);
        } else {
            _error = ERROR_FILE_NOT_FOUND;
            _step = HOST_ERROR_STREAM;
            log_esp3d("File not found: %s", _fileName.c_str());
            return false;
        }
    }
#endif //SD_DEVICE
    resetCommandNumber();
    _currentPosition = 0;
    _processedSize = 0;
    _response = "";
    _error = ERROR_NO_ERROR;
    _startTimeOut =millis();
    _step = HOST_READ_LINE;
    _nextStep = HOST_READ_LINE;

    return true;
}

//Closes open file and releases the SD card if in use.
void GcodeHost::endStream()
{
    log_esp3d("Ending Stream");
#if defined(FILESYSTEM_FEATURE)
    if (_fsType == TYPE_FS_STREAM) {
        if (fileHandle.isOpen()) {
            fileHandle.close();
        }
    }
#endif //FILESYSTEM_FEATURE
#if defined(SD_DEVICE)
    if (_fsType ==TYPE_SD_STREAM) {
        if (fileHandle.isOpen()) {
            fileHandle.close();
        }
        if(_needRelease) {
            ESP_SD::releaseFS();
        }
    }
#endif //SD_DEVICE
    _step = HOST_NO_STREAM;
    _nextStep = HOST_NO_STREAM;
    //reset();
}


//NEEDS REWRITING FOR MULTIPLE LINES AND MUTEX - doneeee
bool GcodeHost::sendCommand(const uint8_t* injection, size_t len)
{   
    String inject = "";
    const uint8_t* injectAddr = injection;
    //read command into string
    for (size_t i = 0; i < len;  i++){
        inject = inject + (char)*injectAddr;
        injectAddr++;
    }
    _injectionQueued = true;
    inject.trim();
    
    if(xSemaphoreTake(_injectionMutex, MUTEX_TIMEOUT)) {
        /*if ((inject.indexOf("M112") != -1) || (inject.indexOf("M108") != -1)){ //if is emergency stop, jump to top of queue, set _ignoreAck - any others want to skip ack? need an emergency parser function - M108 too
            while(inject.length() > 0){
                if(_injectedCommand.length() > 0){
                    _injectedCommand ='\n' + _injectedCommand;
                }
                int NL = inject.indexOf('\n');
                if (NL != -1){
                    _injectedCommand = _injectedCommand + inject.substring(0, NL-1);
                    inject = inject.substring(NL + 1);
                    inject.trim();

                }else{
                    _injectedCommand = inject + _injectedCommand;
                    inject = "";
                }
            }
            //_needAck = false; //Maybe not this way, read whilst waiting for ack, and process when we aren't 

        } else {*/ if (1){
            while(inject.length() > 0){
                if(_injectedCommand.length() > 0){
                    _injectedCommand = _injectedCommand + '\n';
                }
                int NL = inject.indexOf('\n');
                if (NL != -1){
                    _injectedCommand = _injectedCommand + inject.substring(0, NL-1);
                    inject = inject.substring(NL + 1);
                    inject.trim();

                }else{
                    _injectedCommand = _injectedCommand + inject;
                    inject = "";
                }
            }
        }
        xSemaphoreGive(_injectionMutex);
    }else{
        return false;
    }

    return true;
}


bool GcodeHost::sendScript(const char * line, level_authenticate_type auth_type, ESP3DOutput * output)
{
    _script = *line; //don't need a global variable for this
    sendCommand((const uint8_t *)_script.c_str(), _script.length());
    _script = "";

    //_fsType = TYPE_SCRIPT_STREAM;
    //_step = HOST_START_STREAM;
    //_auth_type = auth_type;
    return true;
    
}

/// @brief Read the next line for processing from the script, FS file or SD file.
//may be better to have seperate function for injection and check _injectionQueued in Handle
void GcodeHost::readNextCommand()
{
    _currentCommand = "";
    _step = HOST_PROCESS_LINE;
    
#if defined(FILESYSTEM_FEATURE) || defined(SD_DEVICE)

    _processedSize++;
    _currentPosition++; 
    char c = (char)fileHandle.read(); //does casting need to be explicit? probably for the best for now at least
    while (((c =='\n') || (c =='\r') || (c == ' '))){ //ignore any leading spaces and empty lines
        _processedSize++;
        _currentPosition++;
        c = (char)fileHandle.read();
    }
    while (c == ';'){ // while its a full line comment, read on to the next line
        c = (char)fileHandle.read();
        while (!((c =='\n') || (c =='\r') || (c == 0) || (c == -1))){ //skim to end of line
            _processedSize++;
            _currentPosition++;
            c = (char)fileHandle.read();
        }
        while (((c =='\n') || (c =='\r') || (c == ' '))){ //ignore any leading spaces and empty lines
            _processedSize++;
            _currentPosition++;
            c = (char)fileHandle.read();
        }
    }

    while (!((c == '\n') || (c =='\r') || (c == 0) || (c == -1) || (_currentPosition >= _totalSize-2))){ // while not end of line or end of file
        if (c == ';'){ //reached a comment, skip to next line
            while (!((c =='\n') || (c =='\r') || (c == 0) || (c == -1))){ //skim to end of line
                _processedSize++;
                _currentPosition++;
                c = (char)fileHandle.read();
            }
        } else { //no comment yet, read into command
            _currentCommand += c;
            _processedSize++;
            _currentPosition++;
            c = (char)fileHandle.read();
        }
    }
#endif //FILESYSTEM_FEATURE

    if (_currentCommand.length() == 0) {
        _step = HOST_STOP_STREAM;
        _nextStep = HOST_NO_STREAM;
    }
}

/// @brief Read the next line for processing from the injection buffer
void GcodeHost::readInjectedCommand()
{
    if (_injectedCommand.length() > 0) {
        if(xSemaphoreTake(_injectionMutex, MUTEX_TIMEOUT)) {
            _currentCommand = "";
            _step = HOST_PROCESS_LINE;
            uint32_t ix = 0;
            char c = _injectedCommand[ix];
            ix++;

            while (((c =='\n') || (c =='\r') || (c == ' '))){
                c = _injectedCommand[ix];
                ix++;
            }
            while (c == ';'){ // while its a full line comment, read on to the next line
                c = _injectedCommand[ix];
                ix++;
                while (!((c =='\n') || (c =='\r') || (c == 0))){ 
                    c = _injectedCommand[ix];
                    ix++;
                }
                while (((c =='\n') || (c =='\r') || (c == ' '))){ 
                    c = _injectedCommand[ix];
                    ix++;
                }
            }

            while (!((c == '\n') || (c =='\r') || (c == 0))){ // while not end of line or end of file
                if (c == ';'){ //reached a comment, skip to next line
                    while (!((c =='\n') || (c =='\r') || (c == 0))){ 
                        c = _injectedCommand[ix];
                        ix++;
                    }
                } else { //no comment yet, read into command
                    _currentCommand += c;
                    c = _injectedCommand[ix];
                    ix++;
                }
            }
            if (ix == _injectedCommand.length()){
                _injectionQueued = false;
                _injectedCommand = "";
            } else {
                _injectedCommand = _injectedCommand.substring(ix);
            }
            _skipChecksum = true; 
            xSemaphoreGive(_injectionMutex);

            if ((_currentCommand.indexOf("M108") != -1)|| (_currentCommand.indexOf("M112") != -1)){
                _needAck = false;
            }

        }
    } else {
        _step = _nextStep;
        _injectionQueued = false;
        _skipChecksum = false; 
    }
}

void GcodeHost::awaitAck()
{
    if ((_currentCommand.indexOf("M190") == 0) && (_currentCommand.indexOf("M109") == 0) && (_currentCommand.indexOf("G4") == 0) && (_currentCommand.indexOf("M400") == 0)) { // should we check for them at the start, or anywhere in the line?
        //_step = HOST_WAIT4_ACK_NT;
        _needAck = true;
        _noTimeout = true;
    }
    else{
        //_step = HOST_WAIT4_ACK;
        _needAck = true;
        _noTimeout = false;
    }
}

//Adds the checksum and line number to the command and sends it to process for sending
void GcodeHost::processCommand()
{
        log_esp3d("Processing command %s ", _currentCommand.c_str());

        ESP3DOutput outputhost(ESP_STREAM_HOST_CLIENT);

        if (esp3d_commands.is_esp_command((uint8_t *)_currentCommand.c_str(), _currentCommand.length())) { // If it's a command for the ESP, send it on
            esp3d_commands.process((uint8_t *)_currentCommand.c_str(), _currentCommand.length(),&outputhost, _auth_type);
            log_esp3d("Command is ESP command: %s, client is %d", _currentCommand.c_str(), outputhost); //Make sure this works
        } else { //if it's for the printer, see if it needs checksum + line no, add if so
            awaitAck();
            if(!_skipChecksum){ //For injected commands
                _currentCommand = CheckSumCommand(_currentCommand.c_str(), _commandNumber);
                _commandNumber++;
                _skipChecksum = false;
            }
            _currentCommand = _currentCommand + "\n";

#if COMMUNICATION_PROTOCOL == SOCKET_SERIAL
            ESP3DOutput output(ESP_SOCKET_SERIAL_CLIENT);
            esp3d_commands.process((uint8_t *)_currentCommand.c_str(), _currentCommand.length(),&_outputStream, _auth_type,&output,_outputStream.client()==ESP_ECHO_SERIAL_CLIENT?ESP_SOCKET_SERIAL_CLIENT:0 ) ;
#endif //COMMUNICATION_PROTOCOL == SOCKET_SERIAL            
#if COMMUNICATION_PROTOCOL == RAW_SERIAL || COMMUNICATION_PROTOCOL == MKS_SERIAL
            ESP3DOutput output(ESP_SERIAL_CLIENT);
            esp3d_commands.process((uint8_t *)_currentCommand.c_str(), _currentCommand.length(),&outputhost, _auth_type,&output);
#endif //COMMUNICATION_PROTOCOL == SERIAL           
            _startTimeOut =millis();
            log_esp3d("Command is GCODE command");
        
        }
        _step = _nextStep;
}

void GcodeHost::handle()
{

    switch(_step) {

    case HOST_NO_STREAM:
        if(_injectionQueued){
            readInjectedCommand();
        }
    break;

    case HOST_START_STREAM:
        startStream();
    break;

    case HOST_STOP_STREAM:
        endStream();
    break;

    case HOST_PAUSE_STREAM:
    //inject pause script/file
#if defined(HOST_PAUSE_SCRIPT)

#endif
        _nextStep = HOST_STREAM_PAUSED;
        _step = HOST_STREAM_PAUSED;
    break;

    case HOST_STREAM_PAUSED:
    //do injection if required
        if(_injectionQueued){
            readInjectedCommand();
        }
    break;

    case HOST_RESUME_STREAM:
    //inject resume script/file
#if defined(HOST_RESUME_SCRIPT)

#endif
        _nextStep = HOST_READ_LINE;
        _step = HOST_READ_LINE;
    break;

    case HOST_ABORT_STREAM:
    //inject abort script/file
#if defined(HOST_ABORT_SCRIPT)

#endif
        _nextStep = HOST_NO_STREAM;
        _step = HOST_STOP_STREAM;
    break;

    case HOST_READ_LINE:
        if (_needCommandNumber != 0){
            gotoLine(_needCommandNumber);
            _needCommandNumber = 0;
            readNextCommand();
        } else if (_nextStep != HOST_READ_LINE){
            if (_nextStep == HOST_PAUSE_STREAM){
                _step = HOST_PAUSE_STREAM;
                _nextStep = HOST_STREAM_PAUSED;
            }else if(_nextStep = HOST_STREAM_PAUSED){
                _step = HOST_STREAM_PAUSED;
            } else if (_nextStep == HOST_ABORT_STREAM){
                _step = HOST_ABORT_STREAM;
                _nextStep == HOST_NO_STREAM;
            } else if (_nextStep == HOST_NO_STREAM){
                _step = HOST_NO_STREAM;
            }
        }else if (_injectionQueued){
            readInjectedCommand();
        }else{
            readNextCommand();
        }
    break;

    case HOST_PROCESS_LINE:
        if (_needAck == true){
            if ((_noTimeout == false) && (millis() - _startTimeOut > ESP_HOST_TIMEOUT)) {
                log_esp3d("Timeout waiting for ack");
                _error = ERROR_TIME_OUT;
                _step = HOST_ERROR_STREAM;
            }
        } else if (_needCommandNumber != 0){
            gotoLine(_needCommandNumber);
            _needCommandNumber = 0;
            readNextCommand();
        } else {
            processCommand();
        }
    break;

    case HOST_ERROR_STREAM: {
        String Error;
        if (_error == ERROR_NO_ERROR) {
            //TODO check _response to put right error
            _error = ERROR_UNKNOW;
        }
        log_esp3d("Error %d", _error);
        Error = "error: stream failed: " + String(_error) + "\n";
#if COMMUNICATION_PROTOCOL == SOCKET_SERIAL
        ESP3DOutput output(ESP_SOCKET_SERIAL_CLIENT);
#endif//COMMUNICATION_PROTOCOL
#if COMMUNICATION_PROTOCOL == RAW_SERIAL || COMMUNICATION_PROTOCOL == MKS_SERIAL
        ESP3DOutput output(ESP_SERIAL_CLIENT);
#endif//COMMUNICATION_PROTOCOL
        output.dispatch((const uint8_t *)Error.c_str(), Error.length());
        _step = HOST_STOP_STREAM;
    }
    break;

    default: //Not handled step
        log_esp3d("Not handled step %d", _step);
    break;

    }

}

bool  GcodeHost::abort()
{
    if (_step == HOST_NO_STREAM) {
        return false;
    }
    log_esp3d("Aborting stream");
    //TODO: what to do in addition ?
    //Emergency stop for Marlin added (Maybe need a switch for other firmwares? -> should be mostly universal -> Not GRBL)
    //abort script/file/whatever
    _error=ERROR_STREAM_ABORTED;
    //we do not use step to do faster abort -> ok as long as we don't mind losing place etc
    endStream();
    _step = HOST_ABORT_STREAM;
    _nextStep = HOST_NO_STREAM;
    return true;
}

bool GcodeHost::pause()
{
    if ((_step == HOST_NO_STREAM) || (_step == HOST_STREAM_PAUSED)) {
        return false;
    }
    _nextStep = HOST_PAUSE_STREAM;
    return true;
}

bool GcodeHost::resume()
{
    if (_step != HOST_STREAM_PAUSED) {
        return false;
    }
    _step = HOST_RESUME_STREAM;
    _nextStep = HOST_READ_LINE;
    return true;
}

uint8_t GcodeHost::Checksum(const char * command, uint32_t commandSize)
{
    uint8_t checksum_val =0;
    if (command == NULL) {
        return 0;
    }
    for (uint32_t i=0; i < commandSize; i++) {
        checksum_val = checksum_val ^ ((uint8_t)command[i]);
    }
    return checksum_val;
}

String GcodeHost::CheckSumCommand(const char* command, uint32_t commandnb)
{
    String commandchecksum = "N" + String((uint32_t)commandnb)+ " " + command;
    uint8_t crc = Checksum(commandchecksum.c_str(), commandchecksum.length());
    commandchecksum+="*"+String(crc);
    return commandchecksum;
}

void GcodeHost::resetCommandNumber()
{
    String resetcmd = "M110 N0";
    if (Settings_ESP3D::GetFirmwareTarget() == SMOOTHIEWARE) {
        resetcmd = "N0 M110";
    } else {
        resetcmd = "M110 N0";
    }
    _commandNumber = 1;

    sendCommand((const uint8_t *)resetcmd.c_str(), resetcmd.length()); //needs testing

}

bool GcodeHost::processFile(const char * filename, level_authenticate_type auth_type, ESP3DOutput * output)
{
    bool target_found = false;
#if COMMUNICATION_PROTOCOL == SOCKET_SERIAL
    log_esp3d("Processing file client is  %d", output?output->client():ESP_SOCKET_SERIAL_CLIENT);
    _outputStream.client(output?output->client():ESP_SOCKET_SERIAL_CLIENT);
#endif//COMMUNICATION_PROTOCOL
#if COMMUNICATION_PROTOCOL == RAW_SERIAL || COMMUNICATION_PROTOCOL == MKS_SERIAL
    log_esp3d("Processing file client is  %d", output?output->client():ESP_SERIAL_CLIENT);
    _outputStream.client(output?output->client():ESP_SERIAL_CLIENT);
#endif//COMMUNICATION_PROTOCOL
    //sanity check
    _fileName = filename[0]!='/'?"/":"";
    _fileName +=filename;
    _fileName.trim();
    log_esp3d("Processing file: %s", filename);
    if (_fileName.length() == 0) {
        log_esp3d("No file to process");
        return false;
    }
    if (_step != HOST_NO_STREAM) {
        log_esp3d("Streaming already in progress");
        return false;
    }
    //TODO UD = USB DISK
#if defined(SD_DEVICE)
    if (_fileName.startsWith(ESP_SD_FS_HEADER)) {
        log_esp3d("Processing SD file");
        target_found = true;
        _fileName= _fileName.substring(strlen(ESP_SD_FS_HEADER),_fileName.length());
        _fsType = TYPE_SD_STREAM;
    }
#endif //SD_DEVICE
#if defined(FILESYSTEM_FEATURE)
    if (!target_found && _fileName.startsWith(ESP_FLASH_FS_HEADER)) {
        target_found = true;
        _fileName= _fileName.substring(strlen(ESP_FLASH_FS_HEADER),_fileName.length());
        log_esp3d("Processing /FS file %s", _fileName.c_str());
        _fsType = TYPE_FS_STREAM;
    }
    //if no header it is also an FS file
    if (!target_found) {
        log_esp3d("Processing FS file %s", _fileName.c_str());
        _fsType = TYPE_FS_STREAM;
        target_found = true;
    }
#endif //FILESYSTEM_FEATURE
    //it is not a file so it is a script
    if (!target_found ) {
        target_found = true;
        _fsType = TYPE_SCRIPT_STREAM;
        //remove the /
        _script=&_fileName[1];
        log_esp3d("Processing Script file %s", _script.c_str());
        _fileName = "";
    }
    _step = HOST_START_STREAM;
    _auth_type = auth_type;
    return true;
}

bool GcodeHost::gotoLine(uint32_t line)
{
    //add checks for current state and step. should be called from Handle()
    _commandNumber = 1;
    _currentPosition = 0;

    fileHandle.seek(_currentPosition);

    for ( _commandNumber = 1; _commandNumber < line; _commandNumber++){
        readNextCommand();
        if (esp3d_commands.is_esp_command((uint8_t *)_currentCommand.c_str(), _currentCommand.length())) {
            _commandNumber--; //if it's for the ESP it doesn't count
        }
    }

    return true;


    /* following method should be faster on long files but doesn't work well enough yet
    char c;
    bool notEmpty = false;
    String check = "";

    if (line < _commandNumber) { //if we're going backwards (which should be the only use case really)
        while (_commandNumber > line){
            _currentPosition -= 1;
            fileHandle.seek((_currentPosition-1));
            c = (char)fileHandle.read();
            if (c == '\n'){ // if we reach an end line, we're at the end of a previous line - make sure it's not a comment or ESP command.
                if (notEmpty == true){ //line isn't empty
                    while ((c == ' ') || (c == '\n') || (c == '\r')){
                        c = (char)fileHandle.read();
                    }
                    if ((c != ';') && (c != '[')) { //not a comment or ESP command, or an empty line, must be a command - count it
                        _commandNumber -= 1;
                    }
                }
                
            } else if (c != ' ') {
                notEmpty = true;
            }
        }
        fileHandle.seek((_currentPosition));
    } else if (line > _commandNumber){
        while (_commandNumber < line){
            readNextCommand();
            _commandNumber++;
        } 
    } else {
        //TODO error message for current line requested
        return false;
    }

    return true;

*/

}



#endif //GCODE_HOST_FEATURE
