#include <Arduino.h>
#include <Command.h>
#include "UARTBusDataDelegate.h"
#include "Configuration.h"
#include "DebugPrint.h"
#include "UARTBus.h"



    int UARTBus::available() {
      return serialPort->available();
    }

    /**
     * @brief Construct a new UARTBus object
     *
     * @param delegateIn A data delegate based on current command on the bus
     */
    UARTBus::UARTBus(UARTBusDataDelegate* delegateIn, Stream* in, Stream* out) {
      delegate = delegateIn;
      serialPort = (HardwareSerial*)(new SerialAdapter(in, out));
      DEBUG_PRINT("STARTING");
    }

    UARTBus::UARTBus() {
      UARTBus(0, nullptr, nullptr);
    }

    void UARTBus::startComms() {
      xTaskCreatePinnedToCore(handleCommunicationTask, "uart", 10000, this, 10, NULL, 0);
    }

    /**
     * @brief Sends a command on the bus
     *
     * @param c
     */
    void UARTBus::sendCommand(Command c) {
      sendData(c);
    }

    /**
     * @brief Consumes a raw command from the bus
     *
     * @return Command
     */
    Command UARTBus::receiveCommand() {
      return receiveData<Command>();
    }

    /**
     * @brief Consumes a Configuration object from the bus
     *
     * @return Configuration
     */
    Configuration UARTBus::receiveConfiguration() {
      return receiveData<Configuration>();
    }

    void UARTBus::updateCommand(Command c) {
        Command readCommand = Command(c);
        xQueueSend(commandQueue, &readCommand, 1000);
    }

    Command UARTBus::getCurrentCommand() {
      Command c;
      c.command = NOOP;
      if(uxQueueMessagesWaiting(commandQueue)) {
        xQueueReceive(commandQueue,&c,0);
      }
      return c;
    }

    void UARTBus::handleCommunicationTask(void* params) {

      UARTBus* This = (UARTBus*) params;

      DEBUG_PRINT("Starting on core: " + (String)xPortGetCoreID());
      for(;;) {
        vTaskDelay(1);
        Command currentCommand = Command();
        currentCommand.command = CommandType::NOOP;
        while(This->serialPort->available()) {
            currentCommand = This->receiveCommand();
            // Configure Bus
            if (currentCommand.command == CommandType::CONFIGURE) {
              DEBUG_PRINT((String) This->address + ": Starting configure...");
              This->address = currentCommand.address++;
              This->sendCommand(currentCommand);
              Configuration c = This->delegate->getConfiguration();
              This->forwardAndAppend(c, This->address);
              currentCommand.command = CommandType::NOOP;
              This->updateCommand(currentCommand);

              DEBUG_PRINT((String) This->address + ": Done configuring");
              continue;
            }

            // For this joint
            if (currentCommand.address == This->address) {
              currentCommand = currentCommand;
              Command forwarded = Command(currentCommand);
              This->delegate->fetchData(currentCommand.command, forwarded.data);
              This->sendCommand(forwarded);
              This->updateCommand(currentCommand);
              continue;
            }

            if (currentCommand.command & CAROUSEL) {
              int nJoints = (int)currentCommand.data[0];
              // Forward original command
              This->sendCommand(currentCommand);
              int16_t* dataBuffer = new int16_t[currentCommand.getNReturn()];
              byte nDataOut = This->delegate->fetchData(currentCommand.command, dataBuffer);
              if ((CommandType)(currentCommand.command & COMMANDTYPE_MASK) == NOOP) {
                This->leftShiftBus(nJoints, nullptr, 0, dataBuffer, nDataOut);
              } else {
                This->leftShiftBus(nJoints, currentCommand.data, 1, dataBuffer, nDataOut);
              }
              delete[] dataBuffer;

              // Forward data
              This->updateCommand(currentCommand);
              continue;
            }

            // Forward
            This->sendCommand(currentCommand);
            currentCommand.command = CommandType::NOOP;
            This->updateCommand(currentCommand);
        }
      }
    }


    /**
     * @brief Handles forwarding and data fetching. Returns any new commands to be processed
     *
     * @return Command
     */
    Command UARTBus::handleCommunication() {
      handleCommunicationTask(this);
      return getCurrentCommand();
    }


