#include "SerialCommands.h"

namespace SerialCommands
{

void dispatch(const String &input, const Handlers &handlers, size_t malletCount, Stream &out)
{
  String cmd = input;
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0)
  {
    return;
  }

  if (cmd == "arp")
  {
    if (handlers.playCalibrationArpeggio != nullptr)
    {
      handlers.playCalibrationArpeggio();
    }
  }
  else if (cmd == "cal")
  {
    if (handlers.runFullCalibration != nullptr)
    {
      handlers.runFullCalibration();
    }
  }
  else if (cmd.startsWith("cal "))
  {
    String malletToken = cmd.substring(4);
    malletToken.trim();
    if (malletToken.length() == 0 || malletToken.indexOf(' ') >= 0)
    {
      out.print("cal usage: cal [malletIndex 0..");
      out.print(static_cast<int>(malletCount) - 1);
      out.println("]");
      return;
    }

    bool tokenIsNumeric = true;
    for (size_t i = 0; i < static_cast<size_t>(malletToken.length()); ++i)
    {
      const char c = malletToken.charAt(i);
      if (c < '0' || c > '9')
      {
        tokenIsNumeric = false;
        break;
      }
    }

    if (!tokenIsNumeric)
    {
      out.print("cal usage: cal [malletIndex 0..");
      out.print(static_cast<int>(malletCount) - 1);
      out.println("]");
      return;
    }

    const int malletIndex = malletToken.toInt();
    if (malletIndex < 0 || malletIndex >= static_cast<int>(malletCount))
    {
      out.print("cal mallet index out of range 0..");
      out.println(static_cast<int>(malletCount) - 1);
      return;
    }

    if (handlers.runFullCalibrationForMallet != nullptr)
    {
      handlers.runFullCalibrationForMallet(static_cast<size_t>(malletIndex));
    }
  }
  else if (cmd == "calvel")
  {
    if (handlers.runVelocityCalibrationOnly != nullptr)
    {
      handlers.runVelocityCalibrationOnly();
    }
  }
  else if (cmd == "status")
  {
    if (handlers.printMalletStatus != nullptr)
    {
      handlers.printMalletStatus();
    }
  }
  else if (cmd == "audiostatus")
  {
    if (handlers.printAudioStatus != nullptr)
    {
      handlers.printAudioStatus();
    }
  }
  else if (cmd == "micdiag")
  {
    if (handlers.printMicDiagnostics != nullptr)
    {
      handlers.printMicDiagnostics();
    }
  }
  else if (cmd.startsWith("micprobe"))
  {
    if (handlers.runMicProbe != nullptr)
    {
      handlers.runMicProbe(cmd);
    }
  }
  else if (cmd.startsWith("micch"))
  {
    if (handlers.setMicChannelFromCommand != nullptr)
    {
      handlers.setMicChannelFromCommand(cmd);
    }
  }
  else if (cmd.startsWith("micshift"))
  {
    if (handlers.setMicShiftFromCommand != nullptr)
    {
      handlers.setMicShiftFromCommand(cmd);
    }
  }
  else if (cmd.startsWith("micmode"))
  {
    if (handlers.setMicModeFromCommand != nullptr)
    {
      handlers.setMicModeFromCommand(cmd);
    }
  }
  else if (cmd.startsWith("ffttest"))
  {
    int malletIndex = 0;
    const int separator = cmd.indexOf(' ');
    if (separator >= 0 && separator + 1 < static_cast<int>(cmd.length()))
    {
      malletIndex = cmd.substring(separator + 1).toInt();
    }
    if (handlers.runFftTestForMallet != nullptr)
    {
      handlers.runFftTestForMallet(malletIndex);
    }
  }
  else if (cmd.startsWith("setnote"))
  {
    if (handlers.setMalletMidiFromCommand != nullptr)
    {
      handlers.setMalletMidiFromCommand(cmd);
    }
  }
  else if (cmd.startsWith("setstrike"))
  {
    if (handlers.setMalletStrikeFromCommand != nullptr)
    {
      handlers.setMalletStrikeFromCommand(cmd);
    }
  }
  else if (cmd.startsWith("setrebound"))
  {
    if (handlers.setMalletReboundFromCommand != nullptr)
    {
      handlers.setMalletReboundFromCommand(cmd);
    }
  }
  else if (cmd == "loopstats")
  {
    if (handlers.printLoopStats != nullptr)
    {
      handlers.printLoopStats();
    }
  }
  else if (cmd == "calrebound")
  {
    if (handlers.runReboundCalibration != nullptr)
    {
      handlers.runReboundCalibration();
    }
  }
  else if (cmd.startsWith("calrebound "))
  {
    String malletToken = cmd.substring(11);
    malletToken.trim();
    const int malletIndex = malletToken.toInt();
    if (malletIndex < 0 || malletIndex >= static_cast<int>(malletCount))
    {
      out.print("calrebound mallet index out of range 0..");
      out.println(static_cast<int>(malletCount) - 1);
    }
    else if (handlers.runReboundCalibrationForMallet != nullptr)
    {
      handlers.runReboundCalibrationForMallet(static_cast<size_t>(malletIndex));
    }
  }
  else if (cmd == "balance")
  {
    if (handlers.toggleVolumeBalance != nullptr)
    {
      handlers.toggleVolumeBalance();
    }
  }
}

} // namespace SerialCommands
