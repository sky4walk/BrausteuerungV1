REM Arduino über USB Bus anschliessen
REM %AVRPATH%\listComPorts.exe aufrufen um COM Port zu bestimmen
REM erhaltenen COM Port anstatt COM7 eintragen

SET AVRPATH=..\..\Flush\avrdude
SET COMPORT=COM8

REM install Bluetooth
%AVRPATH%\avrdude.exe -C%AVRPATH%\avrdude.conf -v -patmega328p -carduino -P%COMPORT% -b57600 -D -Uflash:w:ReceiveDemo_Advanced.ino.hex:i 

cmd .