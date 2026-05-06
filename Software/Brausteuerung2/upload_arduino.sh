#!/bin/bash
# ============================================================
# upload_arduino.sh
# Flasht die Brausteuerung auf einen Arduino UNO via avrdude.
# Port wird automatisch erkannt (ttyUSB* oder ttyACM*).
# ============================================================

set -e

# ------------------------------------------------------------
# Konfiguration
# ------------------------------------------------------------
MCU="atmega328p"
PROGRAMMER="arduino"
BAUD="115200"

# Pfad relativ zum Skript-Verzeichnis
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIRMWARE="$SCRIPT_DIR/build/arduino.avr.uno/Brausteuerung2.ino.hex"

# ------------------------------------------------------------
# Firmware pruefen
# ------------------------------------------------------------
if [ ! -f "$FIRMWARE" ]; then
  echo "[FEHLER] Hex-Datei nicht gefunden:"
  echo "         $FIRMWARE"
  echo ""
  echo "         Bitte zuerst in der Arduino IDE kompilieren:"
  echo "         Sketch -> Kompilieren  (Strg+R)"
  exit 1
fi

echo "[INFO] Firmware: $FIRMWARE"

# ------------------------------------------------------------
# Port automatisch erkennen
# ------------------------------------------------------------
echo "[INFO] Suche Arduino-Port..."

PORTS=()
for p in /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$p" ] && PORTS+=("$p")
done

if [ ${#PORTS[@]} -eq 0 ]; then
  echo "[FEHLER] Kein Arduino-Port gefunden (ttyUSB* / ttyACM*)."
  echo "         Ist der Arduino angeschlossen?"
  echo "         Tipp: sudo usermod -aG dialout \$USER  (dann neu einloggen)"
  exit 1
elif [ ${#PORTS[@]} -eq 1 ]; then
  PORT="${PORTS[0]}"
  echo "[INFO] Port gefunden: $PORT"
else
  echo "[INFO] Mehrere Ports gefunden:"
  for i in "${!PORTS[@]}"; do
    echo "  [$i] ${PORTS[$i]}"
  done
  read -rp "Port-Nummer waehlen [0]: " CHOICE
  CHOICE="${CHOICE:-0}"
  PORT="${PORTS[$CHOICE]}"
  echo "[INFO] Gewaehlt: $PORT"
fi

# ------------------------------------------------------------
# Schreibrecht pruefen
# ------------------------------------------------------------
if [ ! -w "$PORT" ]; then
  echo "[FEHLER] Kein Schreibrecht auf $PORT"
  echo "         Ausfuehren: sudo usermod -aG dialout \$USER"
  echo "         Danach neu einloggen oder einmalig: sudo chmod a+rw $PORT"
  exit 1
fi

# ------------------------------------------------------------
# Flash
# ------------------------------------------------------------
echo ""
echo "[INFO] Flashe -> $PORT ($MCU @ ${BAUD} Baud)"
echo "------------------------------------------------------------"

avrdude \
  -v \
  -p "$MCU" \
  -c "$PROGRAMMER" \
  -P "$PORT" \
  -b "$BAUD" \
  -D \
  -U "flash:w:${FIRMWARE}:i"

echo ""
echo "[OK] Upload abgeschlossen."
