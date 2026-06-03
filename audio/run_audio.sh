#!/bin/bash

FIFO="/tmp/audio_fifo"
AUDIO_DIR="/root/audio"
CARD="plughw:1,0"

if [ ! -p "$FIFO" ]; then
    mkfifo "$FIFO"
    chmod 666 "$FIFO"
fi

echo "[AudioDaemon] started, listening on $FIFO"
exec 3<> "$FIFO"

while read -r line <&3; do
    echo "[AudioDaemon] alert command: $line"
    case "$line" in
        1) aplay -D "$CARD" -q "$AUDIO_DIR/fire_alert.wav" ;;
        2) aplay -D "$CARD" -q "$AUDIO_DIR/no_vest.wav" ;;
        3) aplay -D "$CARD" -q "$AUDIO_DIR/no_helmet.wav" ;;
        4) aplay -D "$CARD" -q "$AUDIO_DIR/intrusion.wav" ;;
        *) echo "[AudioDaemon] unknown command: $line" ;;
    esac
done
