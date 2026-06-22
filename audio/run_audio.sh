#!/bin/bash

FIFO="/tmp/audio_fifo"
AUDIO_DIR="/home/why/rk3588/rkproject/ELF2-feature-ai_project/0602/audio"
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
        1) aplay -D "$CARD" -q "$AUDIO_DIR/火灾.wav" ;;
        2) aplay -D "$CARD" -q "$AUDIO_DIR/工作区1.wav" ;;
        3) aplay -D "$CARD" -q "$AUDIO_DIR/工作区2.wav" ;;
        4) aplay -D "$CARD" -q "$AUDIO_DIR/防护服或安全帽.wav" ;;
        *) echo "[AudioDaemon] unknown command: $line" ;;
    esac
done
