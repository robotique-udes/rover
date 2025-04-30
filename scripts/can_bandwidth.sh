#!/bin/bash

IFACE="canRovus"
MAX_BW=0
CAN_MAX_BPS=1000000  # 1 Mbps
AVG_BW=0
COUNTER=0
TOTAL_BW=0

while true; do
    RX1=$(cat /sys/class/net/$IFACE/statistics/rx_bytes)
    TX1=$(cat /sys/class/net/$IFACE/statistics/tx_bytes)
    TIME1=$(date +%s%N)

    sleep 0.01

    RX2=$(cat /sys/class/net/$IFACE/statistics/rx_bytes)
    TX2=$(cat /sys/class/net/$IFACE/statistics/tx_bytes)
    TIME2=$(date +%s%N)

    RX_BYTES=$((RX2 - RX1))
    TX_BYTES=$((TX2 - TX1))
    TOTAL_BYTES=$((RX_BYTES + TX_BYTES))
    TIME_DELTA_NS=$((TIME2 - TIME1))

    if [ $TIME_DELTA_NS -gt 0 ]; then
        BW=$((TOTAL_BYTES * 8000000000 / TIME_DELTA_NS))
        [ $BW -gt $MAX_BW ] && MAX_BW=$BW

        PERCENT=$((BW * 100 / CAN_MAX_BPS))
        MAX_PERCENT=$((MAX_BW * 100 / CAN_MAX_BPS))

        # Add current bandwidth to total for averaging
        TOTAL_BW=$((TOTAL_BW + BW))
        COUNTER=$((COUNTER + 1))

        # If 5 seconds passed, calculate average and reset counters
        if [ $COUNTER -ge 5 ]; then
            AVG_BW=$((TOTAL_BW / COUNTER))
            COUNTER=0
            TOTAL_BW=0
        fi

        # Convert to Mbps with 3 decimals
        BW_Mbps=$(printf "%.3f" "$(echo "$BW / 1000000" | bc -l)")
        MAX_BW_Mbps=$(printf "%.3f" "$(echo "$MAX_BW / 1000000" | bc -l)")
        AVG_BW_Mbps=$(printf "%.3f" "$(echo "$AVG_BW / 1000000" | bc -l)")

        echo "Now: ${BW_Mbps} Mbps (${PERCENT}%) | Max: ${MAX_BW_Mbps} Mbps (${MAX_PERCENT}%) | Avg (5s): ${AVG_BW_Mbps} Mbps"
    fi
done
