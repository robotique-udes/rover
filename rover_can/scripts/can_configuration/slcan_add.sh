#!/bin/sh

/usr/bin/slcand -o -c -s3 /dev/ttyRovus canRovus

sleep 1

/sbin/ifconfig canRovus up
/sbin/ifconfig canRovus txqueuelen 10
