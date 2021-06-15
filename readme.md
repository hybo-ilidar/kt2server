# Version of KT2SERVER which runs on Linux

Uses libserialport from sigrok.
Download and install per directions on their repository:  

* [https://github.com/martinling/libserialport](https://github.com/martinling/libserialport)

1. Build using script file `doit.sh`

2. Put the name of the comport in the file `comport.txt`. For example,

```bash
$ echo '/dev/ttyUSB0' > comport.txt
```
3. Run the server

```bash
$ kt2server
```

## Notes from Mr. Kim 13 May

보내주신 UDP 소스코드(sensor.cpp)에서 아래 부분으로 수정하는 것이 맞는
것인지 확인 부탁드립니다.

Please check if it is correct to modify the UDP source code (sensor.cpp) below.
(corrected line 111, printing `nframes_rcvd`)


추가로 Receive 되는 것에 비해 Sent 이벤트 발생이 작은 것 같은데
정상적으로 동작하는 것인지도 확인 부탁드리겠습니다.

In addition, the Sent event seems to be smaller than the receiving
event, so please check if it works normally.

## Notes from Dr. Son 15 Jun 2021 (CRC update)

The change is mainly done to eliminate the wrongly-received rows, which
are shown as noticeable lines on the display.

1. A checksum data is appended after each row.

2. It is read by the VS project program. If the checksum is not matched,
   the program will not update corresponding row in the display memory.

3. Therefore, the corresponding region of the previous image will be
   kept if something is wrong with the received row.

4. Noticeable defective lines (such as all-black) will be removed.
   (there will be still some artifacts in dynamic scenes)

Please update the serial to socket converter program for KT.
In your case you will not send a Ethernet packet if the checksum 
matching is failed.

