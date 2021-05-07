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
