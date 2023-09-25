Dependencies
--------

**Mac**

```
xcode-select --install
./get_sdk_mac.sh
```

**Debian / Ubuntu**

```
./get_sdk.sh
```


Programming
----
Ocelot Pro: Press and Hold SW1 while plugging in to get into DFU mode for flashing.

**Panda**

```
scons -u # Compile
./flash.sh # Compile & Flash
```

**SmartDSU**

```
cd smart_dsu
./recover.sh # or ./recover_usb.sh
```

**Panda**

```
scons -u # Compile
./flash.sh # Compile & Flash
```

Troubleshooting
----

If your panda will not flash and is quickly blinking a single Green LED, use:
```
./recover.sh
```


[dfu-util](http://github.com/dsigma/dfu-util.git) for flashing
