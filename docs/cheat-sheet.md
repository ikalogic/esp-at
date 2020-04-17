 
 `make flash ESPBAUD=921600 ESPPORT=COM9` to make and load into flash

`make factory_bin` to merge all *bin files into one that will appear in `build/factory`

# Custom events added

+CBLESCAN:name,MA:CA:DD:RE:SS:SS
+CBLEDISCONNECT:1234(handle)
+CBLECONNECT:1234(handle)
+CBLECONNECT: Failed (with some reason tesst)
+CBLEWRITE: 0(status)