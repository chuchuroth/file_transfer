The robot has multiple network interfaces (WiFi 172.28.3.58, docker0 192.168.128.1, virbr0 192.168.122.1). FastDDS is binding to all of them — the QNC's unicast PDP arrives on the WiFi interface, robot responds, but data traffic likely goes out via docker0/virbr0 which can't reach the QNC. Fix: whitelist only the WiFi interface in the XML, and increase the post-publish drain sleep in the destructor.


Now rewrite the unicast XML to pin FastDDS exclusively to the WiFi interface:



Sent. The key changes:

useBuiltinTransports=false + interfaceWhiteList: 172.28.3.58 — FastDDS now exclusively uses the WiFi interface. Previously it was binding to all interfaces (docker0, virbr0, wlp0s20f3) — discovery likely arrived via WiFi but data was sent out via docker0 which can't reach the QNC
3s destructor sleep (up from 500ms) — keeps the DataWriter alive longer so TRANSIENT_LOCAL cache can be drained to the QNC even with slightly slow discovery
Did the bridge receive [CMD] write_cmd slave=1 this time?
