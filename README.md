# file_transfer




  Serial port  : /dev/ttyAMA0
  DE/RE GPIO   : none
  DDS domain   : 0
  Stats interval: every 10 writes

✓ RS485 open: /dev/ttyAMA0
[DDS] QncBridge active on domain 0
      SUB: qnc/modbus/write_cmd  qnc/modbus/read_cmd
      PUB: qnc/modbus/response   qnc/modbus/stats
[Bridge] Discovery pause 1 s...
[Bridge] Waiting for commands (Ctrl-C to stop)...
[CMD] write_cmd slave=1 reg=0x103 val=0 tag=open
[CMD] read_cmd  slave=1 start=0x200 count=3 fc=FC04 tag=pos_open
[CMD] write_cmd slave=1 reg=0x103 val=1000 tag=close
[CMD] read_cmd  slave=1 start=0x200 count=3 fc=FC04 tag=pos_close
[CMD] write_cmd slave=1 reg=0x103 val=0 tag=open
[CMD] read_cmd  slave=1 start=0x200 count=3 fc=FC04 tag=pos_open
[CMD] write_cmd slave=1 reg=0x103 val=1000 tag=close
[CMD] read_cmd  slave=1 start=0x200 count=3 fc=FC04 tag=pos_close
[CMD] write_cmd slave=1 reg=0x103 val=0 tag=open
