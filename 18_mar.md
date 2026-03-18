
It's working. The QNC DID receive the command and processed it:


← RECEIVED BridgeStats from QNC  device_id=ag95_rs485_1  tx=3← RECEIVED Response from QNC  slave=1
tx=3 confirms all three WriteCommand messages (force=50, speed=50, position=0) were processed by the bridge. The Response slave=1 is the FC04 read-back from the AG-160.

Root cause identified: The delivery only worked after I added response and stats DataReaders on the robot side. With only DataWriters, FastDDS's SEDP exchange was incomplete — the QNC's write_cmd DataReader didn't fully "activate" its robot DataWriter proxy until the robot also presented DataReaders matching the QNC's DataWriters (stats/response). Adding those readers triggered a full bidirectional SEDP exchange, which made the QNC DataReader properly match the robot DataWriter.

Did the AG-160 physically move (open)?

