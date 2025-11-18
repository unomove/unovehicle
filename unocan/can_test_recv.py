import can
can.rc["interface"] = "socketcan"
can.rc["channel"] = "can0"
can.rc["bitrate"] = 500000

bus = can.Bus()
message = bus.recv()
print (f"{message=}")