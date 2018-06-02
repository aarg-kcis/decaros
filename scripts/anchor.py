import rospy
import DW1000Constants as C
from Deca_device import DecaDevice
from decaros.msg import ControlSignal
from decaros.msg import ControlSignalReply

SEND_POLL       = 1
SEND_POLL_ACK   = 2
SEND_RANGE      = 3
BROADCAST       = 0xFF
TIMEOUT_TIME    = 1
RATE            = 100
NODE_TYPE       = 1 # ANCHOR

def init():
    global replyPub, reply, sequence, lastSignalServiced, timestampPub
    global timePollReceived, timePollAckSent, timeRangeReceived
    timePollReceived       = {}
    timePollAckSent        = {}
    timeRangeReceived      = {}
    replyPub            = rospy.Publisher('control_reply', ControlSignalReply, queue_size=10)
    timestampPub        = rospy.Publisher('anchor_timestamps', AnchorTimeStamps, queue_size=10)
    reply               = ControlSignalReply()
    sequence            = 0
    lastSignalServiced  = None
    rospy.Subscriber('control_signal', ControlSignal, controlSignalCB)

def controlSignalCB(signal):
    global sequence
    if signal.sender == MY_ADDRESS:
        if signal.signal == SEND_POLL_ACK:
            transmitPollAck(signal.sequence)

def getTimeStampForSequence(seq):
    ts = AnchorTimeStamps()
    ts.id                   = MY_ADDRESS
    ts.sequence             = seq
    ts.timePollReceived     = timePollReceived[seq]
    ts.timePollAckSent      = timePollAckSent[seq]
    ts.timeRangeReceived    = timeRangeReceived[seq]
    return ts

def deletePrevTimeStamps(seq):
    timePollReceived    = {seq: timePollReceived[seq]}
    timePollAckSent     = {seq: timePollAckSent[seq]}
    timeRangeReceived   = {seq: timeRangeReceived[seq]}


def checkFlags():
    if sentFlag:
        reply.sender    = MY_ADDRESS
        reply.sequence  = sequence
        reply.signal    = lastSignalServiced
        if  lastSignalServiced == SEND_POLL_ACK:
            timePollAckSent[sequence]  = DW1000.getTransmitTimestamp()
        replyPub.publish(reply)
    elif receivedFlag:
        msgType, sender, sequence, node_type = DW1000.getData(4)
        if node_type == NODE_TYPE:
            return
        if msgType == C.POLL:
            timePollReceived[sequence] = DW1000.getReceiveTimestamp()
        elif msgType == C.RANGE
            timeRangeReceived[sequence] = DW1000.getReceiveTimestamp()
            timestampPub.publish(getTimeStampForSequence(sequence))
            deletePrevTimeStamps(sequence)

def transmitPollAck(sequence):
    DW1000.newTransmit()
    DW1000.setData([C.POLL_ACK, MY_ADDRESS, sequence, NODE_TYPE], 4)
    DW1000.startTransmit()
    lastSignalServiced = SEND_POLL

def spin():
    global sequence
    rospy.loginfo("Finding Global Positions")
    rate = rospy.Rate(RATE)
    rospy.on_shutdown(shutdown)
    while not rospy.is_shutdown():
        checkFlags();
        sequence += 1
        rate.sleep()
    rospy.spin()

def shutdown():
    rospy.loginfo("Shutting down.")
    rospy.sleep(1)

if __name__ == '__main__':
	node_name = "anchor_{}".format()
    rospy.init_node(node_name)
    MY_ADDRESS  = rospy.get_param("~id")
    print("Anchor Address: {}".format(MY_ADDRESS))
    init()
    try:
        spin()
    except rospy.ROSInterruptException:
        print("Closing...")
        pass