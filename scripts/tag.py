#!/usr/bin/env python

import rospy
import DW1000
import DW1000Constants as C
from Deca_device import DecaDevice
from decaros.msg import ControlSignal
from decaros.msg import ControlSignalReply
from decaros.msg import TagTimeStamps

SEND_POLL       = 1
SEND_POLL_ACK   = 2
SEND_RANGE      = 3
BROADCAST       = 0xFF
TIMEOUT_TIME    = 1
RATE            = 100
NODE_TYPE       = 0 # TAG
sentFlag        = False
receivedFlag    = False

def init():
    global replyPub, reply, sequence, lastSignalServiced, timestampPub
    global timePollSent, timePollAckReceived, timeRangeSent
    timePollSent        = {}
    timePollAckReceived = {}
    timeRangeSent       = {}
    replyPub            = rospy.Publisher('control_reply', ControlSignalReply, queue_size=10)
    timestampPub        = rospy.Publisher('tag_timestamps', TagTimeStamps, queue_size=10)
    reply               = ControlSignalReply()
    sequence            = 0
    lastSignalServiced  = None
    rospy.Subscriber('control_signal', ControlSignal, controlSignalCB)

def initDW1000():
    PIN_IRQ = 19
    PIN_SS = 16
    DW1000.begin(PIN_IRQ)
    DW1000.setup(PIN_SS)
    DW1000.generalConfiguration("7D:00:22:EA:82:60:3B:9C", C.MODE_LONGDATA_FAST_ACCURACY)
    DW1000.registerCallback("handleSent", handleSent)
    DW1000.registerCallback("handleReceived", handleReceived)
    DW1000.setAntennaDelay(C.ANTENNA_DELAY_RASPI)
    receiver()

def handleSent():
    global sentFlag
    sentFlag = True

def handleReceived():
    global receivedFlag
    receivedFlag = True

def receiver():
    print "Initializing receiver"
    DW1000.newReceive()
    DW1000.receivePermanently()
    DW1000.startReceive()

def controlSignalCB(signal):
    global sequence
    print "Received signal"
    print signal
    if signal.sender == MY_ADDRESS:
        sequence = signal.sequence
        if signal.signal == SEND_POLL:
            transmitPoll(signal.sequence)
        elif signal.signal == SEND_RANGE and signal.sequence in timePollSent.keys():
            transmitRange(signal.sequence)

def getTimeStampForSequence(seq):
    ts = AnchorTimeStamps()
    ts.id                   = MY_ADDRESS
    ts.sequence             = seq
    ts.timePollSent         = timePollSent[seq]
    ts.timePollAckReceived  = timePollAckReceived[seq]
    ts.timeRangeSent        = timeRangeSent[seq]
    return ts

def deletePrevTimeStamps(seq):
    timePollSent        = {seq: timePollSent[seq]}
    timePollAckReceived = {seq: timePollAckReceived[seq]}
    timeRangeSent       = {seq: timeRangeSent[seq]}

def checkFlags():
    global sequence
    if sentFlag:
        print "Sent message"
        reply.sender    = MY_ADDRESS
        reply.sequence  = sequence
        reply.signal    = lastSignalServiced
        if  lastSignalServiced == SEND_POLL:
            timePollSent[sequence]  = DW1000.getTransmitTimestamp()
        elif lastSignalServiced == SEND_RANGE:
            timeRangeSent[sequence] = DW1000.getTransmitTimestamp()
            timestampPub.publish(getTimeStampForSequence(sequence))
            deletePrevTimeStamps(sequence)
        replyPub.publish(reply)
    elif receivedFlag:
        print "Received message"
        msgType, sender, sequence, node_type = DW1000.getData(4)
        if node_type == NODE_TYPE:
            return
        timePollAckReceived[sequence] = DW1000.getReceiveTimestamp()

def transmitPoll(sequence):
    print "Transmitting POLL"
    DW1000.newTransmit()
    DW1000.setData([C.POLL, MY_ADDRESS, sequence, NODE_TYPE], 4)
    DW1000.startTransmit()
    lastSignalServiced = SEND_POLL

def transmitRange(address):
    print "Transmitting RANGE"
    DW1000.newTransmit()
    DW1000.setData([C.RANGE, MY_ADDRESS, sequence, NODE_TYPE], 4)
    DW1000.startTransmit()
    lastSignalServiced = SEND_RANGE

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
    rospy.init_node("tag", anonymous=True)
    MY_ADDRESS  = rospy.get_param("~id")
    print("Tag Address: {}".format(MY_ADDRESS))
    init()
    initDW1000()
    try:
        spin()
    except rospy.ROSInterruptException:
        print("Closing...")
        pass
