#!/usr/bin/env python

import rospy
import DW1000
import DW1000Constants as C
from Deca_device import DecaDevice
from decaros.msg import ControlSignal
from decaros.msg import ControlSignalReply
from decaros.msg import AnchorTimeStamps

SEND_POLL       = 1
SEND_POLL_ACK   = 2
SEND_RANGE      = 3
BROADCAST       = 0xFF
TIMEOUT_TIME    = 1
RATE            = 100
NODE_TYPE       = 1 # ANCHOR
sentFlag        = False
receivedFlag    = False

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
    lastSignalServiced  = SEND_POLL
    rospy.Subscriber('control_signal', ControlSignal, controlSignalCB)

def initDW1000():
    PIN_IRQ = 19
    PIN_SS = 16
    DW1000.begin(PIN_IRQ)
    DW1000.setup(PIN_SS)
    DW1000.generalConfiguration("7D:00:22:EA:82:60:3B:0C", C.MODE_LONGDATA_FAST_ACCURACY)
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
    print "Received signal"
    print signal
    global sequence
    if signal.sender == MY_ADDRESS:
        if signal.signal == SEND_POLL_ACK and signal.sequence in timePollReceived.keys():
            sequence = signal.sequence
            transmitPollAck(sequence)

def getTimeStampForSequence(seq):
    ts = AnchorTimeStamps()
    ts.id                   = MY_ADDRESS
    ts.sequence             = seq
    ts.timePollReceived     = timePollReceived[seq]
    ts.timePollAckSent      = timePollAckSent[seq]
    ts.timeRangeReceived    = timeRangeReceived[seq]
    return ts

def deletePrevTimeStamps(seq):
    global timePollReceived, timePollAckSent, timeRangeReceived
    timePollReceived    = {seq: timePollReceived[seq]}
    timePollAckSent     = {seq: timePollAckSent[seq]}
    timeRangeReceived   = {seq: timeRangeReceived[seq]}


def checkFlags():
    global sequence, sentFlag, receivedFlag
    if sentFlag:
        print "Sent something"
        sentFlag = False
        reply.sender    = MY_ADDRESS
        reply.sequence  = sequence
        reply.signal    = lastSignalServiced
        if  lastSignalServiced == SEND_POLL:
            timePollAckSent[sequence]  = DW1000.getTransmitTimestamp()
            print "Poll Ack sent for {}".format(sequence)
            replyPub.publish(reply)
    elif receivedFlag:
        receivedFlag = False
        print "Received something"
        msgType, sender, sequence, node_type = DW1000.getData(4)
        if node_type == NODE_TYPE:
            return
        if msgType == C.POLL:
            print "Poll received for {}".format(sequence)
            timePollReceived[sequence] = DW1000.getReceiveTimestamp()
        elif msgType == C.RANGE:
            print "Range received for {}".format(sequence)
            timeRangeReceived[sequence] = DW1000.getReceiveTimestamp()
            timestampPub.publish(getTimeStampForSequence(sequence))
            deletePrevTimeStamps(sequence)

def transmitPollAck(sequence):
    print "transmitting POLLACK"
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
    rospy.init_node("anchor", anonymous=True)
    MY_ADDRESS  = rospy.get_param("~id")
    print("Anchor Address: {}".format(MY_ADDRESS))
    init()
    initDW1000()
    try:
        spin()
    except rospy.ROSInterruptException:
        print("Closing...")
        pass