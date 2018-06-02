#!/usr/bin/env python

import time
import rospy
from Deca_device import DecaDevice
from decaros.msg import ControlSignal
from decaros.msg import ControlSignalReply

SEND_POLL       = 1
SEND_POLL_ACK   = 2
SEND_RANGE      = 3
BROADCAST       = 0xFF
TIMEOUT_TIME    = 1
RATE            = 100

def init():
    global anchorList, tagList
    global controlPub, control, sequence
    global commandExecuted, expectedControlReply
    anchorList      = {}
    tagList         = {}
    controlPub      = rospy.Publisher('control_signal', ControlSignal, queue_size=10)
    control         = ControlSignal()
    sequence        = 1
    commandExecuted = False
    expectedControlReply = ControlSignalReply()
    rospy.Subscriber('control_reply', ControlSignalReply, controlSignalReplyCB)

def controlSignalReplyCB(reply):
    commandExecuted = (reply == expectedControlReply)

def populateDeviceLists(ANCHORS, TAGS):
    for i in ANCHORS:
        anchorList[i] = DecaDevice(i, DecaDevice.ANCHOR)
    for i in TAGS:
        tagList[i] = DecaDevice(i, DecaDevice.TAG)

def registerExepectedReply(controlsignal):
    expectedControlReply.sender     = controlsignal.sender
    expectedControlReply.signal     = controlsignal.signal
    expectedControlReply.sequence   = controlsignal.sequence

def sendControlSignal(seq, signal, sender, receiver=BROADCAST):
    control.sequence    = seq
    control.signal      = signal
    control.sender      = sender
    control.receiver    = receiver
    controlPub.publish(control)
    # print "{}, {}, {}, {}".format(control.sequence, control.signal, control.sender, control.receiver)
    registerExepectedReply(control)

def wait(sentTime):
    while not commandExecuted and (time.time()-sentTime)<TIMEOUT_TIME:
        pass

def startRanging():
    for tagAddress in tagList:
        sendControlSignal(sequence, SEND_POLL, tagAddress)
        wait(time.time())

    for anchorAddress in anchorList:
        sendControlSignal(sequence, SEND_POLL_ACK, anchorAddress)
        wait(time.time())

    for tagAddress in tagList:
        for anchorAddress in anchorList:
            sendControlSignal(sequence, SEND_RANGE, tagAddress, anchorAddress)
            wait(time.time())

def spin():
    global sequence
    rospy.loginfo("Finding Global Positions")
    rate = rospy.Rate(RATE)
    rospy.on_shutdown(shutdown)
    while not rospy.is_shutdown():
        startRanging();
        sequence += 1
        rate.sleep()
    rospy.spin()

def shutdown():
    rospy.loginfo("Shutting down.")
    rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('central_node', anonymous=True)
    init()
    ANCHORS     = map(int, rospy.get_param("~anchors").split(","))
    TAGS        = map(int, rospy.get_param("~tags").split(','))
    print ANCHORS
    print TAGS
    populateDeviceLists(ANCHORS, TAGS)
    for i in anchorList:
        print anchorList[i]
    for i in tagList:
        print tagList[i]

    try:
        spin()
    except rospy.ROSInterruptException:
        print("Closing...")
        pass