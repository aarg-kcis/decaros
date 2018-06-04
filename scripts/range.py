#!/usr/bin/env python

import rospy
import DW1000
import DW1000Constants as C
from Deca_device import DecaDevice
from decaros.msg import AnchorTimeStamps
from decaros.msg import TagTimeStamps

Tag_seq 		= 0
Anchor_seq 		= 0
RATE            = 100

def init():
    global sequence
    global timePollReceived, timePollAckSent, timeRangeReceived,timePollSent,timePollAckReceived,timeRangeSent
    anchor_time_msg     = {}
    tag_time_msg        = {}
    current_sequence    = 0
    rospy.Subscriber('Tag_TimeStamps', TagTimeStamps, TagTimeStampsCB)
    rospy.Subscriber('Anchor_TimeStamps',AnchorTimeStamps, AnchorTimeStampsCB)

def TagTimeStampsCB(Time_msg):
	global tag_time_msg
	tag_time_msg[Time_msg.sequence] = Time_msg
	print Time_msg

def AnchorTimeStampCB(Time_msg):
	global anchor_time_msg
	anchor_time_msg[Time_msg.sequence] = Time_msg
	print Time_msg

def deletePreviousSequenceData():
    for i in tag_time_msg:
        for j in i.keys():
            if j != tag_time_msg.sequence-1:
                del i[j]

   	for i in anchor_time_msg:
   		for j in i.keys():
   			if j != anchor_time_msg.sequence - 1 :
   				del i[j]


def getrange():
	global current_sequence
	if tag_time_msg.sequence == anchor_time_msg.sequence :
		current_sequence = tag_time_msg.sequence
		round1 = DW1000.wrapTimestamp(tag_time_msg[current_sequence].timePollAckReceived - tag_time_msg[current_sequence].timePollSent)
        reply1 = DW1000.wrapTimestamp(anchor_time_msg[current_sequence].timePollAckSent - anchor_time_msg[current_sequence].timePollReceived)
        round2 = DW1000.wrapTimestamp(anchor_time_msg[current_sequence].timeRangeReceived - anchor_time_msg[current_sequence].timePollAckSent)
        reply2 = DW1000.wrapTimestamp(tag_time_msg[current_sequence].timeRangeSent - tag_time_msg[current_sequence].timePollAckReceived)
        print "round1: {}\t\t reply1:{}".format(round1, reply1)
        print "round2: {}\t\t reply2:{}".format(round2, reply2)
        deletePreviousSequenceData()
        range1 = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2)
        return (range1 % C.TIME_OVERFLOW) * C.DISTANCE_OF_RADIO


def spin():
    rospy.loginfo("Finding Global Positions")
    rate = rospy.Rate(RATE)
    rospy.on_shutdown(shutdown)
    while not rospy.is_shutdown():
        print getrange()
        rate.sleep()
    rospy.spin()

def shutdown():
    rospy.loginfo("Shutting down.")
    rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node("range", anonymous=True)
    try:
        spin()
    except rospy.ROSInterruptException:
        print("Closing...")
        pass
