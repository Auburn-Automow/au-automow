#!/usr/bin/env python
# encoding: utf-8

"""
osc_broadcaster.py
"""

import sys, liblo, pybonjour
import select

import roslib; roslib.load_manifest('osc_monitor')
import rospy

from power_control_board.msg import PowerControl,CutterControl

resolved = []
clients = {}
timeout = 5

def resolve_callback(sdRef, flags, interfaceIndex, errorCode, fullname, hosttarget, port, txtRecord):
    if errorCode == pybonjour.kDNSServiceErr_NoError:
        clients[hosttarget] = port
        resolved.append(True)

def browse_callback(sdRef, flags, interfaceIndex, errorCode, serviceName, regtype, replyDomain):
    if errorCode != pybonjour.kDNSServiceErr_NoError:
        return
    if not (flags & pybonjour.kDNSServiceFlagsAdd):
        print 'Service removed'
        return
    if serviceName == "ROS OSC Service":
        return

    print 'Service added; resolving'

    resolve_sdRef = pybonjour.DNSServiceResolve(0,
                                                interfaceIndex,
                                                serviceName,
                                                regtype,
                                                replyDomain,
                                                resolve_callback)

    try:
        while not resolved:
            ready = select.select([resolve_sdRef], [], [], timeout)
            if resolve_sdRef not in ready[0]:
                print 'Resolve timed out'
                break
            pybonjour.DNSServiceProcessResult(resolve_sdRef)
        else:
            resolved.pop()
    finally:
        resolve_sdRef.close()


def pcb_oscPublish(data):
    """ Publishes Power Control Data to the OSC protocol """
    bundle = liblo.Bundle()
    
    if data.StateofCharge <= 30:
        color = 'red'
    elif data.StateofCharge <= 45:
        color = 'orange'
    elif data.StateofCharge <= 60:
        color = 'yellow'
    else:
        color = 'green'
    
    bundle.add(liblo.Message("/1/ChargeLabel","Charge: " + str(data.StateofCharge)+"%"))
    bundle.add(liblo.Message("/1/Charge/color",color))
    bundle.add(liblo.Message("/1/Charge",data.StateofCharge/100.0))
    bundle.add(liblo.Message("/2/ChargeLabel","Charge: " + str(data.StateofCharge)+"%"))
    bundle.add(liblo.Message("/2/Charge/color",color))
    bundle.add(liblo.Message("/2/Charge",data.StateofCharge/100.0))
    bundle.add(liblo.Message("/1/Voltage",data.Voltage))
    bundle.add(liblo.Message("/1/VoltageLabel","Voltage: " +str(data.Voltage)))
    bundle.add(liblo.Message("/1/Current",abs(data.Current)))
    bundle.add(liblo.Message("/1/ChassisTempTop",data.Temperature1))
    bundle.add(liblo.Message("/1/ChassisTempBottom",data.Temperature2))
    bundle.add(liblo.Message("/2/CutterLeft",data.LeftCutterStatus))
    bundle.add(liblo.Message("/2/CutterRight",data.RightCutterStatus))
    for host,port in clients.iteritems():
        try:
            target = liblo.Address(host,port)
        except liblo.AddressError, err:
            print str(err)
        liblo.send(target,bundle)

def main():
    rospy.init_node("OSC_Broadcaster")
    pcb_subscriber = rospy.Subscriber("PowerControl",PowerControl,pcb_oscPublish)


    sdRef = pybonjour.DNSServiceRegister(name = "ROS OSC Service",
                                        regtype = "_osc._udp",
                                        port = 8000)
    
    browse_sdRef = pybonjour.DNSServiceBrowse(regtype = "_osc._udp",
                                            callBack = browse_callback)

    while not rospy.is_shutdown():
        ready = select.select([browse_sdRef],[],[])
        if browse_sdRef in ready[0]:
            pybonjour.DNSServiceProcessResult(browse_sdRef)


        pass

    sdRef.close()
    browse_sdRef.close()

if __name__ == '__main__':
    main()
