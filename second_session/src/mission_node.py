#!/usr/bin/python3

from second_session.srv import GetNextDestination, GetNextDestinationResponse, GetNextDestinationRequest
import rospy, random, math


def GetRandomDestination(req : GetNextDestinationRequest):
    
    res = GetNextDestinationResponse()
    
    res.next_x = res.next_y = 30

    while -20 > res.next_x or res.next_x > 20 or -20 > res.next_y  or res.next_y > 20:
        
        r = random.uniform(10,20)
        teta = random.uniform(0, 2*math.pi)

        x = req.current_x + r*math.cos(teta)
        y = req.current_y + r*math.sin(teta)

        res.next_x = x
        res.next_y = y

    print("New destination")
    return res

rospy.init_node('mission_node', anonymous=True)


s = rospy.Service('/mammad', GetNextDestination, GetRandomDestination)


rospy.spin()
