# -*- coding: utf-8 -*-
#!/usr/bin/env python
import sys
import rospy
import urllib
import urllib2
import httplib
import json
import requests


# url = "http://192.168.12.20:8080/v2.0.0/mission_queue"

# url_2 = 'http://192.168.12.20:8080/v2.0.0/positions'

def mir_move_goal(goal_id):
    url = "http://192.168.12.20:8080/v2.0.0/mission_queue"
    payload_head = "{\r\n  \"mission_id\": \"29629d35-4f36-11e9-b7ba-94c691a737d7\",\r\n  \"message\": \"move to\",\r\n  \"parameters\": [{\"input_name\": \"goal\", \"value\": \""
    payload_end = "\"}],\r\n  \"priority\": 0\r\n}"
    payload = payload_head + goal_id + payload_end

    headers = {
        'authorization': "Basic RGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA==",
        'content-type': "application/json",
        'cache-control': "no-cache"
        }
    try:
        response = requests.request("POST", url, data=payload, headers=headers)
        #print(response.text)
        j = response.json()
        id = j["id"]
        while True:
            r = requests.request("GET", url+'/'+str(id), headers=headers)
            #print r.text
            j = r.json()
            if j["finished"]:
                print 'move finished!!'
                return True
                break
            rospy.sleep(1)
    except:
        print 'move failed!!'
        return False

def get_mir_goal():
    url = 'http://192.168.12.20:8080/v2.0.0/positions'
    headers = {
        'authorization': "Basic RGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA==",
        'content-type': "application/json",
        'cache-control': "no-cache"
        }
    try:
        r = requests.request("GET", url, headers=headers)
        print r.text
        return r.json()
    except:
        print 'get failed!!'
        
if __name__ == '__main__':
    goal_list = get_mir_goal()
    # print('mir goal list: ', goal_list)
