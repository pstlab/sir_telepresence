#!/usr/bin/env python3
import rospy
import sys
import requests
import json
import dateutil.parser
import time
from datetime import datetime
import pytz

class reminder:

  def __init__(self):
    self.active_request = True
    self.host = '0.0.0.0'
    self.port = '8000'
    self.run()
  

  def run(self):
      print('Inizio')
      rate = rospy.Rate(10)
      while not rospy.is_shutdown():
         if(self.active_request):
          self.active_request = False
          try:
            rospy.loginfo('Chiamata a duckling')
            #request_time = time.time() #unix time
            #tzinfos = {"CET": dateutil.tz.gettz("Europe/Rome")}
            #request_time =  dateutil.parser.parse(str(datetime.now()), tzinfos=tzinfos, dayfirst=True).timestamp() #unix
            request_time = datetime.now(tz=pytz.timezone('Europe/Rome'))
            print('Time richiesta: ' + str(request_time))
            r = requests.post('http://' + self.host + ':' + self.port + '/parse/',  data={"text":"Fra cinque minuti","locale":"it_IT", "tz":"Europe/Rome" })
            if(r.status_code == requests.codes.ok):
              rospy.loginfo('Chiamata con successo')
              print(json.loads(r.text))
              current_timestamp = json.loads(r.text)[0]['value']['values'][0]['value']
              tzinfos = {"CET": dateutil.tz.gettz("Europe/Rome")}
              response_human_time = dateutil.parser.parse(current_timestamp, tzinfos=tzinfos)
              #response_time = dateutil.parser.parse(current_timestamp, tzinfos=tzinfos, dayfirst=True).timestamp() #unix time
              print('Time reminder: ' + str(response_human_time))
              #print('Unix time request:' + str(request_time))
              #print('Unix time response:' + str(response_time))
              #difference_time = (response_time - request_time)/60/1000 #previous attempt in unix
              difference_time = (response_human_time - request_time).total_seconds()
              print(difference_time)
          except requests.exceptions.RequestException as e:
            rospy.loginfo('Errore')
            rospy.logerr('Duckling server call failed\n' +
                         ''.join(traceback.format_stack()))
            raise SystemExit(e)
         rate.sleep()

def main(args):
  rospy.init_node('reminder', anonymous=True)
  rm = reminder()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
