#ifdef BASE_40
enum Events {
   SOLENOID_EJECTS_TRAY = 1,
   TEST_COMPLETE_TRAY_SWITCH_DETECTED = 2
};


void bleNotifyEvent(uint16 event)
{
   static uint16 lastEvent = 0;
   
   if (event != lastEvent) {
      lastEvent = event;
      int32 ts = time_get();
      char b[6];
      int i;
      char *p= &ts;
      for(i=0; i< 4; i++)
      {
         b[i] = p[i];
      }
      p=&event;
      b[i++] = p[0];
      b[i] = p[1];
      ble_cmd_attributes_write(BLE_HANDLE_EVENTS, 0, 6, b);
      while(!ble_log_process());
   }
}

#endif

