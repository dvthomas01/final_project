# phases/acquire_bin.py
from common import Phase, Command, Event
from link import SerialLink
from . import apriltag_pose as ap
import time


class AcquireCBin:
    """
    First phase in the FSM.
    Behaviour: send one ROTATE_CW command, wait for the ESP to reply
    'ROTATE_DONE', then hand control back to the Controller so it can
    switch to Phase.FINISH (or whatever next phase you prefer).
    """
 
    def __init__(self, link: SerialLink):
        self._link = link
        self._sent = False          # True after we’ve issued the rotate
        self._step = 0
        self._driveduration = 2
        self._depositduration = 1
        self._t0 = None

    # ---------------------------------------------------------------
    # Controller calls enter() exactly once when this phase becomes
    # active.  Good place to reset internal variables.
    # ---------------------------------------------------------------
    def enter(self) -> None:
        self._sent = False
        self._step = 0

    # ---------------------------------------------------------------
    # Controller calls tick(event) every 50 ms.
    # If the rotate is complete, return Phase.FINISH (causes shutdown).
    # Otherwise return None so the controller keeps us active.
    # ---------------------------------------------------------------
    def tick(self, event: Event | None) -> Phase | None:
        print(ap.readApriltag(9,True))
        # align using left camerea
        if self._step == 0:
            if abs (ap.readApriltag(9,True)[0])< 0.05 : 
                self._link.enqueue(Command.STOP.value) 
                self._step = 1
                self._sent = True
            if (ap.readApriltag(9,True)[0])>0: 
                self._link.enqueue(Command.ALIGN_F.value)
                self._sent = True
            else: 
                self._link.enqueue(Command.ALIGN_B.value)
                self._sent = True
            print("acquire_Cbin.py STEP 1")
            return None
        if self._step == 1: 
            self._link.enqueue(Command.STOP.value)  
            self._step = 2
            self._sent = True
            print("acquire_Cbin.py STEP 2")

            return None
        
        
        if self._step == 2 and event and event.name == "STOP":
            self._link.enqueue(Command.ROTATE_CCW.value) 
            self._sent = True
            self._step = 3
            print("acquire_Cbin.py STEP 3")
            return None
        
        if self._step == 3 and event and event.name == "ROTATE_DONE":
            self._link.enqueue(Command.STOP.value) 
            self._sent = True
            self._step = 4
            print("acquire_Cbin.py STEP 4")
            return None
        
        if self._step == 4 and event:
            self._link.enqueue(Command.APPROACH_PICKUP.value) 
            if ap.readApriltag(7,False)[1] <0.5:
            #if time.monotonic() - self._t0 >= self._driveduration:
                self._link.enqueue(Command.STOP.value) 
                self._step = 5
                self._sent = True
                return None
            self._sent = True
            print("setup.py STEP 5")
            return None   
        
        if self._step == 5: 
            self._link.enqueue(Command.STOP.value)  
            self._step = 6
            self._sent = True
            print("setup.py STEP 6")
            return None
        
        if self._step == 6 and event and event.name == "STOP":
            self._link.enqueue(Command.GRAB_BIN.value)  
            self._step = 7
            self._sent = True
            print("setup.py STEP 7")

        
        if self._step == 7 and event and event.name =="GRAB_BIN": #TODO: is this correct. update the success of grab 
            if abs (ap.readApriltag(7,True)[0])< 0.05 : 
                self._link.enqueue(Command.STOP.value) 
                self._step = 8
                self._sent = True
            if (ap.readApriltag(7,True)[0])>0: 
                self._link.enqueue(Command.ALIGN_F.value)
                self._sent = True
            else: 
                self._link.enqueue(Command.ALIGN_B.value)
                self._sent = True
            print("acquire_Cbin.py STEP 8")
            return None
        
        if self._step == 8 and event and event.name == "STOP":
            self._link.enqueue(Command.ROTATE_CCW.value) 
            self._sent = True
            self._step = 9
            print("acquire_Cbin.py STEP 9")
            return None

        if self._step ==  9 and event and event.name == "ROTATE_DONE":
            self._link.enqueue(Command.APPROACH_PICKUP.value) 
            if ap.readApriltag(7,False)[1] <0.5:
            #if time.monotonic() - self._t0 >= self._driveduration:
                self._link.enqueue(Command.STOP.value) 
                self._step = 10
                self._sent = True
                return None
            self._sent = True
            print("setup.py STEP 10")
            return None   
        
        
        if self._step ==  10 and event: 
            self._link.enqueue(Command.DEPOSIT_BIN.value) 
            if time.monotonic() - self._t0 >= self._depositduration:
                self._link.enqueue(Command.STOP.value) 
                self._step = 11
                self._sent = True
                return None
            self._sent = True
            print("setup.py STEP 11")
            return None   

        
        if self._step == 11 and event and event.name == "STOP":
            return Phase.FINISH        

