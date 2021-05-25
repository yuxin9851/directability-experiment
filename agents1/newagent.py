import ast
import enum
import re
import sys
from typing import final, List, Dict, Final
import numpy as np  # type: ignore
import random  # type: ignore

from matrx.agents import StateTracker, Navigator
from matrx.agents.agent_types.patrolling_agent import PatrollingAgentBrain  # type: ignore
from matrx.actions import MoveNorth, OpenDoorAction, CloseDoorAction, GrabObject, DropObject  # type: ignore
from matrx.actions.move_actions import MoveEast, MoveSouth, MoveWest  # type: ignore
from matrx.agents.agent_utils.state import State  # type: ignore
from matrx.messages import Message
from matrx.api import api

from agents1.BlockPositions import BlockPositions, sameAppearance
from bw4t.BW4TBrain import BW4TBrain

class Phase(enum.Enum):
    PLAN_PATH_ALONG_DROPZONE=0, # to update our knowledge about placed blocks
    FOLLOW_PATH_ALONG_DROPZONE=1,
    FIND_NEXT_GOAL=2,# determine next goal zone
    PICK_SOME_CLOSED_DOOR=3, # we only explore closed rooms
    #we leave explored room doors open so we know we explored them
    #Pradeep decided we MUST enter room and search the inside
    PLAN_PATH_TO_CLOSED_DOOR=4,
    FOLLOW_PATH_TO_CLOSED_DOOR=5,
    OPEN_DOOR=6,
    PLAN_ROOM_SEARCH_PATH=7,
    FOLLOW_ROOM_SEARCH_PATH=8,
    PLAN_PATH_TO_BLOCK=9,
    FOLLOW_PATH_TO_BLOCK=10,
    TAKE_BLOCK=11,
    PLAN_PATH_TO_DROPPOINT=12,
    FOLLOW_PATH_TO_DROPPOINT=13,
    DROP_BLOCK=14

class NewAgent(BW4TBrain):
    '''
    This agent makes random walks and opens any doors it hits upon
    '''

    def __init__(self, settings: Dict[str, object]):
        super().__init__(settings)
        self._phase = Phase.PLAN_PATH_ALONG_DROPZONE
        self._blockpositions = BlockPositions()


    def initialize(self):
        super().initialize()
        self._state_tracker = StateTracker(agent_id = self.agent_id)
        self._navigator = Navigator(agent_id = self.agent_id,
            action_set = self.action_set, algorithm = Navigator.A_STAR_ALGORITHM)

    #called every tick
    def filter_bw4t_observations(self, state):
        '''
            process incoming messages.
            Reported blocks are added to self._blocks
        '''
        self._processMessages()
        return state  # Why need to returning state

    def _processMessages(self):
        if len(self.received_messages) > 0:
            print(self.agent_name + "recvd msgs:", self.received_messages )

        for msg in self.received_messages:
            if msg.startswith("Command:"):
                print("Yes, sir！")

            if msg.startswith("Warning:"):
                print("I should not go this way!")

            if msg.startswith("Direcion:"):
                print("I should go with the direction")

            if msg.startswith("Found:"):
                try:
                    content = msg[6:]
                    infos = ast.literal_eval(content)
                    for blockinfo in infos:
                       self._blockpositions = self._blockpositions.updateInfo(blockinfo)
                except:
                    print("Warning, paring err " + str(sys.exc_info()) + ": " + content)
                # finally:
                #     self.received_messages = []

    #not called every tick
    def decide_on_bw4t_action(self, state: State):

        oldblocks = self._blockpositions
        self._blockpositions = self._blockpositions.update(state)

        changes = self._blockpositions.getDifference(oldblocks)

        if len(changes) > 0:
            msg = Message(content = 'Found:' + str(changes), from_id = 'me' )
            #self.send_message(msg)

        while True:
            if self._phase == Phase.PLAN_PATH_ALONG_DROPZONE:
                self._navigator.reset_full()
                waypoints = map(lambda info:info['location'], self._getDropZones(state))
                self._navigator.add_waypoints(waypoints)
                self._phase = Phase.FOLLOW_PATH_ALONG_DROPZONE

            if Phase.FOLLOW_PATH_ALONG_DROPZONE == self._phase:
                # This explores the area so we know the needed blocks afterwards
                self._state_tracker.update(state)
                # execute the  path steps as planned in self._navigator
                action = self._navigator.get_move_action(self._state_tracker)
                if action != None:
                    return action, {}
                # If we get here, we're there
                self._phase = Phase.FIND_NEXT_GOAL

            if Phase.FIND_NEXT_GOAL == self._phase:
                self._goalZone = None

                done = True
                for info in self._getDropZones(state):
                    goodblocks = [blockinfo
                                  for blockinfo in self._blockpositions.getBlocksAt(info['location'])
                                  if sameAppearance(blockinfo['visualization'], info['visualization'])]
                    if len(goodblocks) == 0:
                        done = False
                        self._goalZone = info
                        break

                if done == True:
                    api._matrx_done = True

                if self._goalZone == None:
                    self._phase = Phase.PLAN_PATH_ALONG_DROPZONE
                else:
                    # whether see certain blocks already
                    options = self._blockpositions.getAppearance(self._goalZone['visualization'])
                    droplocs = [info['location'] for info in self._getDropZones(state)]
                    options = [info for info in options if not info['location'] in droplocs]

                    if len(options) == 0:
                        self._phase = Phase.PICK_SOME_CLOSED_DOOR
                    else:
                        self._block = random.choice(options)
                        self._phase = Phase.PLAN_PATH_TO_BLOCK


            if Phase.PICK_SOME_CLOSED_DOOR == self._phase:
                closedDoors = [door for door in state.values()
                               if 'class_inheritance' in door
                               and 'Door' in door['class_inheritance']
                               and not door['is_open']]

                if len(self.received_messages) != 0:

                    one_time = False
                    choosen_doors = []

                    for i in reversed(range(len(self.received_messages))):
                        if(self.received_messages[i].startswith("c")):
                            choosen_door = int(self.received_messages[i][1:])
                            for j in range(len(closedDoors)):
                                if int(closedDoors[j]['room_name'][5:]) == choosen_door:
                                    self._door = closedDoors[j]
                                    break
                            self._phase = Phase.PLAN_PATH_TO_CLOSED_DOOR
                            one_time = True
                            self.received_messages.pop(i)
                            break

                        elif(self.received_messages[i].startswith("w")):
                            choosen_doors.append(int(self.received_messages[i][1:]))

                        elif (self.received_messages[i].startswith("s")):
                            choosen_door = int(self.received_messages[i][1:])
                            for j in range(len(closedDoors)):
                                if int(closedDoors[j]['room_name'][5:]) == choosen_door:
                                    random_number = random.randint(1, 10)
                                    if  random_number <= 3:
                                        self._door = random.choice(closedDoors)
                                    else:
                                        self._door = closedDoors[j]
                                    break
                            self._phase = Phase.PLAN_PATH_TO_CLOSED_DOOR
                            one_time = True
                            self.received_messages.pop(i)
                            break

                    if one_time == True:
                        continue

                    targetDoors = []
                    nontargetDoors = []

                    for i in range(len(closedDoors)):
                        door = int(closedDoors[i]['room_name'][5:])
                        found = False
                        for j in range(len(choosen_doors)):
                            if door == choosen_doors[j]:
                                nontargetDoors.append(closedDoors[i])
                                found = True
                                break

                        if found == True:
                            continue

                        targetDoors.append(closedDoors[i])

                    rand_number = random.randint(1, 10)
                    if rand_number <= 9:
                        self._door = random.choice(targetDoors)
                    else:
                        self._door = random.choice(nontargetDoors)

                    self._phase = Phase.PLAN_PATH_TO_CLOSED_DOOR
                    #self.received_messages = self.received_messages[:-1]

                else:
                    if len(closedDoors) == 0:
                        # can't handle this situation.
                        self._phase = Phase.PLAN_PATH_ALONG_DROPZONE
                    else:
                        self._door = random.choice(closedDoors)
                        self._phase = Phase.PLAN_PATH_TO_CLOSED_DOOR

            if Phase.PLAN_PATH_TO_CLOSED_DOOR == self._phase:
                # self._door must be set to target door
                self._navigator.reset_full()
                doorLoc: tuple = self._door['location']
                # HACK we assume door is at south side of room

                doorLoc = doorLoc[0], doorLoc[1] + 1
                print("heading for door at ", doorLoc)
                self._navigator.add_waypoints([doorLoc])
                self._phase = Phase.FOLLOW_PATH_TO_CLOSED_DOOR

            if Phase.FOLLOW_PATH_TO_CLOSED_DOOR == self._phase:
                # self._door must be set
                self._state_tracker.update(state)
                # execute the  path steps as planned in self._navigator
                action = self._navigator.get_move_action(self._state_tracker)
                if action != None:
                    return action, {}
                # If we get here, we're there
                self._phase = Phase.OPEN_DOOR


            if Phase.OPEN_DOOR == self._phase:
                # self._door must be set
                print("opening door!")
                self._phase = Phase.PLAN_ROOM_SEARCH_PATH
                return OpenDoorAction.__name__, {'object_id': self._door['obj_id']}

            if Phase.PLAN_ROOM_SEARCH_PATH == self._phase:
                # self._door must be set
                roomTiles = [info['location'] for info in state.values()
                             if 'class_inheritance' in info
                             and 'AreaTile' in info['class_inheritance']
                             # notice, in matrx a room can go to only 1 door
                             # because of the 'room_name' property of doors
                             and 'room_name' in info
                             and info['room_name'] == self._door['room_name']
                             ]
                # FIXME we want to sort these tiles for efficient search...
                # CHECK rooms don't need to be square I assume?
                self._navigator.reset_full()
                self._navigator.add_waypoints(roomTiles)
                self._phase = Phase.FOLLOW_ROOM_SEARCH_PATH

            if Phase.FOLLOW_ROOM_SEARCH_PATH == self._phase:
                self._state_tracker.update(state)
                action = self._navigator.get_move_action(self._state_tracker)
                if action != None:
                    return action, {}
                # If we get here, we're done
                self._phase = Phase.FIND_NEXT_GOAL

            if Phase.PLAN_PATH_TO_BLOCK == self._phase:
                # self._block must be set to info of target block
                # self._goalZone must be set to goalzone needing that block
                # we assume door to room containing block is open
                self._navigator.reset_full()
                self._navigator.add_waypoints([self._block['location']])
                self._phase = Phase.FOLLOW_PATH_TO_BLOCK

            if Phase.FOLLOW_PATH_TO_BLOCK == self._phase:
                # self._block must be set to info of target block
                # self._goalZone must be set to goalzone needing that block
                self._state_tracker.update(state)
                action = self._navigator.get_move_action(self._state_tracker)
                if action != None:
                    return action, {}
                if self._navigator.is_done:
                    self._phase = Phase.TAKE_BLOCK
                else:
                    print("oops, door is closed?")
                    # door closed?? Explore that room now.
                    area = [area for area in state.values()
                            if 'class_inheritance' in area
                            and 'AreaTile' in area['class_inheritance']
                            and area['location'] == self._block['location']][0]
                    self._door = state.get_room_doors(area['room_name'])[0]
                    self._phase = Phase.PLAN_PATH_TO_CLOSED_DOOR

            if Phase.TAKE_BLOCK == self._phase:
                # self._block must be set to info of target block
                # self._goalZone must be set to goalzone needing that block
                print("delivering block")
                msg = Message(content='Delivering block ' + str(self._block['visualization']), from_id='me')
                self.send_message(msg)
                self._phase = Phase.PLAN_PATH_TO_DROPPOINT
                return GrabObject.__name__, {'object_id': self._block['obj_id']}

            if Phase.PLAN_PATH_TO_DROPPOINT == self._phase:
                # self._block must be set to info of target block
                # self._goalZone must be set to goalzone needing that block
                self._navigator.reset_full()
                self._navigator.add_waypoints([self._goalZone['location']])
                self._phase = Phase.FOLLOW_PATH_TO_DROPPOINT

            if Phase.FOLLOW_PATH_TO_DROPPOINT == self._phase:
                self._state_tracker.update(state)
                # execute the  path steps as planned in self._navigator
                action = self._navigator.get_move_action(self._state_tracker)
                if action != None:
                    return action, {}
                # If we get here, we're there
                self._phase = Phase.DROP_BLOCK

            if Phase.DROP_BLOCK == self._phase:
                print("Dropped box ", self._block)
                self._phase = Phase.FIND_NEXT_GOAL
                # don't use 'object_id':self._nextBlock[0],
                # there seems a bug in MATRX DropObject #7
                # Maybe it's fixed now
                return DropObject.__name__, {}
        # open any nearby closed doors
        #for doorId in self._nearbyDoors(state):
            # if not state[doorId]['is_open']:
            #     print("opening nearby door")
            #     return (OpenDoorAction.__name__, {'object_id': doorId})

        # TODO pick up or put down object
        #         if False: # self.onBlock(state)!=None and self.holdingBlock()==None:
        #             print("picking up a block")
        #             self.getBlock()

        # pick random walk direction
        # act: tuple = (random.choice(self._moves), {})
        # # print(act)
        # return act

    def _getDropZones(self, state: State):
        '''
        @return list of drop zones (their full dict), in order (the first one is the
        the place that requires the first drop)
        '''
        # we must use is_goal_block, not is_drop_zone, to collect also the
        # correct appearance
        places = state[{'is_goal_block': True}]
        # sort does in-place sorting
        places.sort(key=lambda info: info['location'][1], reverse=True)
        return places

    def _isCarrying(self, state: State, appearance: dict):
        """
        @param state the current State
        @param appearance a dict with the required block appearance
        @return true iff we are carrying a block of given appearance
        """
        for carrying in state[self.agent_id]['is_carrying']:
            if sameAppearance(carrying['visualization'], appearance):
                return True
        return False

    def _getDropOff(self, state: State, y: int) -> tuple:
        """
        @param y the y location of the required drop off location
        @return the drop off location (x,y) given the y.
        @throws out of index error if there is no drop zone at given y position.
        """
        for id in state.keys():
            if 'is_drop_zone' in state[id] and state[id]['location'][1] == y:
                return state[id]['location']
        raise ValueError("There is no block at y location " + str(y))

    def _findLocationOfBlock(self, state: State, appearance: dict):
        """
        @param state the current State
        @param appearance the 'visualization' settings. Must contain
        'size', 'shape' and  color.
        @return (id, x,y) of a block of requested appearance,
        that is not already on a dropoff point or being carried.
        """
        droplocations = [state[id]['location']
                         for id in state.keys()
                         if 'is_goal_block' in state[id] and state[id]['is_goal_block']]

        # find locations of all blocks of given appearance that are not already
        # on a droplocation
        locs = [(id,) + state[id]['location'] for id in state.keys()
                if 'is_collectable' in state[id]
                and state[id]['is_collectable']
                and state[id]['visualization']['size'] == appearance['size']
                and state[id]['visualization']['shape'] == appearance['shape']
                and state[id]['visualization']['colour'] == appearance['colour']
                and not state[id]['location'] in droplocations
                and len(state[id]['carried_by']) == 0
                ]
        if len(locs) == 0:
            return None
        return random.choice(locs)

    def _findRoomContaining(self, state: State, loc: tuple):
        """
        @param loc the (x,y) location
        @return a (should be unique) room name
        that contains that location, or None if no such room.
        Basically we look for an AreaTile at given loc.
        NOTICE: room name is a label used by the room designer,
        it's not the ID. I assume that properly designed
        worlds use this label consistently to tag other objects in the same room,
        typically the doors tiles and walls.
        """
        locs = [state[id]['room_name'] for id in state.keys()
                if 'class_inheritance' in state[id]
                and 'AreaTile' in state[id]['class_inheritance']
                and state[id]['location'] == loc
                ]
        if (len(locs) == 0):
            return None
        return locs[0]

    def _nearbyDoors(self, state: State):
        # copy from humanagent
        # Get all doors from the perceived objects
        objects = list(state.keys())
        doors = [obj for obj in objects if 'is_open' in state[obj]]
        doors_in_range = []
        for object_id in doors:
            # Select range as just enough to grab that object
            dist = int(np.ceil(np.linalg.norm(
                np.array(state[object_id]['location']) - np.array(
                    state[self.agent_id]['location']))))
            if dist <= self._door_range:
                doors_in_range.append(object_id)
        return doors_in_range

