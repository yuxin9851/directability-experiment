import ast
import sys
from typing import final, List, Dict, Final
import numpy as np  # type: ignore
import random  # type: ignore
from matrx.agents.agent_types.patrolling_agent import PatrollingAgentBrain  # type: ignore
from matrx.actions import MoveNorth, OpenDoorAction, CloseDoorAction  # type: ignore
from matrx.actions.move_actions import MoveEast, MoveSouth, MoveWest  # type: ignore
from matrx.agents.agent_utils.state import State  # type: ignore
from matrx.api import api

from bw4t.BW4TBrain import BW4TBrain


class Fire(BW4TBrain):
    '''
    This agent makes random walks and opens any doors it hits upon
    '''

    def __init__(self, settings: Dict[str, object]):
        super().__init__(settings)
        self._moves = [MoveNorth.__name__, MoveEast.__name__, MoveSouth.__name__, MoveWest.__name__]

    def initialize(self):
        super().initialize()
        self._door_range = 1


    def filter_bw4t_observations(self, state):
        self._processMessages()
        return state  # Why need to returning state

    def _processMessages(self):
        '''
        process incoming messages.
        Reported blocks are added to self._blocks
        '''
        if len(self.received_messages) > 0:
            print(self.agent_name + " recvd msgs:", self.received_messages)
        for msg in self.received_messages:
            if msg.startswith("Found:"):
                try:
                    content = msg[6:]
                    infos = ast.literal_eval(content)
                    for blockinfo in infos:
                        self._blockpositions = self._blockpositions.updateInfo(blockinfo)
                except:
                    print("Warning. parsing err " + str(sys.exc_info()) + ": " + content)
        # workaround for bug
        self.received_messages = []

    def decide_on_bw4t_action(self, state: State):
        self._checknearbyAgents(state)

        # open any nearby closed doors
        # for doorId in self._nearbyDoors(state):
        #     if not state[doorId]['is_open']:
        #         print("opening nearby door")
        #         return (OpenDoorAction.__name__, { 'object_id':doorId })

        # TODO pick up or put down object
        #         if False:

        # state.get()
        # if self.onBlock(state)!=None and self.holdingBlock()==None:
        #    print("picking up a block")
        #    self.getBlock()

        # pick random walk direction
        act: tuple = (random.choice(self._moves), {})
        # print(act)
        return act

    def _checknearbyAgents(self, state: State):
        # state.get_agents_with_property()
        agents = state.get_agents()
        if len(agents) > 1:
            print("fire burns!")
            api._matrx_done = True
                # sys.exit()
            # sys.exit()

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

