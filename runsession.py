import os

from bw4t.bw4tlogger import BW4TLogger
from bw4t.BW4TWorld import BW4TWorld
from bw4t.statistics import Statistics

from setuptools.sandbox import save_path   # type: ignore
from agents1.human import Human
from agents1.newagent import NewAgent
from agents1.tiger import Tiger
from agents1.fire import Fire

"""
This runs a single session. You have to log in on localhost:3000 and 
press the start button in god mode to start the session.
"""

if __name__ == "__main__":
    agents = [
        {'name':'agent1', 'botclass': NewAgent, 'settings':{'slowdown':10, 'colorblind':False}},
        #{'name':'agent2', 'botclass': RandomAgent, 'settings': {'slowdown': 5, 'colorblind': False}},
        {'name':'tiger1', 'botclass': Tiger, 'settings': {'slowdown': 20, 'colorblind': False}},
        {'name':'tiger2', 'botclass': Tiger, 'settings': {'slowdown': 20, 'colorblind': False}},
        {'name':'tiger3', 'botclass': Tiger, 'settings': {'slowdown': 20, 'colorblind': False}},
        {'name': 'fire1', 'botclass': Fire, 'settings': {'slowdown': 20, 'colorblind': False}},
        {'name': 'fire2', 'botclass': Fire, 'settings': {'slowdown': 20, 'colorblind': False}},
        {'name':'human1', 'botclass':Human, 'settings': {'slowdown': 1}},
        ]

    print("Started world...")
    world=BW4TWorld(agents).run()
    print("DONE!")
    print(Statistics(world.getLogger().getFileName()))
    
