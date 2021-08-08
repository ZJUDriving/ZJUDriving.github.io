import argparse
import carla
from env import Env
from agents.test_agent import testAgent
from agents.lane_follow import lane_follow_agent

import warnings
warnings.filterwarnings('ignore')

def parse_args():
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--pretime',
        metavar='PRETIME',
        default=10,
        help='look ahead time')
    argparser.add_argument(
        '--aheaddist',
        metavar='AHEAD',
        default=20,
        help='min look ahead distance')
    argparser.add_argument(
        '--time-step',
        metavar='TP',
        default=1/60.0,
        help='time_step')
    argparser.add_argument(
        '--d-step',
        metavar='DP',
        default=2.0,
        help='distance sample')
    args = argparser.parse_args()
    return args
        

if __name__ == "__main__":
    args = parse_args()
    client = carla.Client(args.host, args.port)
    client.set_timeout(4.0)

    # agent = testAgent()
    world = Env.Basic_Env(client.get_world(), args)
    state = world.reset()
    agent = lane_follow_agent(world.player)
    while True:
        # client.get_world().tick()
        world.render()
        action = agent.take_action(state)
        state = world.step(action)
        
    pygame.quit()