from __future__ import print_function
#import hddl_pyhop as pyhop
from . import pyhop
import rclpy
from my_sim_plugins.msg import BlockPoseArray

## NOTE: Pyhop from:
#   https://bitbucket.org/dananau/pyhop/src/master/

## Blocksworld domain from blocks_world_examples.py + blocks_world_methods.py + blocks_world_operators.py

# Initializing the PyHOP domain

#-- helping functions to implement methods e operators. 

def is_done(b1,state,goal):
    if b1 == 'table': return True
    if b1 in goal.pos and goal.pos[b1] != state.pos[b1]:
        return False
    if state.pos[b1] == 'table': return True
    return is_done(state.pos[b1],state,goal)

def status(b1,state,goal):
    if is_done(b1,state,goal):
        return 'done'
    elif not state.clear[b1]:
        return 'inaccessible'
    elif not (b1 in goal.pos) or goal.pos[b1] == 'table':
        return 'move-to-table'
    elif is_done(goal.pos[b1],state,goal) and state.clear[goal.pos[b1]]:
        return 'move-to-block'
    else:
        return 'waiting'

def all_blocks(state):
    return state.clear.keys()

#--


#-- Primitive Actions
#   NOTE: these are in them if PRECONDITION then EFFECTS

def pickup(state,b):
    if state.pos[b] == 'table' and state.clear[b] == True and state.holding == False: #PRECONDITION: Empty Hand, Block on Table, Block Clear
        state.pos[b] = 'hand'
        state.clear[b] = False
        state.holding = b
        return state
    else: return False

def unstack(state,b,c):
    if state.pos[b] == c and c != 'table' and state.clear[b] == True and state.holding == False: #PRECONDITION: Empty Hand, Block NOT on Table, Block Clear
        state.pos[b] = 'hand'
        state.clear[b] = False
        state.holding = b
        state.clear[c] = True
        return state
    else: return False
    
def putdown(state,b):
    if state.pos[b] == 'hand': #PRECONDITION: Block in HAND 
        state.pos[b] = 'table'
        state.clear[b] = True
        state.holding = False
        return state
    else: return False

def stack(state,b,c):
    if state.pos[b] == 'hand' and state.clear[c] == True: #PRECONDITION: Block in HAND and Target Block Clear
        state.pos[b] = c
        state.clear[b] = True
        state.holding = False
        state.clear[c] = False
        return state
    else: return False

# --


#-- High-Level Methods
#   NOTE: these are in the form if PRECONDITION then SUBTASKS

def moveb_m(state,goal):
    """
    This method implements the following block-stacking algorithm:
    If there's a block that can be moved to its final position, then
    do so and call move_blocks recursively. Otherwise, if there's a
    block that needs to be moved and can be moved to the table, then 
    do so and call move_blocks recursively. Otherwise, no blocks need
    to be moved.
    """
    for b1 in all_blocks(state):
        s = status(b1,state,goal)
        if s == 'move-to-table':
            return [('move_one',b1,'table'),('move_blocks',goal)]
        elif s == 'move-to-block':
            return [('move_one',b1,goal.pos[b1]), ('move_blocks',goal)]
        else:
            continue
    
    # If we get here, no blocks can be moved to their final locations.
    # If there's one needs to be moved and can be moved to the table, do so.
    b1 = pyhop.find_if(
        lambda x: status(x,state,goal) == 'waiting' \
                and not state.pos[x] == 'table',        # Éric Jacopin's bug fix
        all_blocks(state))
    if b1 != None:
        return [('move_one',b1,'table'), ('move_blocks',goal)]
    #
    # if we get here, there are no blocks that need moving
    return []

### methods for "move_one"

def move1(state,b1,dest):
    """
    Generate subtasks to get b1 and put it at dest.
    """
    return [('get', b1), ('put', b1,dest)]


### methods for "get"

def get_m(state,b1):
    """
    Generate either a pickup or an unstack subtask for b1.
    """
    if state.clear[b1]:
        if state.pos[b1] == 'table':
                return [('pickup',b1)]
        else:
                return [('unstack',b1,state.pos[b1])]
    else:
        return False


### methods for "put"

def put_m(state,b1,b2):
    """
    Generate either a putdown or a stack subtask for b1.
    b2 is b1's destination: either the table or another block.
    """
    if state.holding == b1:
        if b2 == 'table':
                return [('putdown',b1)]
        else:
                return [('stack',b1,b2)]
    else:
        return False


class DynamicBlocksworld:
    def __init__(self, node, topic='/object_pose'):
        """
        node: istanza di rclpy.node.Node
        topic: topic ROS2 bridgato da ros_ign_bridge
        """
        self.node = node
        self.received = False
        self.state = None
        self.goal  = None


        # Subscription to bridged topic by plugin
        self.sub = node.create_subscription(
            BlockPoseArray, 
            topic, 
            self._callback,
            100
        )


        # 1) Pose Extraction of models "block_..." 
        # The goal is to build a symbolic dictionary for the planner based on the pose retrieved from gazebo topic
        #pos = {} #Block Name -> Key,  Content-> Block Underneath
        #Block Size is 0.07x0.07x0.07m
        #If z is 'low', then its placed on the table. Otherwise it is place on another block
        #An offset of 0.018 (half of half block size) is used to determine threshold


    def _callback(self, msg: BlockPoseArray):
        if not self.received: #Callback is executed once, at the end of the callback this field is set to true

            pos = {}  

            for block in msg.blocks:
                name = block.name
                pose = block.pose

                if not name.startswith("block_"):
                    continue

                if pose.position.z < 0.053:
                    pos[name] = 'table'
                else:
                    under = 'table'
                    for other in msg.blocks:
                        if other.name == name or not other.name.startswith('block_'):
                            continue
                        dz = pose.position.z - other.pose.position.z
                        dx = abs(pose.position.x - other.pose.position.x)
                        dy = abs(pose.position.y - other.pose.position.y)
                        if (dz < 0.088 and dz > 0) and dx < 0.018 and dy < 0.018:
                            under = other.name
                            break
                    pos[name] = under


            
            # 2) Computes clear state dictionary: 
            #clear = {} #Block Name -> Key,  Content -> Boolean (True if Block is Clear, False otherwise)
            clear = {b: all(pos.get(o) != b for o in pos) for b in pos}

            # 3) Build corresponding symbolig Pyhop state
            state1 = pyhop.State('state1')
            state1.pos     = pos
            state1.clear   = clear
            state1.holding = False
            pyhop.print_state(state1)

            # 4) Build desired symbolig Pyhop goal
            goal = pyhop.Goal('goal')
            goal.pos={'block_a':'block_d','block_b':'block_c', 'block_c':'table', 'block_d':'table', 'block_e':'block_a', 'block_f':'block_b' }
            goal.clear={'block_a':False, 'block_c':False,'block_b':False, 'block_d':False, 'block_e':True, 'block_f':True}

            goal.holding=False
            
            self.state, self.goal = state1, goal
             # Flag to set in order to execute this callback only once
            self.received = True 

    def get_state_and_goal(self, timeout_sec=1.0):
        start = self.node.get_clock().now().nanoseconds / 1e9
        while not self.received:
            now = self.node.get_clock().now().nanoseconds / 1e9
            if now - start >= timeout_sec:
                break
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.state, self.goal



def run_pyhop_plan(state, goal):
    # register methods/operators (do only once)
    pyhop.declare_methods('move_blocks', moveb_m)
    pyhop.declare_methods('move_one', move1)
    pyhop.declare_methods('get', get_m)
    pyhop.declare_methods('put', put_m)
    pyhop.declare_operators(pickup, unstack, putdown, stack)
    # call planner
    result = pyhop.pyhop(state, [('move_blocks', goal)], verbose=0)
    return result  #could be False or (plan, state_trace)
