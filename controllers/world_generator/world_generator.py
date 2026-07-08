import random
from controller import Supervisor

# ==========================================
# 1. SETUP AND CONSTANTS
# ==========================================
supervisor = Supervisor()
root_node = supervisor.getRoot()
children_field = root_node.getField('children')

# Challenge Requirements
NUM_ROBOTS = random.randint(6, 10)
NUM_HOLES = random.randint(2, 4)

# Dimensions (in meters)
EPUCK_DIA = 0.074
HOLE_WIDTH = EPUCK_DIA * 1.55  
ARENA_Y = 2.0  
WALL_X = 0.0   

print("========================================")
print(f"GENERATING WORLD: {NUM_ROBOTS} Robots | {NUM_HOLES} Holes")
print("========================================")

# ==========================================
# 2. SPAWN THE E-PUCKS
# ==========================================
for i in range(NUM_ROBOTS):
    x = random.uniform(-1.4, -0.3)
    y = random.uniform(-0.9, 0.9)
    rot = random.uniform(0, 6.28)
    
    # NEW: We added "DEF EPUCK_{i}" so the supervisor can find it later!
    epuck_str = f'''
    DEF EPUCK_{i} E-puck {{
      translation {x} {y} 0
      rotation 0 0 1 {rot}
      name "epuck_{i}"
      controller "fluid_swarm"
    }}
    '''
    children_field.importMFNodeFromString(-1, epuck_str)

# ==========================================
# 3. CALCULATE AND SPAWN THE WALL
# ==========================================
total_hole_space = NUM_HOLES * HOLE_WIDTH
total_solid_space = ARENA_Y - total_hole_space
segment_length = total_solid_space / (NUM_HOLES + 1)

current_y = -1.0 

for i in range(NUM_HOLES + 1):
    center_y = current_y + (segment_length / 2)
    
    wall_str = f'''
    Solid {{
      translation {WALL_X} {center_y} 0.05
      children [
        Shape {{
          appearance PBRAppearance {{ baseColor 0.2 0.2 0.8 roughness 1 metalness 0 }}
          geometry Box {{ size 0.1 {segment_length} 0.2 }}
        }}
      ]
      boundingObject Box {{ size 0.1 {segment_length} 0.2 }}
    }}
    '''
    children_field.importMFNodeFromString(-1, wall_str)
    current_y += segment_length + HOLE_WIDTH

# ==========================================
# 4. SUPERVISOR LOOP (The Score Tracker)
# ==========================================
timestep = int(supervisor.getBasicTimeStep())

# Grab references to all the robots we just spawned
robot_nodes = []
for i in range(NUM_ROBOTS):
    node = supervisor.getFromDef(f"EPUCK_{i}")
    robot_nodes.append(node)

print("Timer started. May the swarm flow!")

# Watch them every tick
while supervisor.step(timestep) != -1:
    finished_count = 0
    
    for node in robot_nodes:
        # SAFETY CHECK: Only check position if the robot successfully spawned
        if node is not None: 
            pos = node.getPosition()
            # If the robot's X coordinate is past the wall (e.g., > 1.0 meters near the light)
            if pos[0] > 1.0: 
                finished_count += 1
            
    # CHECK WIN CONDITION
    if finished_count == NUM_ROBOTS:
        final_time = supervisor.getTime()
        print("\n========================================")
        print("MISSION ACCOMPLISHED!")
        print(f"All {NUM_ROBOTS} robots crossed the wall.")
        print(f"Total Time: {final_time:.2f} seconds")
        print("========================================\n")
        
        # Automatically pause the simulation so the time doesn't keep running!
        supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
        break