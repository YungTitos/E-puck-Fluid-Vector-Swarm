import random
import optuna
import json
from controller import Supervisor

# SETUP AND CONSTANTS 
supervisor = Supervisor()
root_node = supervisor.getRoot()
children_field = root_node.getField('children')
timestep = int(supervisor.getBasicTimeStep())

EPUCK_DIA = 0.074
HOLE_WIDTH = EPUCK_DIA * 1.5  
ARENA_Y = 2.0  
WALL_X = 0.0

WEIGHTS = {
    "W_LIGHT": 0.0005,
    "W_PROX": 0.00005,
    "W_SLIDE": 0.01,
    "W_NOISE": 1.5,
}

spawned_nodes = [] 

# SCENARIOS 
all_scenarios = []
for r in range(6, 11):      # robots
    for h in range(2, 5):   # holes
        # Difficulty -> Robots per Hole 
        difficulty = r / h 
        all_scenarios.append({"robots": r, "holes": h, "difficulty": difficulty})

# Hardest scenarios first 
all_scenarios.sort(key=lambda x: x["difficulty"], reverse=True)


# SIMULATION FUNCTIONS

def clear_world():
    """Removes all dynamically spawned nodes from the previous run."""
    global spawned_nodes
    for node in spawned_nodes:
        try:
            node.remove()
        except:
            pass
    spawned_nodes.clear()

def run_scenario(num_robots, num_holes, custom_data_str):
    """Spawns the world, runs the simulation for one scenario, and returns the stats."""
    global spawned_nodes
    clear_world()

    # Spawn robots
    for i in range(num_robots):
        x = random.uniform(-1.4, -0.3)
        y = random.uniform(-0.9, 0.9)
        rot = random.uniform(0, 6.28)
        
        epuck_str = f'''
        DEF EPUCK_{i} E-puck {{
          translation {x} {y} 0
          rotation 0 0 1 {rot}
          name "epuck_{i}"
          controller "robot_particle"
          customData "{custom_data_str}" 
        }}
        '''
        children_field.importMFNodeFromString(-1, epuck_str)
        spawned_nodes.append(supervisor.getFromDef(f"EPUCK_{i}"))

    # Spawn walls
    total_solid_space = ARENA_Y - (num_holes * HOLE_WIDTH)
    segment_length = total_solid_space / (num_holes + 1)
    current_y = -1.0 

    for i in range(num_holes + 1):
        center_y = current_y + (segment_length / 2)
        wall_str = f'''
        DEF WALL_{i} Solid {{
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
        spawned_nodes.append(supervisor.getFromDef(f"WALL_{i}"))
        current_y += segment_length + HOLE_WIDTH

    # Run sub-simulation
    MAX_SECONDS = 120  
    max_ticks = int((MAX_SECONDS * 1000) / timestep)
    ticks = 0
    finished_count = 0
    
    while supervisor.step(timestep) != -1 and ticks < max_ticks:
        ticks += 1
        finished_count = 0
        
        for i in range(num_robots):
            node = spawned_nodes[i] 
            if node is not None:
                pos = node.getPosition()
                if pos[0] > 1.0: 
                    finished_count += 1
                    
        if finished_count == num_robots:
            break 

    time_taken = ticks * (timestep / 1000.0)
    failed_robots = num_robots - finished_count
    
    return time_taken, failed_robots

# SINGLE SIMULATION

def run_single_simulation(params, num_robots=None, num_holes=None):
    """Runs exactly one scenario. If robots or holes aren't specified, picks randomly."""
    if num_robots is None:
        num_robots = random.randint(6, 10)
    if num_holes is None:
        num_holes = random.randint(2, 4)
        
    custom_data_str = f"{params[0]},{params[1]},{params[2]},{params[3]}"
    
    print(f"\nStarting Single Simulation ({num_robots}R/{num_holes}H)", flush=True)
    
    time_taken, failed_robots = run_scenario(num_robots, num_holes, custom_data_str)
    
    print(f"Simulation Finished | Time: {time_taken:.1f}s | Failed: {failed_robots}", flush=True)


def single_simulation():
    """Runs a single test with explicitly defined parameters."""
    
    test_params = list(WEIGHTS.values())
    
    # Completely random number of robots and holes
    run_single_simulation(test_params)
    
    # Option B: Specific number of robots and holes (e.g., 8 robots, 3 holes)
    # run_single_simulation(test_params, num_robots=8, num_holes=3)

# EVALUATION AND OPTIMIZATION

def evaluate_parameters(params, trial=None):
    """
    Evaluates a specific set of parameters across all scenarios
    """
    custom_data_str = f"{params['w_light']},{params['w_prox']},{params['w_slide']},{params['w_noise']}"
    total_score = 0
    
    run_name = f"Trial {trial.number}" if trial else "Manual Full Gauntlet"
    print(f"\nStarting {run_name}", flush=True)

    for step, scenario in enumerate(all_scenarios):
        num_robots = scenario["robots"]
        num_holes = scenario["holes"]
        
        # Execute the scenario
        time_taken, failed_robots = run_scenario(num_robots, num_holes, custom_data_str)

        # Scoring
        sub_score = time_taken + (failed_robots * 50) 
        total_score += sub_score
        
        print(
            f"  Step {step+1}/{len(all_scenarios)} ({num_robots}R/{num_holes}H) | Time: {time_taken:.1f}s | Failed: {failed_robots}",
            flush=True,
        )

        # Kill switch: Prune the run if a robot fails
        if failed_robots > 0:
            remaining_scenarios = len(all_scenarios) - (step + 1)
            penalty = remaining_scenarios * 300  
            total_score += penalty
            
            print(f"  [EARLY STOP] Parameters failed the Gauntlet. Penalty applied.", flush=True)
            
            # If we are in an Optuna trial, raise the prune exception
            if trial is not None:
                raise optuna.TrialPruned()
            else:
                break

    print(f"{run_name} Finished -> Total Score: {total_score:.2f}", flush=True)
    return total_score


def optuna_objective(trial):
    """Optuna objective function wrapper."""
    params = {
        "w_light": trial.suggest_float("w_light", 1e-5, 1e-3, log=True),
        "w_prox": trial.suggest_float("w_prox", 1e-6, 1e-4, log=True),
        "w_slide": trial.suggest_float("w_slide", 1e-3, 5e-2, log=True),
        "w_noise": trial.suggest_float("w_noise", 0.1, 3.0)
    }
    return evaluate_parameters(params, trial=trial)


def optimization_pipeline(n_trials=200):
    """Initializes and runs the Optuna study, then saves results."""
    print("STARTING FAIL-FAST OPTUNA PIPELINE")
    study = optuna.create_study(direction="minimize", pruner=optuna.pruners.MedianPruner())
    study.optimize(optuna_objective, n_trials=n_trials)

    print("\nOPTIMIZATION COMPLETE!")
    print("Best Generalized Score: ", study.best_value)
    print("Best Universal Parameters:")
    for key, value in study.best_params.items():
        print(f"  {key}: {value}")

    print("\nExporting results to JSON...")
    results_export = {
        "best_score": study.best_value,
        "best_params": study.best_params,
        "all_trials": []
    }

    for t in study.trials:
        trial_data = {
            "trial_number": t.number,
            "state": str(t.state),
            "final_score": t.value if t.value is not None else -1, 
            "params": t.params,
            "history": t.user_attrs.get("step_history", [])
        }
        results_export["all_trials"].append(trial_data)

    json_filename = "optuna_swarm_results.json"
    with open(json_filename, "w") as f:
        json.dump(results_export, f, indent=4)

    print(f"Saved to '{json_filename}'")

if __name__ == "__main__":
    
    # ==========================================
    # TOGGLE THIS VARIABLE TO CHANGE MODES
    # "Single Simulation" : 1
    # "Optimization"  : 2
    EXECUTION_MODE = 1 
    
    if EXECUTION_MODE == 1:
        single_simulation()
    elif EXECUTION_MODE == 2:
        optimization_pipeline(n_trials=200)
    else:
        print("Invalid EXECUTION_MODE selected.")

    supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)