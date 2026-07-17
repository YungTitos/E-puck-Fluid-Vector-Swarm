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
    "w_light": 0.00035064081371168326,
    "w_prox": 0.00007652504295632288,
    "w_slide": 0.0012960190610422663,
    "w_noise": 1.6244235549045687
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
          controller "fluid_swarm"
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
    MAX_SECONDS = 90 # 1.5 minutes 
    max_ticks = int((MAX_SECONDS * 1000) / timestep)
    ticks = 0
    finished_count = 0
    
    announced = set()

    while supervisor.step(timestep) != -1 and ticks < max_ticks:
        
        finished_count = 0

        for i in range(num_robots):
            node = spawned_nodes[i]
            if node is not None:
                pos = node.getPosition()
                if pos[0] > 0.1:
                    finished_count += 1
                    if i not in announced:
                        print("Robot successfully reached the target zone!", flush=True)
                        announced.add(i)

        ticks += 1

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
        
    custom_data_str = ",".join(map(str, params))
    
    print(f"\nStarting Single Simulation ({num_robots}R/{num_holes}H)", flush=True)
    
    time_taken, failed_robots = run_scenario(num_robots, num_holes, custom_data_str)
    
    print(f"Simulation Finished | Time: {time_taken:.1f}s | Failed: {failed_robots}", flush=True)


def single_simulation():
    """Runs a single test with explicitly defined parameters."""
    test_params = list(WEIGHTS.values())
    run_single_simulation(test_params)

# EVALUATION AND OPTIMIZATION

def evaluate_parameters(params, trial=None):
    """
    Evaluates a specific set of parameters across all scenarios
    """
    custom_data_str = f"{params['w_light']},{params['w_prox']},{params['w_slide']},{params['w_noise']}"
    total_score = 0
    step_history = []  # Initialize history list
    
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
        
        # Record history for JSON export
        step_history.append({
            "step": step + 1, 
            "scenario": f"{num_robots}R/{num_holes}H", 
            "time_taken": time_taken, 
            "failed_robots": failed_robots,
            "sub_score": sub_score,
            "accumulated_score": total_score
        })
        
        print(
            f"  Step {step+1}/{len(all_scenarios)} ({num_robots}R/{num_holes}H) | Time: {time_taken:.1f}s | Failed: {failed_robots}",
            flush=True,
        )

        # Hard Kill Switch (Manual Pruning for Failures)
        if failed_robots > 0:
            remaining_scenarios = len(all_scenarios) - (step + 1)
            penalty = remaining_scenarios * 300  
            total_score += penalty
            print(f"  [EARLY STOP] Parameters failed the Gauntlet. Penalty applied.", flush=True)
            
            if trial is not None:
                trial.set_user_attr("step_history", step_history) # Save history before pruning
                raise optuna.TrialPruned()
            else:
                break
                
        # Soft Kill Switch (Optuna MedianPruner for slow but successful runs)
        if trial is not None:
            trial.report(total_score, step) # Give Optuna the data it needs to calculate medians
            if trial.should_prune():
                print(f"  [PRUNED] Trial is slower than the median of previous trials.", flush=True)
                trial.set_user_attr("step_history", step_history) # Save history before pruning
                raise optuna.TrialPruned()

    # Save history for successful completions
    if trial is not None:
        trial.set_user_attr("step_history", step_history)

    print(f"{run_name} Finished -> Total Score: {total_score:.2f}", flush=True)
    return total_score


def optuna_objective(trial):
    """Optuna objective function wrapper."""
    params = {
        "w_light": trial.suggest_float("w_light", 1e-6, 1e-3, log=True),
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

    return study.best_params

def data_collection_mode(params_to_test=None):
    """
    Statistical evaluation using the best parameters found by Optuna.
    If no params are passed, it defaults to the global WEIGHTS.
    """
    print("STARTING DATA COLLECTION EXPERIMENTS")

    if params_to_test is None:
        params_to_test = WEIGHTS
        print("Using global WEIGHTS for data collection.")
    else:
        print("Using newly optimized parameters for data collection.")
    
    custom_data_str = ",".join(str(v) for v in params_to_test.values())
    RUNS_PER_SETUP = 5 # Run each configuration 5 times to average out random spawn luck
    
    experiments = {
        "sweep_robots": [], # Fix holes=2, vary robots 6->12
        "sweep_holes": []   # Fix robots=10, vary holes 1->5
    }

    # The Congestion Test
    print("\n--- Running Experiment A: Sweeping Robot Count (Fixed at 2 Holes) ---")
    for r in range(6, 13): 
        total_time = 0
        total_failures = 0
        
        for run in range(RUNS_PER_SETUP):
            time_taken, failed = run_scenario(r, 2, custom_data_str)
            total_time += time_taken
            total_failures += failed
            
        avg_time = total_time / RUNS_PER_SETUP
        success_rate = ((r * RUNS_PER_SETUP) - total_failures) / (r * RUNS_PER_SETUP) * 100
        
        print(f"  {r} Robots | Avg Time: {avg_time:.1f}s | Success Rate: {success_rate:.1f}%")
        experiments["sweep_robots"].append({"robots": r, "holes": 2, "avg_time": avg_time, "success_rate": success_rate})

    # The Relief Test
    print("\n--- Running Experiment B: Sweeping Hole Count (Fixed at 10 Robots) ---")
    for h in range(1, 6): 
        total_time = 0
        total_failures = 0
        
        for run in range(RUNS_PER_SETUP):
            time_taken, failed = run_scenario(10, h, custom_data_str)
            total_time += time_taken
            total_failures += failed
            
        avg_time = total_time / RUNS_PER_SETUP
        success_rate = ((10 * RUNS_PER_SETUP) - total_failures) / (10 * RUNS_PER_SETUP) * 100
        
        print(f"  {h} Holes | Avg Time: {avg_time:.1f}s | Success Rate: {success_rate:.1f}%")
        experiments["sweep_holes"].append({"robots": 10, "holes": h, "avg_time": avg_time, "success_rate": success_rate})

    # EXPORT DATA
    print("\nExporting final evaluation data to JSON...")
    with open("final_evaluation_data.json", "w") as f:
        json.dump(experiments, f, indent=4)
    print(f"Saved to 'final_evaluation_data.json'")

if __name__ == "__main__":
    # ==========================================
    # TOGGLE THIS VARIABLE TO CHANGE MODES
    # "Single Simulation" : 1
    # "Optimization"      : 2
    # "Data Collection"   : 3
    # "Overnight (2 and 3 together)": 4
    # ==========================================
    EXECUTION_MODE = 4
    
    if EXECUTION_MODE == 1:
        single_simulation()
    elif EXECUTION_MODE == 2:
        optimization_pipeline(n_trials=100)
    elif EXECUTION_MODE == 3:
        data_collection_mode()
    elif EXECUTION_MODE == 4:
        best_found = optimization_pipeline(n_trials=100) 
        data_collection_mode(best_found)
    else:
        print("Invalid EXECUTION_MODE selected.")

    supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)